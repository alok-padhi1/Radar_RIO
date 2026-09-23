"""
estimator/eskf_rio/estimator.py — ESKF Radar-Inertial Odometry Estimator.

Frame contract:
    IMU input:     body FRD (from SCALED_IMU2)
    Radar input:   U300 native -> body FRD via RadarExtrinsics (with axis permutation)
    ESKF state:    NED world frame (position, velocity) / body FRD (biases)
    Output:        NED world frame

Doppler pipeline (Corrective Addendum par 2):
    1. Transform radar points to body FRD via R_B_R
    2. Compute body-frame LOS unit vectors
    3. Pre-compensate lever-arm Doppler (Addendum par 3)
    4. Run robust Doppler solver (RANSAC + IRLS + gates)
    5. Rotate solved velocity to body frame (R_B_R already applied via points)
    6. Feed body-frame velocity to ESKF update

Safety rules:
    - ZUPT is independent of radar rejection
    - Radar rejected != stationary
    - No velocity clamping (Addendum par 9)
"""

import numpy as np
from typing import Optional, List
import logging
import time

from estimator.state import NavigationMode, QualityLevel, RIOState, RadarFrame, IMUSample, AltimeterSample
from estimator.u300_velocity.doppler import RobustDopplerEstimator, DopplerVelocityResult
from estimator.eskf_rio.eskf import ESKF
from estimator.radar_extrinsics import RadarExtrinsics
from config.system import load_config

logger = logging.getLogger(__name__)


class ESKFRIOEstimator:
    """ESKF-based Radar-Inertial Odometry estimator.

    All internal state is in NED/FRD convention.
    Radar points are transformed to body FRD via RadarExtrinsics before Doppler solve.
    Lever-arm is pre-compensated in Doppler space (Addendum par 3).
    """

    def __init__(self,
                 health_manager,
                 config: dict = None):
        self.health_manager = health_manager

        # Load config
        if config is None:
            config = load_config()
        self._config = config

        # Load extrinsics from config
        self.extrinsics = RadarExtrinsics.from_config(config)

        self.eskf = None

        # Robust Doppler solver with full pipeline
        self.doppler_estimator = RobustDopplerEstimator(
            min_points=6,
            max_condition_number=30.0,
            ransac_eps=0.40,
            ransac_iters=60,
            min_inlier_ratio=0.35,
            huber_delta_mps=0.20,
            max_speed_mps=25.0,
            max_sigma_v_mps=0.60,
            max_accel_mps2=15.0,
            deadband_mps=0.05,
        )

        # Initialization state
        self.initialized = False
        self.init_start_time = 0.0
        self.init_duration = 5.0  # seconds of stationary data
        self.init_accel_sum = np.zeros(3)
        self.init_gyro_sum = np.zeros(3)
        self.init_count = 0

        self.last_imu_time = None
        self.last_gyro = np.zeros(3)  # body FRD

        # ZUPT state — independent of radar (correction #6)
        self.accel_var_thresh = 0.05
        self.gyro_var_thresh = 0.01
        self.zupt_cov = np.eye(3) * (0.01 ** 2)
        self.imu_buffer: List[IMUSample] = []
        self._is_stationary = True
        self._stationary_count = 0
        self._radar_log_counter = 0

        # Health tracking — per Addendum par 6, 7
        self._last_radar_accept_time = 0.0
        self._last_radar_frame_time = 0.0

        # RIO state output — all vectors in NED/FRD
        self.current_rio_state = RIOState(
            timestamp=0.0,
            position=np.zeros(3),       # p_NED [N, E, D]
            velocity=np.zeros(3),       # v_NED [vN, vE, vD]
            quaternion=np.array([1.0, 0.0, 0.0, 0.0]),  # q_NB
            bias_accel=np.zeros(3),     # body FRD
            bias_gyro=np.zeros(3),      # body FRD
            radar_points_used=0,
            radar_velocity=np.zeros(3),  # body FRD
            radar_velocity_covariance=np.eye(3).tolist(),
            innovation=np.zeros(3),
            valid=False
        )

    @property
    def is_stationary(self) -> bool:
        """Whether the vehicle is currently detected as stationary (ZUPT active).

        This is determined ONLY by IMU variance, NOT by radar rejection.
        """
        return self._is_stationary

    def _initialize(self, imu: IMUSample) -> bool:
        """Gravity-aligned initialization from stationary FRD IMU data.

        In FRD, stationary accelerometer reads [0, 0, -g] (proper accel is upward).
        Gravity in NED is [0, 0, +g].
        """
        if self.init_count == 0:
            self.init_start_time = imu.timestamp

        accel = np.array([imu.accel_x, imu.accel_y, imu.accel_z])  # body FRD
        gyro = np.array([imu.gyro_x, imu.gyro_y, imu.gyro_z])      # body FRD

        self.init_accel_sum += accel
        self.init_gyro_sum += gyro
        self.init_count += 1

        if imu.timestamp - self.init_start_time > self.init_duration:
            avg_accel = self.init_accel_sum / self.init_count
            avg_gyro = self.init_gyro_sum / self.init_count

            g_norm = np.linalg.norm(avg_accel)

            initial_p = np.zeros(3)     # p_NED
            initial_v = np.zeros(3)     # v_NED
            initial_q = np.array([1.0, 0.0, 0.0, 0.0])  # q_NB (identity = level)
            initial_ba = np.zeros(3)    # body FRD
            initial_bg = avg_gyro       # body FRD — estimate bias from stationary data

            # NED gravity: [0, 0, +g_norm]
            self.eskf = ESKF(initial_p, initial_q, initial_v, initial_ba, initial_bg,
                             gravity=np.array([0.0, 0.0, g_norm]))
            self.initialized = True
            logger.info(f"ESKF Initialized (FRD/NED). g_norm: {g_norm:.4f}, "
                        f"bg: [{initial_bg[0]:.5f}, {initial_bg[1]:.5f}, {initial_bg[2]:.5f}], "
                        f"gravity_NED: [0, 0, +{g_norm:.4f}]")
            return True

        return False

    def _check_zupt(self) -> bool:
        """Check if the vehicle is stationary using IMU variance ONLY.

        ZUPT is independent of radar rejection.
        """
        if len(self.imu_buffer) < 50:
            return False

        accels = np.array([[m.accel_x, m.accel_y, m.accel_z] for m in self.imu_buffer])
        gyros = np.array([[m.gyro_x, m.gyro_y, m.gyro_z] for m in self.imu_buffer])
        accel_var = np.var(np.linalg.norm(accels, axis=1))
        gyro_var = np.var(np.linalg.norm(gyros, axis=1))

        return accel_var < self.accel_var_thresh and gyro_var < self.gyro_var_thresh

    def process_imu(self, imu: IMUSample):
        """Process an IMU sample in body FRD."""
        if not self.initialized:
            self._initialize(imu)
            return

        if self.last_imu_time is None:
            self.last_imu_time = imu.timestamp
            return

        dt = imu.timestamp - self.last_imu_time
        self.last_imu_time = imu.timestamp

        if dt <= 0 or dt > 1.0:
            return

        accel = np.array([imu.accel_x, imu.accel_y, imu.accel_z])  # body FRD
        gyro = np.array([imu.gyro_x, imu.gyro_y, imu.gyro_z])      # body FRD
        self.last_gyro = gyro

        self.eskf.predict(dt, accel, gyro)

        # ZUPT Logic — maintain rolling buffer
        self.imu_buffer.append(imu)
        if len(self.imu_buffer) > 50:
            self.imu_buffer.pop(0)

        is_zupt = self._check_zupt()
        if is_zupt:
            # Apply ZUPT: zero velocity in body FRD
            self.eskf.update_velocity(np.zeros(3), self.zupt_cov, mahalanobis_thresh=10.0)

            # Addendum par 8: Gravity-based attitude update during ZUPT
            # Accelerometer reads gravity direction in body frame
            # Use it to correct roll and pitch
            self.eskf.update_gravity_alignment(accel)

            self._stationary_count += 1
            if self._stationary_count >= 5:
                self._is_stationary = True
        else:
            self._stationary_count = 0
            self._is_stationary = False

        self._update_rio_state(imu.timestamp)

        # Addendum par 9: State validity check (NO clamping)
        v_mag = np.linalg.norm(self.current_rio_state.velocity)
        if v_mag > 25.0:
            logger.error(
                f"VELOCITY DIVERGENCE: {v_mag:.1f} m/s exceeds physical limit. "
                f"State is INVALID. Marking estimator invalid."
            )
            self.current_rio_state.valid = False

    def process_radar(self, frame: RadarFrame):
        """Process a radar frame — Doppler ego-velocity estimation + ESKF update.

        Pipeline (Corrective Addendum par 2, 3):
        1. Extract raw radar-frame points
        2. Transform to body FRD via R_B_R (includes axis permutation)
        3. Pre-compensate lever-arm in Doppler space
        4. Run robust Doppler solver on body-frame data
        5. Feed body-frame velocity to ESKF
        """
        if not self.initialized or self.eskf is None:
            return

        pts = frame.measurements
        self._last_radar_frame_time = time.monotonic()

        if len(pts) < 6:
            self.current_rio_state.radar_points_used = 0
            self._log_radar_frame(len(pts), None, "too_few_raw")
            return

        # Step 1: Extract raw radar-frame measurements
        points_radar = np.array([[m.x_r, m.y_r, m.z_r] for m in pts])
        dopplers = np.array([m.doppler_mps for m in pts])

        # Step 2: Transform points to body FRD via R_B_R
        # This includes the axis permutation (X_lat,Y_fwd,Z_up -> FRD)
        # and the mechanical tilt rotation
        points_body = self.extrinsics.points_radar_to_body(points_radar)

        # Step 3: Pre-compensate lever-arm Doppler (Addendum par 3)
        # d_corrected_i = d_i + u_B_i^T (omega_B x t_B_R)
        omega_body = self.last_gyro - self.eskf.bg  # bias-corrected body FRD
        v_lever = np.cross(omega_body, self.extrinsics.t_B_R)

        # Compute body-frame ranges and LOS for lever-arm compensation
        ranges_body = np.linalg.norm(points_body, axis=1)
        valid_pts = ranges_body > 0.1
        if valid_pts.sum() < 6:
            self.current_rio_state.radar_points_used = 0
            self._log_radar_frame(len(pts), None, "too_few_after_transform")
            return

        u_body = points_body[valid_pts] / ranges_body[valid_pts, np.newaxis]
        dopplers_comp = dopplers[valid_pts] + (u_body @ v_lever)
        points_body_valid = points_body[valid_pts]

        # Step 4: Run robust Doppler solver on body-frame data
        # The solver returns velocity in BODY frame directly
        # (since points are already in body frame)
        res = self.doppler_estimator.estimate(
            points_body_valid, dopplers_comp,
            t_frame=frame.timestamp
        )

        if not res.is_valid:
            self.current_rio_state.radar_points_used = 0
            self.current_rio_state.n_radar_points = len(pts)
            self.current_rio_state.n_static_points = res.num_points_used
            self._log_radar_frame(len(pts), res, res.reason)
            return

        # Step 5: The velocity is already in body FRD (solved from body-frame points)
        # No additional R_B_R rotation needed! The lever-arm is pre-compensated.
        v_body = res.velocity
        P_v_body = res.covariance

        # Step 6: ESKF velocity update
        accepted, reason, inn, S, K, maha = self.eskf.update_velocity(v_body, P_v_body)

        if accepted:
            self._last_radar_accept_time = time.monotonic()
            self.current_rio_state.radar_points_used = res.num_points_used
            self.current_rio_state.radar_velocity = v_body
            self.current_rio_state.radar_velocity_covariance = P_v_body.tolist()
            self.current_rio_state.innovation = inn
            self.current_rio_state.mahalanobis_distance = maha
            self.current_rio_state.n_radar_points = len(pts)
            self.current_rio_state.n_static_points = res.num_points_used
            self.current_rio_state.doppler_residual_rms = float(np.sqrt(np.mean(inn**2))) if inn is not None else 0.0
            self.current_rio_state.valid = True
            if np.linalg.norm(v_body) > 0.3:
                self._is_stationary = False
                self._stationary_count = 0
        else:
            self.current_rio_state.radar_points_used = 0
            self.current_rio_state.n_radar_points = len(pts)
            self.current_rio_state.n_static_points = res.num_points_used

        self._log_radar_frame(len(pts), res, reason if not accepted else "accepted",
                              v_body=v_body, inn=inn, maha=maha, accepted=accepted)

    def _log_radar_frame(self, n_raw: int, res: Optional[DopplerVelocityResult],
                         status: str, v_body=None, inn=None, maha=None,
                         accepted=None):
        """Diagnostic logging per Addendum par 26."""
        self._radar_log_counter += 1
        # Log every frame when moving, every 20th when idle
        v_mag = np.linalg.norm(v_body) if v_body is not None else 0.0
        if v_mag > 0.1 or self._radar_log_counter % 20 == 0:
            v_str = f"v_body=[{v_body[0]:.3f},{v_body[1]:.3f},{v_body[2]:.3f}]" if v_body is not None else "v_body=None"
            inn_str = f"inn=[{inn[0]:.3f},{inn[1]:.3f},{inn[2]:.3f}]" if inn is not None else "inn=None"
            maha_str = f"maha={maha:.2f}" if maha is not None else "maha=None"
            cond_str = f"cond={res.condition_number:.1f}" if res is not None else "cond=N/A"
            pts_str = f"pts={res.num_points_used}" if res is not None else f"pts={n_raw}"
            logger.info(
                f"Radar [{status}] {pts_str} {cond_str} {v_str} {inn_str} {maha_str}"
            )

    def _update_rio_state(self, timestamp: float):
        """Copy ESKF nominal state to RIO output state."""
        if self.eskf is None:
            return
        self.current_rio_state.timestamp = timestamp
        self.current_rio_state.position = self.eskf.p.copy()      # p_NED
        self.current_rio_state.velocity = self.eskf.v.copy()      # v_NED
        self.current_rio_state.quaternion = self.eskf.q.copy()    # q_NB
        self.current_rio_state.bias_accel = self.eskf.ba.copy()   # body FRD
        self.current_rio_state.bias_gyro = self.eskf.bg.copy()    # body FRD

        # Addendum par 9: NO velocity clamping. If velocity diverges,
        # mark state invalid rather than hiding the failure.
        v_mag = np.linalg.norm(self.current_rio_state.velocity)
        if v_mag > 25.0:
            self.current_rio_state.valid = False
