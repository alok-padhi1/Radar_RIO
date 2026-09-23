"""
estimator/eskf_rio/estimator.py — ESKF Radar-Inertial Odometry Estimator.

Frame contract:
    IMU input:     body FRD (from SCALED_IMU2, no conversion)
    Radar input:   U300 radar frame → body FRD via RadarExtrinsics
    ESKF state:    NED world frame (position, velocity) / body FRD (biases)
    Output:        NED world frame

Safety rules:
    - ZUPT is independent of radar rejection (Master Plan §23, correction #6)
    - Radar rejected ≠ stationary — only IMU variance detector triggers ZUPT
    - Doppler validity requires: min points + rank 3 + conditioning + residual + temporal consistency
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
    Radar velocity is transformed to body FRD via RadarExtrinsics before fusion.
    """

    def __init__(self,
                 health_manager,
                 config: dict = None):
        self.health_manager = health_manager

        # Load config
        if config is None:
            config = load_config()
        self._config = config

        # Load extrinsics from config — PLACEHOLDER until calibrated
        self.extrinsics = RadarExtrinsics.from_config(config)

        self.eskf = None

        # Doppler solver — correction #7: requires min points + rank + conditioning
        self.doppler_estimator = RobustDopplerEstimator(
            min_points=6,
            max_condition_number=100.0
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
            # FRD stationary accel: [0, 0, -g_norm]
            # Check: R_NB @ [0,0,-g] + [0,0,+g] = [0,0,-g+g] = 0  ✓
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

        Master Plan §23: ZUPT is independent of radar rejection.
        radar rejected ≠ stationary.
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

        if self._check_zupt():
            # Apply ZUPT: pseudo-measurement of zero velocity in body FRD
            self.eskf.update_velocity(np.zeros(3), self.zupt_cov, mahalanobis_thresh=10.0)
            self._stationary_count += 1
            if self._stationary_count >= 5:
                self._is_stationary = True
        else:
            self._stationary_count = 0
            self._is_stationary = False

        self._update_rio_state(imu.timestamp)

    def process_radar(self, frame: RadarFrame):
        """Process a radar frame — Doppler ego-velocity estimation + ESKF update.

        Correction #7: Doppler validity requires min points + rank 3 +
        conditioning + residual gate + temporal consistency.
        """
        if not self.initialized or self.eskf is None:
            return

        pts = frame.measurements
        if len(pts) < 6:
            # Not enough points — measurement unavailable
            # Do NOT set velocity to zero (safety rule #6)
            self.current_rio_state.radar_points_used = 0
            return

        # Extract radar-frame measurements
        points = np.array([[m.x_r, m.y_r, m.z_r] for m in pts])
        dopplers = np.array([m.doppler_mps for m in pts])

        # Body angular velocity (bias-corrected) for lever-arm compensation
        omega_body = self.last_gyro - self.eskf.bg  # body FRD

        # Estimate ego velocity in RADAR frame
        res = self.doppler_estimator.estimate(
            points, dopplers,
            angular_velocity=omega_body,
            lever_arm=self.extrinsics.get_lever_arm()
        )

        if not res.is_valid:
            # Measurement unavailable — IMU propagates, covariance grows
            # Do NOT apply ZUPT here (correction #6: radar rejected ≠ stationary)
            self.current_rio_state.radar_points_used = 0
            self.current_rio_state.n_radar_points = len(pts)
            self.current_rio_state.n_static_points = res.num_points_used
            return

        # Transform radar velocity to body-origin velocity in body FRD
        v_body = self.extrinsics.velocity_radar_to_body(res.velocity, omega_body)

        # Transform covariance to body frame
        R_BR = self.extrinsics.R_B_R
        P_v_body = R_BR @ res.covariance @ R_BR.T

        # Apply ESKF velocity update
        accepted, reason, inn, S, K, maha = self.eskf.update_velocity(v_body, P_v_body)

        if accepted:
            self.current_rio_state.radar_points_used = res.num_points_used
            self.current_rio_state.radar_velocity = v_body  # body FRD
            self.current_rio_state.radar_velocity_covariance = P_v_body.tolist()
            self.current_rio_state.innovation = inn
            self.current_rio_state.mahalanobis_distance = maha
            self.current_rio_state.n_radar_points = len(pts)
            self.current_rio_state.n_static_points = res.num_points_used
            self.current_rio_state.doppler_residual_rms = float(np.sqrt(np.mean(inn**2))) if inn is not None else 0.0
            self.current_rio_state.valid = True
            # When radar measures significant velocity, clear stationary flag
            if np.linalg.norm(v_body) > 0.3:
                self._is_stationary = False
                self._stationary_count = 0
        else:
            self.current_rio_state.radar_points_used = 0
            self.current_rio_state.n_radar_points = len(pts)
            self.current_rio_state.n_static_points = res.num_points_used
            # Radar rejected — DO NOT apply ZUPT (correction #6)
            # IMU continues propagating, covariance grows naturally

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

        # Velocity sanity check
        v_mag = np.linalg.norm(self.current_rio_state.velocity)
        if v_mag > 50.0:
            logger.warning(f"Velocity magnitude {v_mag:.1f} m/s exceeds limit, clamping")
            self.current_rio_state.velocity *= 50.0 / v_mag
            self.eskf.v = self.current_rio_state.velocity.copy()
