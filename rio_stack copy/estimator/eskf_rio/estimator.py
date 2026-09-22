import numpy as np
from typing import Optional, List, Dict
import logging
import time

from estimator.state import NavigationMode, QualityLevel, RIOState, RadarFrame, IMUSample, AltimeterSample
from estimator.u300_velocity.doppler import RobustDopplerEstimator, DopplerVelocityResult
from estimator.eskf_rio.eskf import ESKF

logger = logging.getLogger(__name__)

class ESKFRIOEstimator:
    def __init__(self, 
                 health_manager,  # May be None during startup; set later via set_health_manager()
                 lever_arm_radar_to_imu: np.ndarray = np.array([-0.09, -0.145, 0.025])):
        self.health_manager = health_manager
        self.lever_arm = lever_arm_radar_to_imu
        
        self.eskf = None
        
        self.doppler_estimator = RobustDopplerEstimator(min_points=6, max_condition_number=100.0)
        
        # Initialization state
        self.initialized = False
        self.init_start_time = 0.0
        self.init_duration = 5.0
        self.init_accel_sum = np.zeros(3)
        self.init_gyro_sum = np.zeros(3)
        self.init_count = 0
        
        self.last_imu_time = None
        self.last_gyro = np.zeros(3)
        
        # ZUPT state — initialized here so they exist before _initialize() completes
        self.accel_var_thresh = 0.05
        self.gyro_var_thresh = 0.01
        self.zupt_cov = np.eye(3) * (0.01 ** 2)
        self.imu_buffer: List[IMUSample] = []
        self._is_stationary = True  # Start as stationary
        self._stationary_count = 0
        
        # For publishing RIO state
        self.current_rio_state = RIOState(
            timestamp=0.0, position=np.zeros(3), velocity=np.zeros(3),
            quaternion=np.array([1.0, 0.0, 0.0, 0.0]), bias_accel=np.zeros(3), bias_gyro=np.zeros(3),
            radar_points_used=0, radar_velocity=np.zeros(3), radar_velocity_covariance=np.eye(3).tolist(),
            innovation=np.zeros(3),
            valid=False  # Explicitly invalid until first successful update
        )

    def set_health_manager(self, health_manager):
        """Allow health_manager to be set after construction (fixes startup order bug)."""
        self.health_manager = health_manager
        # Load ZUPT thresholds from config if available
        if health_manager is not None:
            self.accel_var_thresh = getattr(getattr(health_manager, "config", None), "accel_var_thresh", 0.05)
            self.gyro_var_thresh = getattr(getattr(health_manager, "config", None), "gyro_var_thresh", 0.01)

    @property
    def is_stationary(self) -> bool:
        """Whether the vehicle is currently detected as stationary (ZUPT active)."""
        return self._is_stationary

    def _initialize(self, imu: IMUSample) -> bool:
        """ stationary initialization """
        if self.init_count == 0:
            self.init_start_time = imu.timestamp
            
        accel = np.array([imu.accel_x, imu.accel_y, imu.accel_z])
        gyro = np.array([imu.gyro_x, imu.gyro_y, imu.gyro_z])
        
        self.init_accel_sum += accel
        self.init_gyro_sum += gyro
        self.init_count += 1
        
        if imu.timestamp - self.init_start_time > self.init_duration:
            avg_accel = self.init_accel_sum / self.init_count
            avg_gyro = self.init_gyro_sum / self.init_count
            
            # Gravity alignment
            g_norm = np.linalg.norm(avg_accel)
            
            initial_p = np.zeros(3)
            initial_v = np.zeros(3)
            # Identity quaternion — assume roughly level for now
            initial_q = np.array([1.0, 0.0, 0.0, 0.0])
            initial_ba = np.zeros(3)
            initial_bg = avg_gyro  # Estimate gyro bias from stationary data
            
            # Load ZUPT configuration from health manager if available
            if self.health_manager is not None:
                self.accel_var_thresh = getattr(getattr(self.health_manager, "config", None), "accel_var_thresh", 0.05)
                self.gyro_var_thresh = getattr(getattr(self.health_manager, "config", None), "gyro_var_thresh", 0.01)
            
            # Determine gravity direction from IMU data.
            # The IMU reads FLU (after FRD→FLU conversion in raw_imu_reader.py).
            # In FLU, when sitting on a table, gravity measured acceleration is [0, 0, +9.81] (Up).
            # In the ESKF world frame, we need g to match: v_dot = R*a + g
            # If IMU reads +9.81 in Z when stationary, and v_dot = 0:
            #   0 = R*[0,0,+9.81] + g  =>  g = [0, 0, -9.81]
            # This means world Z is Up (FLU), gravity points down.
            self.eskf = ESKF(initial_p, initial_q, initial_v, initial_ba, initial_bg,
                             gravity=np.array([0.0, 0.0, -g_norm]))
            self.initialized = True
            logger.info(f"ESKF Initialized. g_norm: {g_norm:.2f}, bg: {initial_bg}, "
                        f"gravity: [0, 0, {-g_norm:.4f}]")
            return True
            
        return False

    def _check_zupt(self) -> bool:
        """Check if the vehicle is stationary using IMU variance."""
        if len(self.imu_buffer) < 50:
            return False
            
        accels = np.array([[m.accel_x, m.accel_y, m.accel_z] for m in self.imu_buffer])
        gyros = np.array([[m.gyro_x, m.gyro_y, m.gyro_z] for m in self.imu_buffer])
        accel_var = np.var(np.linalg.norm(accels, axis=1))
        gyro_var = np.var(np.linalg.norm(gyros, axis=1))
        
        return accel_var < self.accel_var_thresh and gyro_var < self.gyro_var_thresh

    def process_imu(self, imu: IMUSample):
        if not self.initialized:
            self._initialize(imu)
            return
            
        if self.last_imu_time is None:
            self.last_imu_time = imu.timestamp
            return
            
        dt = imu.timestamp - self.last_imu_time
        self.last_imu_time = imu.timestamp
        
        if dt <= 0 or dt > 1.0:  # Reject backward or stale jumps > 1s
            return
            
        accel = np.array([imu.accel_x, imu.accel_y, imu.accel_z])
        gyro = np.array([imu.gyro_x, imu.gyro_y, imu.gyro_z])
        self.last_gyro = gyro
        
        self.eskf.predict(dt, accel, gyro)
        
        # ZUPT Logic — maintain rolling buffer
        self.imu_buffer.append(imu)
        if len(self.imu_buffer) > 50:  # ~0.25 seconds @ 200Hz
            self.imu_buffer.pop(0)
            
        if self._check_zupt():
            # Apply ZUPT (pseudo-measurement of zero velocity)
            self.eskf.update_velocity(np.zeros(3), self.zupt_cov, mahalanobis_thresh=10.0)
            self._stationary_count += 1
            if self._stationary_count >= 5:  # Need sustained detection
                self._is_stationary = True
        else:
            self._stationary_count = 0
            self._is_stationary = False
        
        self._update_rio_state(imu.timestamp)

    def process_radar(self, frame: RadarFrame):
        if not self.initialized or self.eskf is None:
            return
            
        pts = frame.measurements
        if len(pts) < 6:
            return
            
        # Extract matrices
        points = np.array([[m.x_r, m.y_r, m.z_r] for m in pts])
        dopplers = np.array([m.doppler_mps for m in pts])
        
        # Estimate ego velocity
        res = self.doppler_estimator.estimate(
            points, dopplers, 
            angular_velocity=self.last_gyro - self.eskf.bg, 
            lever_arm=self.lever_arm
        )
        
        if not res.is_valid:
            self.current_rio_state.radar_points_used = 0
            return
            
        # Apply ESKF update
        accepted, reason, inn, S, K, maha = self.eskf.update_velocity(res.velocity, res.covariance)
        
        # Log results
        if accepted:
            self.current_rio_state.radar_points_used = res.num_points_used
            self.current_rio_state.radar_velocity = res.velocity
            self.current_rio_state.radar_velocity_covariance = res.covariance.tolist()
            self.current_rio_state.innovation = inn
            self.current_rio_state.mahalanobis_distance = maha
            self.current_rio_state.n_radar_points = len(pts)
            self.current_rio_state.n_static_points = res.num_points_used
            self.current_rio_state.doppler_residual_rms = float(np.sqrt(np.mean(inn**2))) if inn is not None else 0.0
            # Mark RIO as valid after a successful radar update
            self.current_rio_state.valid = True
            # When radar agrees we're moving, clear stationary flag
            if np.linalg.norm(res.velocity) > 0.3:
                self._is_stationary = False
                self._stationary_count = 0
        else:
            self.current_rio_state.radar_points_used = 0
            # Apply ZUPT when radar update is rejected to bound drift
            self.eskf.update_velocity(np.zeros(3), self.zupt_cov, mahalanobis_thresh=10.0)
            
    def _update_rio_state(self, timestamp: float):
        if self.eskf is None:
            return
        self.current_rio_state.timestamp = timestamp
        self.current_rio_state.position = self.eskf.p.copy()
        self.current_rio_state.velocity = self.eskf.v.copy()
        self.current_rio_state.quaternion = self.eskf.q.copy()
        self.current_rio_state.bias_accel = self.eskf.ba.copy()
        self.current_rio_state.bias_gyro = self.eskf.bg.copy()
        
        # Velocity sanity check — clamp unrealistic values
        v_mag = np.linalg.norm(self.current_rio_state.velocity)
        if v_mag > 50.0:  # > 50 m/s is unrealistic for most platforms
            logger.warning(f"Velocity magnitude {v_mag:.1f} m/s exceeds limit, clamping")
            self.current_rio_state.velocity *= 50.0 / v_mag
            self.eskf.v = self.current_rio_state.velocity.copy()
