import numpy as np
from typing import Optional, List, Dict
import logging
import time

from estimator.state import NavigationMode, QualityLevel, RIOState, RadarFrame, IMUSample, AltimeterSample
from estimator.u300_velocity.doppler import RobustDopplerEstimator, DopplerVelocityResult
from estimator.eskf_rio.eskf import ESKF
from estimator.health import NavigationHealthManager

logger = logging.getLogger(__name__)

class ESKFRIOEstimator:
    def __init__(self, 
                 health_manager: NavigationHealthManager,
                 lever_arm_radar_to_imu: np.ndarray = np.array([-0.09, -0.145, 0.025])): # default approx
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
        
        # For publishing RIO state
        self.current_rio_state = RIOState(
            timestamp=0.0, position=np.zeros(3), velocity=np.zeros(3),
            quaternion=np.array([1.0, 0.0, 0.0, 0.0]), bias_accel=np.zeros(3), bias_gyro=np.zeros(3),
            radar_points_used=0, radar_velocity=np.zeros(3), radar_velocity_covariance=np.eye(3).tolist(),
            innovation=np.zeros(3)
        )

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
            
            # Simple gravity alignment (assuming stationary, Z-down)
            g_norm = np.linalg.norm(avg_accel)
            g_dir = avg_accel / g_norm
            
            # Z axis is -g_dir (since gravity points UP in body frame when sitting on table)
            # Actually, proper acceleration is UP (-Z in FRD, +Z in FLU)
            # If Z is Down, proper accel is UP, so measured accel is [0, 0, -9.8] in Z-down body frame
            
            initial_p = np.zeros(3)
            initial_v = np.zeros(3)
            initial_q = np.array([1.0, 0.0, 0.0, 0.0]) # Identity (assume roughly level for now)
            initial_ba = np.zeros(3)
            initial_bg = avg_gyro
            
            # Initialize ZUPT thresholds
            self.accel_var_thresh = 0.05
            self.gyro_var_thresh = 0.01
            
            # ZUPT buffer
            self.imu_buffer = []
            
            self.eskf = ESKF(initial_p, initial_q, initial_v, initial_ba, initial_bg)
            self.initialized = True
            logger.info(f"ESKF Initialized. g_norm: {g_norm:.2f}, bg: {initial_bg}")
            return True
            
        return False

    def process_imu(self, imu: IMUSample):
        if not self.initialized:
            self._initialize(imu)
            return
            
        if self.last_imu_time is None:
            self.last_imu_time = imu.timestamp
            return
            
        dt = imu.timestamp - self.last_imu_time
        self.last_imu_time = imu.timestamp
        
        if dt <= 0:
            return
            
        accel = np.array([imu.accel_x, imu.accel_y, imu.accel_z])
        gyro = np.array([imu.gyro_x, imu.gyro_y, imu.gyro_z])
        self.last_gyro = gyro
        
        self.eskf.predict(dt, accel, gyro)
        
        # ZUPT Logic
        self.imu_buffer.append(imu)
        if len(self.imu_buffer) > 50: # 0.25 seconds @ 200Hz
            self.imu_buffer.pop(0)
            
        if len(self.imu_buffer) == 50:
            accels = np.array([[m.accel_x, m.accel_y, m.accel_z] for m in self.imu_buffer])
            gyros = np.array([[m.gyro_x, m.gyro_y, m.gyro_z] for m in self.imu_buffer])
            accel_var = np.var(np.linalg.norm(accels, axis=1))
            gyro_var = np.var(np.linalg.norm(gyros, axis=1))
            
            if accel_var < self.accel_var_thresh and gyro_var < self.gyro_var_thresh:
                # Apply ZUPT (pseudo-measurement of zero velocity)
                zv_cov = np.eye(3) * 0.01**2
                self.eskf.update_velocity(np.zeros(3), zv_cov, mahalanobis_thresh=10.0)
        
        self._update_rio_state(imu.timestamp)

    def process_radar(self, frame: RadarFrame):
        if not self.initialized or self.eskf is None:
            return
            
        pts = frame.measurements
        if len(pts) < 6:
            # logger.warning(f"Radar dropped: {len(pts)} points < 6")
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
            # logger.warning(f"Radar update invalid: {res.reason}")
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
        else:
            self.current_rio_state.radar_points_used = 0
            
    def _update_rio_state(self, timestamp: float):
        if self.eskf is None:
            return
        self.current_rio_state.timestamp = timestamp
        self.current_rio_state.position = self.eskf.p.copy()
        self.current_rio_state.velocity = self.eskf.v.copy()
        self.current_rio_state.quaternion = self.eskf.q.copy()
        self.current_rio_state.bias_accel = self.eskf.ba.copy()
        self.current_rio_state.bias_gyro = self.eskf.bg.copy()
