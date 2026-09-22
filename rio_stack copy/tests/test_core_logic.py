import math
import numpy as np
import pytest

from estimator.state import RadarMeasurement, AltimeterSample, IMUSample, NavigationState, NavigationMode
from estimator.uncertainty import cartesian_from_polar, polar_from_cartesian, jacobian_polar_to_cartesian
from drivers.altimeter.quality_filter import AltimeterQualityFilter
from estimator.health import NavigationHealthManager

def test_polar_cartesian_roundtrip():
    """Verify coordinate transforms."""
    r_orig, theta_orig, phi_orig = 10.0, math.radians(30), math.radians(-10)
    xyz = cartesian_from_polar(r_orig, theta_orig, phi_orig)
    
    r, theta, phi = polar_from_cartesian(*xyz)
    
    assert math.isclose(r, r_orig)
    assert math.isclose(theta, theta_orig)
    assert math.isclose(phi, phi_orig)

def test_altimeter_correction():
    """Verify attitude correction for altimeter."""
    qf = AltimeterQualityFilter()
    
    # Flat
    assert math.isclose(qf.correct_agl(10.0, 0.0, 0.0), 10.0)
    
    # 45 deg roll -> cos(45) = 0.707
    assert math.isclose(qf.correct_agl(10.0, math.radians(45), 0.0), 10.0 * 0.707106, rel_tol=1e-4)

def test_health_manager_transitions():
    """Verify state machine logic."""
    hm = NavigationHealthManager()
    
    assert hm.mode == NavigationMode.INITIALIZING
    
    # Provide healthy sensor data
    hm.update_radar_health(10, 5) # 10 points, 5 static
    hm.update_imu_health(50.0, 0.1, 9.8) # 50Hz, small gyro, 1G accel
    
    # Mock RIO state
    from estimator.state import RIOState
    rio = RIOState(timestamp=0, valid=True, n_static_points=5)
    
    hm.update_state_machine(rio=rio)
    # After first update with valid RIO data, transitions to RIO_IMU_ONLY
    assert hm.mode == NavigationMode.RIO_IMU_ONLY

def test_navigation_mode_enum_values():
    """Verify that all expected NavigationMode values exist."""
    # These are all the modes referenced across the codebase
    assert NavigationMode.INITIALIZING.value == "INITIALIZING"
    assert NavigationMode.RIO_IMU_ONLY.value == "RIO_IMU_ONLY"
    assert NavigationMode.RIO_ONLY.value == "RIO_ONLY"
    assert NavigationMode.RADAR_VELOCITY_GOOD.value == "RADAR_VELOCITY_GOOD"
    assert NavigationMode.RADAR_VELOCITY_DEGRADED.value == "RADAR_VELOCITY_DEGRADED"
    assert NavigationMode.SLAM_GOOD.value == "SLAM_GOOD"
    assert NavigationMode.SLAM_DEGRADED.value == "SLAM_DEGRADED"
    assert NavigationMode.RECOVERY.value == "RECOVERY"
    assert NavigationMode.INVALID.value == "INVALID"

def test_eskf_stationary_zero_distance():
    """Verify that the ESKF estimator reports zero velocity when stationary."""
    from estimator.eskf_rio.estimator import ESKFRIOEstimator
    
    est = ESKFRIOEstimator(health_manager=None)
    
    # Simulate stationary IMU data for initialization (>5 seconds)
    t = 0.0
    dt = 0.01  # 100Hz
    for i in range(600):  # 6 seconds of init data
        sample = IMUSample(
            timestamp=t,
            accel_x=0.0, accel_y=0.0, accel_z=9.81,  # FLU: gravity is +Z
            gyro_x=0.0, gyro_y=0.0, gyro_z=0.0,
            valid=True
        )
        est.process_imu(sample)
        t += dt
    
    assert est.initialized, "ESKF should be initialized after 6s of data"
    
    # Now feed more stationary IMU data
    for i in range(200):
        sample = IMUSample(
            timestamp=t,
            accel_x=0.0, accel_y=0.0, accel_z=9.81,
            gyro_x=0.0, gyro_y=0.0, gyro_z=0.0,
            valid=True
        )
        est.process_imu(sample)
        t += dt
    
    # Velocity should be near zero
    v_mag = np.linalg.norm(est.current_rio_state.velocity)
    assert v_mag < 0.5, f"Stationary velocity magnitude should be near zero, got {v_mag:.3f} m/s"
    
    # Position should be near zero
    p_mag = np.linalg.norm(est.current_rio_state.position)
    assert p_mag < 5.0, f"Stationary position magnitude should be near zero, got {p_mag:.3f} m"
