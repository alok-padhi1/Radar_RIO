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
    assert hm.mode == NavigationMode.RIO_ONLY
