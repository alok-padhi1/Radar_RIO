import numpy as np
from estimator.eskf_rio.eskf import ESKF

def test_eskf():
    initial_p = np.zeros(3)
    initial_v = np.zeros(3)
    initial_q = np.array([1.0, 0.0, 0.0, 0.0])
    initial_ba = np.zeros(3)
    initial_bg = np.zeros(3)
    
    eskf = ESKF(initial_p, initial_q, initial_v, initial_ba, initial_bg)
    
    # Predict with no movement
    dt = 0.01
    accel = np.array([0.0, 0.0, -9.80665]) # Z-down
    gyro = np.zeros(3)
    
    for _ in range(100):
        eskf.predict(dt, accel, gyro)
        
    print("Test ESKF Predict Stationary")
    print(f"Position: {eskf.p}")
    print(f"Velocity: {eskf.v}")
    assert np.allclose(eskf.p, 0.0, atol=1e-3)
    assert np.allclose(eskf.v, 0.0, atol=1e-3)
    print("Pass!")
    
    # Test Radar update
    v_R = np.array([0.0, 0.0, 0.0])
    P_v = np.eye(3) * 0.1**2
    accepted, reason, inn, S, K, maha = eskf.update_velocity(v_R, P_v)
    print(f"Test Radar Update: Accepted={accepted}, Reason={reason}")
    assert accepted
    
    # Test Altimeter update
    height_m = 0.0
    accepted, reason = eskf.update_altimeter(height_m)
    print(f"Test Altimeter Update: Accepted={accepted}, Reason={reason}")
    assert accepted
    
    print("All ESKF tests pass!")

if __name__ == "__main__":
    test_eskf()
