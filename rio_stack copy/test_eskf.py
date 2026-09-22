import numpy as np
from estimator.eskf_rio.eskf import ESKF

def test_eskf():
    initial_p = np.zeros(3)
    initial_v = np.zeros(3)
    initial_q = np.array([1.0, 0.0, 0.0, 0.0])
    initial_ba = np.zeros(3)
    initial_bg = np.zeros(3)
    
    # FLU convention: accelerometer reads [0, 0, +9.81] when stationary
    # (proper acceleration is upward, opposing gravity)
    # Gravity vector is [0, 0, -9.81] (points downward in FLU)
    # The ESKF prediction is: v_dot = R*a + g = [0,0,+9.81] + [0,0,-9.81] = 0
    eskf = ESKF(initial_p, initial_q, initial_v, initial_ba, initial_bg,
                gravity=np.array([0.0, 0.0, -9.80665]))
    
    # Predict with no movement — FLU stationary accel
    dt = 0.01
    accel = np.array([0.0, 0.0, 9.80665])  # FLU: proper accel is +Z (Up)
    gyro = np.zeros(3)
    
    for _ in range(100):
        eskf.predict(dt, accel, gyro)
        
    print("Test ESKF Predict Stationary (FLU)")
    print(f"Position: {eskf.p}")
    print(f"Velocity: {eskf.v}")
    assert np.allclose(eskf.p, 0.0, atol=1e-3), f"Position drifted: {eskf.p}"
    assert np.allclose(eskf.v, 0.0, atol=1e-3), f"Velocity drifted: {eskf.v}"
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
