"""
test_eskf.py — Basic ESKF stationary test (FRD/NED convention).

Frame: Body=FRD, World=NED
Stationary FRD accel: [0, 0, -9.80665] (proper accel is upward = -Z in FRD)
NED gravity: [0, 0, +9.80665] (gravity points down = +Z in NED)
Net: R @ [0,0,-g] + [0,0,+g] = [0,0,-g+g] = 0  ✓
"""

import numpy as np
from estimator.eskf_rio.eskf import ESKF


def test_eskf():
    initial_p = np.zeros(3)
    initial_v = np.zeros(3)
    initial_q = np.array([1.0, 0.0, 0.0, 0.0])
    initial_ba = np.zeros(3)
    initial_bg = np.zeros(3)

    # NED convention: gravity = [0, 0, +9.80665]
    eskf = ESKF(initial_p, initial_q, initial_v, initial_ba, initial_bg,
                gravity=np.array([0.0, 0.0, 9.80665]))

    # Predict with stationary FRD accel
    dt = 0.01
    accel = np.array([0.0, 0.0, -9.80665])  # FRD: proper accel upward = -Z
    gyro = np.zeros(3)

    for _ in range(100):
        eskf.predict(dt, accel, gyro)

    print("Test ESKF Predict Stationary (FRD/NED)")
    print(f"Position (NED): {eskf.p}")
    print(f"Velocity (NED): {eskf.v}")
    assert np.allclose(eskf.p, 0.0, atol=1e-3), f"Position drifted: {eskf.p}"
    assert np.allclose(eskf.v, 0.0, atol=1e-3), f"Velocity drifted: {eskf.v}"
    print("Pass!")

    # Test Radar velocity update (body FRD)
    v_body = np.array([0.0, 0.0, 0.0])
    P_v = np.eye(3) * 0.1**2
    accepted, reason, inn, S, K, maha = eskf.update_velocity(v_body, P_v)
    print(f"Test Radar Update: Accepted={accepted}, Reason={reason}")
    assert accepted

    # Test Altimeter update
    agl_m = 0.0
    accepted, reason = eskf.update_altimeter(agl_m)
    print(f"Test Altimeter Update: Accepted={accepted}, Reason={reason}")
    assert accepted

    print("All ESKF tests pass (FRD/NED)!")


if __name__ == "__main__":
    test_eskf()
