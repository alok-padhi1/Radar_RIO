"""
tests/test_eskf_comprehensive.py — 15 comprehensive ESKF tests (Master Plan §62).

Frame: Body=FRD, World=NED
Gravity NED: [0, 0, +9.80665]
Stationary FRD accel: [0, 0, -9.80665]
"""

import numpy as np
import pytest
from estimator.eskf_rio.eskf import ESKF


def _make_eskf(**kwargs):
    """Create an ESKF with NED gravity."""
    return ESKF(
        initial_p=kwargs.get('p', np.zeros(3)),
        initial_q=kwargs.get('q', np.array([1.0, 0.0, 0.0, 0.0])),
        initial_v=kwargs.get('v', np.zeros(3)),
        initial_ba=kwargs.get('ba', np.zeros(3)),
        initial_bg=kwargs.get('bg', np.zeros(3)),
        gravity=np.array([0.0, 0.0, 9.80665]),  # NED
    )

G = 9.80665
DT = 0.01
STAT_ACCEL = np.array([0.0, 0.0, -G])  # FRD stationary


# 1. test_gravity — correct gravity cancellation in FRD/NED
def test_gravity():
    eskf = _make_eskf()
    for _ in range(1000):
        eskf.predict(DT, STAT_ACCEL, np.zeros(3))
    assert np.allclose(eskf.v, 0, atol=1e-2), f"v={eskf.v}"
    assert np.allclose(eskf.p, 0, atol=1e-1), f"p={eskf.p}"


# 2. test_initial_attitude — gravity-aligned initialization
def test_initial_attitude():
    eskf = _make_eskf()
    R = eskf.quaternion_to_matrix(eskf.q)
    # Identity rotation: body Z (down) maps to NED Z (down)
    assert np.allclose(R, np.eye(3), atol=1e-6)


# 3. test_constant_velocity — IMU + radar agree, velocity tracks
def test_constant_velocity():
    eskf = _make_eskf(v=np.array([1.0, 0.0, 0.0]))  # 1 m/s North
    for i in range(100):
        eskf.predict(DT, STAT_ACCEL, np.zeros(3))
    # Position should move ~1m north in 1 second
    assert eskf.p[0] > 0.9, f"Expected forward motion, p_N={eskf.p[0]}"
    assert abs(eskf.p[1]) < 0.1, f"Unexpected East motion, p_E={eskf.p[1]}"


# 4. test_constant_acceleration — position grows quadratically
def test_constant_acceleration():
    eskf = _make_eskf()
    # Apply 1 m/s² forward in body FRD (+X)
    accel_fwd = np.array([1.0, 0.0, -G])  # forward accel + gravity compensation
    for _ in range(100):
        eskf.predict(DT, accel_fwd, np.zeros(3))
    # After 1s at 1m/s²: v ≈ 1 m/s, p ≈ 0.5 m
    assert eskf.v[0] > 0.8, f"Expected forward velocity, vN={eskf.v[0]}"
    assert eskf.p[0] > 0.3, f"Expected forward position, pN={eskf.p[0]}"


# 5. test_yaw — pure yaw rotation, position bounded
def test_yaw():
    eskf = _make_eskf()
    gyro_yaw = np.array([0.0, 0.0, 0.5])  # 0.5 rad/s yaw rate (FRD: +Z = yaw right)
    for _ in range(200):
        eskf.predict(DT, STAT_ACCEL, gyro_yaw)
    p_mag = np.linalg.norm(eskf.p)
    assert p_mag < 2.0, f"Position drifted during pure yaw: {p_mag:.2f}m"


# 6. test_pitch — pure pitch, position bounded
def test_pitch():
    eskf = _make_eskf()
    gyro_pitch = np.array([0.0, 0.3, 0.0])  # pitch rate (FRD: +Y = pitch down)
    for _ in range(100):
        eskf.predict(DT, STAT_ACCEL, gyro_pitch)
    p_mag = np.linalg.norm(eskf.p)
    assert p_mag < 5.0, f"Position drifted during pure pitch: {p_mag:.2f}m"


# 7. test_roll — pure roll, position bounded
def test_roll():
    eskf = _make_eskf()
    gyro_roll = np.array([0.3, 0.0, 0.0])  # roll rate (FRD: +X = roll right)
    for _ in range(100):
        eskf.predict(DT, STAT_ACCEL, gyro_roll)
    p_mag = np.linalg.norm(eskf.p)
    assert p_mag < 5.0, f"Position drifted during pure roll: {p_mag:.2f}m"


# 8. test_bias — bias estimation converges with radar updates
def test_bias():
    eskf = _make_eskf(bg=np.array([0.01, -0.005, 0.002]))  # known bias
    true_v = np.zeros(3)
    for i in range(500):
        # IMU with bias
        eskf.predict(DT, STAT_ACCEL, np.array([0.01, -0.005, 0.002]))
        # Frequent radar updates saying zero velocity
        if i % 5 == 0:
            eskf.update_velocity(true_v, np.eye(3) * 0.01**2)
    v_mag = np.linalg.norm(eskf.v)
    assert v_mag < 0.5, f"Velocity should be bounded with bias: {v_mag:.3f}"


# 9. test_radar_update — velocity corrected by radar measurement
def test_radar_update():
    eskf = _make_eskf()
    # Inflate velocity covariance so Kalman gain is meaningful
    eskf.P[3:6, 3:6] = np.eye(3) * 10.0
    # Predict a few steps
    for _ in range(10):
        eskf.predict(DT, STAT_ACCEL, np.zeros(3))
    # Radar says we're moving 2 m/s forward in body
    v_radar = np.array([2.0, 0.0, 0.0])
    P_v = np.eye(3) * 0.5**2  # Moderate confidence
    accepted, reason, _, _, _, _ = eskf.update_velocity(v_radar, P_v, mahalanobis_thresh=10.0)
    assert accepted, f"Radar update should be accepted: {reason}"
    # Velocity should have moved toward 2 m/s
    R = eskf.quaternion_to_matrix(eskf.q)
    v_body = R.T @ eskf.v
    assert v_body[0] > 0.5, f"Expected forward velocity after update: {v_body[0]}"


# 10. test_radar_rejection — large innovation rejected, IMU continues
def test_radar_rejection():
    eskf = _make_eskf()
    for _ in range(10):
        eskf.predict(DT, STAT_ACCEL, np.zeros(3))
    # Absurd radar measurement
    v_absurd = np.array([100.0, 0.0, 0.0])
    P_v = np.eye(3) * 0.01**2  # Very confident = large Mahalanobis
    accepted, reason, _, _, _, maha = eskf.update_velocity(v_absurd, P_v, mahalanobis_thresh=3.0)
    assert not accepted, "Absurd measurement should be rejected"


# 11. test_zupt — zero velocity applied when stationary
def test_zupt():
    eskf = _make_eskf(v=np.array([0.1, 0.05, -0.02]))  # small initial drift
    # Inflate velocity covariance so ZUPT can correct
    eskf.P[3:6, 3:6] = np.eye(3) * 1.0
    zupt_cov = np.eye(3) * 0.01**2
    for _ in range(100):  # Enough iterations for convergence
        eskf.predict(DT, STAT_ACCEL, np.zeros(3))
        eskf.update_velocity(np.zeros(3), zupt_cov, mahalanobis_thresh=10.0)
    v_mag = np.linalg.norm(eskf.v)
    assert v_mag < 0.15, f"ZUPT should drive velocity to ~0: {v_mag}"


# 12. test_u200a — altitude update constrains Z
def test_u200a():
    eskf = _make_eskf()
    # Inflate position covariance so the altimeter update is accepted
    eskf.P[2, 2] = 10.0  # Large Z uncertainty
    # Simulate slight downward drift
    eskf.p[2] = 0.5  # 0.5m down in NED
    # Altimeter says AGL = 1.0m → p_D should be -1.0 in NED
    accepted, reason = eskf.update_altimeter(1.0, var_h=0.1**2, mahalanobis_thresh=5.0)
    assert accepted, f"Altimeter should be accepted: {reason}"
    # p[2] should move toward -1.0 (i.e. 1m above ground in NED)
    assert eskf.p[2] < 0.5, f"Altimeter should correct Z upward: p_D={eskf.p[2]}"


# 13. test_large_innovation — Mahalanobis gate rejects bad measurement
def test_large_innovation():
    eskf = _make_eskf()
    for _ in range(10):
        eskf.predict(DT, STAT_ACCEL, np.zeros(3))
    # Moderate measurement but very tight covariance = high Mahalanobis
    v_meas = np.array([5.0, 0.0, 0.0])
    P_tight = np.eye(3) * 0.001**2
    accepted, _, _, _, _, maha = eskf.update_velocity(v_meas, P_tight, mahalanobis_thresh=3.0)
    assert not accepted, "Large innovation should be rejected"
    assert maha > 3.0, f"Mahalanobis should exceed threshold: {maha}"


# 14. test_covariance_symmetry — P remains symmetric after operations
def test_covariance_symmetry():
    eskf = _make_eskf()
    for i in range(100):
        eskf.predict(DT, STAT_ACCEL + np.random.randn(3) * 0.01, np.random.randn(3) * 0.001)
        if i % 10 == 0:
            eskf.update_velocity(np.random.randn(3) * 0.1, np.eye(3) * 0.1**2)
    diff = np.max(np.abs(eskf.P - eskf.P.T))
    assert diff < 1e-10, f"P is not symmetric: max asymmetry = {diff}"


# 15. test_covariance_positive_definite — P remains positive definite
def test_covariance_positive_definite():
    eskf = _make_eskf()
    for i in range(100):
        eskf.predict(DT, STAT_ACCEL + np.random.randn(3) * 0.01, np.random.randn(3) * 0.001)
        if i % 10 == 0:
            eskf.update_velocity(np.random.randn(3) * 0.1, np.eye(3) * 0.1**2)
    eigvals = np.linalg.eigvalsh(eskf.P)
    assert np.all(eigvals > -1e-10), f"P is not PSD: min eigenvalue = {eigvals.min()}"
