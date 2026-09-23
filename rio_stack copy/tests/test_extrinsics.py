"""
tests/test_extrinsics.py — Radar extrinsics and lever-arm tests.

Tests the TiltMount-based R_B_R computation and lever-arm formulas.
"""

import numpy as np
import pytest
from estimator.radar_extrinsics import RadarExtrinsics, AltimeterExtrinsics


def test_identity_transform():
    """Identity extrinsics: radar frame = body frame."""
    ext = RadarExtrinsics.identity()
    v_radar = np.array([1.0, 2.0, 3.0])
    v_body = ext.velocity_radar_to_body(v_radar, np.zeros(3))
    assert np.allclose(v_body, v_radar)


def test_axis_permutation_forward():
    """U300 Y_radar (forward/boresight) should map to body X (forward) at 0 tilt."""
    R = RadarExtrinsics._build_R_B_R(tilt_deg=0.0, lateral_sign=1.0)
    # Radar Y axis (forward) → body X axis (forward)
    v_radar_fwd = np.array([0.0, 1.0, 0.0])
    v_body = R @ v_radar_fwd
    assert abs(v_body[0] - 1.0) < 0.01, f"v_body={v_body}, expected X=1"
    assert abs(v_body[1]) < 0.01, f"v_body Y should be ~0, got {v_body[1]}"
    assert abs(v_body[2]) < 0.01, f"v_body Z should be ~0, got {v_body[2]}"


def test_axis_permutation_lateral():
    """U300 X_radar (lateral) should map to body Y (right) at 0 tilt."""
    R = RadarExtrinsics._build_R_B_R(tilt_deg=0.0, lateral_sign=1.0)
    v_radar_lat = np.array([1.0, 0.0, 0.0])
    v_body = R @ v_radar_lat
    assert abs(v_body[1] - 1.0) < 0.01, f"v_body={v_body}, expected Y=1"


def test_axis_permutation_vertical():
    """U300 Z_radar (up) should map to body -Z (down) at 0 tilt."""
    R = RadarExtrinsics._build_R_B_R(tilt_deg=0.0, lateral_sign=1.0)
    v_radar_up = np.array([0.0, 0.0, 1.0])
    v_body = R @ v_radar_up
    assert abs(v_body[2] - (-1.0)) < 0.01, f"v_body={v_body}, expected Z=-1"


def test_50deg_tilt_boresight():
    """At 50 deg tilt, radar boresight (Y_radar) should point 50 deg below body forward."""
    R = RadarExtrinsics._build_R_B_R(tilt_deg=50.0, lateral_sign=1.0)
    v_radar_fwd = np.array([0.0, 1.0, 0.0])
    v_body = R @ v_radar_fwd
    # cos(50) in X, sin(50) in Z
    import math
    assert abs(v_body[0] - math.cos(math.radians(50))) < 0.01
    assert abs(v_body[2] - math.sin(math.radians(50))) < 0.01
    assert abs(v_body[1]) < 0.01  # No lateral component


def test_lever_arm_pure_yaw():
    """Pure yaw with lever arm should produce cross-product velocity."""
    t_B_R = np.array([0.16, 0.0, 0.0])  # radar 16cm forward
    ext = RadarExtrinsics(R_B_R=np.eye(3), t_B_R=t_B_R)

    v_radar = np.zeros(3)
    omega_body = np.array([0.0, 0.0, 1.0])  # 1 rad/s yaw right

    v_body = ext.velocity_radar_to_body(v_radar, omega_body)
    expected = -np.cross(omega_body, t_B_R)
    assert np.allclose(v_body, expected, atol=1e-6), f"v_body={v_body} vs expected={expected}"


def test_lever_arm_pure_pitch():
    """Pure pitch with lever arm."""
    t_B_R = np.array([0.16, 0.0, 0.07])
    ext = RadarExtrinsics(R_B_R=np.eye(3), t_B_R=t_B_R)

    omega_body = np.array([0.0, 0.5, 0.0])
    v_body = ext.velocity_radar_to_body(np.zeros(3), omega_body)
    expected = -np.cross(omega_body, t_B_R)
    assert np.allclose(v_body, expected, atol=1e-6)


def test_lever_arm_pure_roll():
    """Pure roll with lever arm."""
    t_B_R = np.array([0.16, 0.05, 0.07])
    ext = RadarExtrinsics(R_B_R=np.eye(3), t_B_R=t_B_R)

    omega_body = np.array([0.5, 0.0, 0.0])
    v_body = ext.velocity_radar_to_body(np.zeros(3), omega_body)
    expected = -np.cross(omega_body, t_B_R)
    assert np.allclose(v_body, expected, atol=1e-6)


def test_rotate_velocity_no_lever():
    """rotate_velocity_to_body should rotate without lever-arm."""
    R = RadarExtrinsics._build_R_B_R(tilt_deg=0.0)
    ext = RadarExtrinsics(R_B_R=R, t_B_R=np.array([0.16, 0.0, 0.0]))

    v_radar = np.array([0.0, 1.0, 0.0])  # forward in radar
    v_body = ext.rotate_velocity_to_body(v_radar)
    # Should be [1, 0, 0] in body (forward), NO lever-arm
    assert abs(v_body[0] - 1.0) < 0.01


def test_points_transform():
    """Point cloud radar->body transform with identity R."""
    t_B_R = np.array([0.16, 0.05, 0.07])
    ext = RadarExtrinsics(R_B_R=np.eye(3), t_B_R=t_B_R)

    pts_radar = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    pts_body = ext.points_radar_to_body(pts_radar)
    expected = pts_radar + t_B_R
    assert np.allclose(pts_body, expected, atol=1e-6)


def test_config_loading():
    """Load extrinsics from a config dict."""
    config = {
        'extrinsics': {
            'radar_to_body': {
                'translation_m': [0.16, 0.05, 0.07],
                'rotation_rpy_deg': [0.0, 50.0, 0.0],
            },
            'altimeter_to_body': {
                'translation_m': [0.0, 0.0, 0.10],
                'beam_axis_body': [0.0, 0.0, 1.0],
            }
        }
    }
    ext = RadarExtrinsics.from_config(config)
    assert not ext.calibrated
    assert np.allclose(ext.t_B_R, [0.16, 0.05, 0.07])
    assert ext.tilt_deg == 50.0

    alt = AltimeterExtrinsics.from_config(config)
    assert not alt.calibrated
    assert np.allclose(alt.beam_axis_body, [0.0, 0.0, 1.0])


def test_R_B_R_determinant():
    """R_B_R should be a proper rotation (det = +1) with lateral_sign=+1."""
    R = RadarExtrinsics._build_R_B_R(tilt_deg=50.0, lateral_sign=1.0)
    det = np.linalg.det(R)
    assert abs(det - 1.0) < 1e-6, f"det(R_B_R) = {det}, expected +1"


def test_R_B_R_reflection_with_negative_lateral():
    """R_B_R with lateral_sign=-1 should be a reflection (det = -1)."""
    R = RadarExtrinsics._build_R_B_R(tilt_deg=50.0, lateral_sign=-1.0)
    det = np.linalg.det(R)
    assert abs(det - (-1.0)) < 1e-6, f"det(R_B_R) = {det}, expected -1"
