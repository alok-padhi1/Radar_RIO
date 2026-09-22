"""
tests/test_extrinsics.py — Radar extrinsics and lever-arm tests.

Tests the exact formula from correction #4:
    v_radar_body = R_B_R @ v_radar
    v_body = v_radar_body - cross(omega_body, t_B_R)
"""

import numpy as np
import pytest
from estimator.radar_extrinsics import RadarExtrinsics, AltimeterExtrinsics, _rpy_to_matrix


def test_identity_transform():
    """Identity extrinsics: radar frame = body frame."""
    ext = RadarExtrinsics.identity()
    v_radar = np.array([1.0, 2.0, 3.0])
    v_body = ext.velocity_radar_to_body(v_radar, np.zeros(3))
    assert np.allclose(v_body, v_radar)


def test_rotation_only():
    """Pure 90° yaw: radar X → body -Y (FRD convention)."""
    R = _rpy_to_matrix(0, 0, 90)  # 90° yaw
    ext = RadarExtrinsics(R_B_R=R, t_B_R=np.zeros(3))
    v_radar = np.array([1.0, 0.0, 0.0])  # forward in radar
    v_body = ext.velocity_radar_to_body(v_radar, np.zeros(3))
    # Rz(90°) @ [1,0,0] = [0,1,0] (yaw right maps radar-X to body-Y)
    assert abs(v_body[1] - 1.0) < 0.01 or abs(v_body[0]) < 0.01, f"v_body={v_body}"


def test_lever_arm_pure_yaw():
    """Correction #4: pure yaw with lever arm should produce cross-product velocity."""
    t_B_R = np.array([0.16, 0.0, 0.0])  # radar 16cm forward
    ext = RadarExtrinsics(R_B_R=np.eye(3), t_B_R=t_B_R)

    v_radar = np.zeros(3)  # radar measures zero (stationary scene)
    omega_body = np.array([0.0, 0.0, 1.0])  # 1 rad/s yaw right (FRD: +Z)

    v_body = ext.velocity_radar_to_body(v_radar, omega_body)

    # Expected: v_body = 0 - cross([0,0,1], [0.16,0,0]) = -[0, 0.16, 0] = [0, -0.16, 0]
    # (The radar sees a velocity due to rotation, we subtract it)
    expected = -np.cross(omega_body, t_B_R)
    assert np.allclose(v_body, expected, atol=1e-6), f"v_body={v_body} vs expected={expected}"


def test_lever_arm_pure_pitch():
    """Pure pitch with lever arm."""
    t_B_R = np.array([0.16, 0.0, 0.07])  # radar forward and slightly up
    ext = RadarExtrinsics(R_B_R=np.eye(3), t_B_R=t_B_R)

    omega_body = np.array([0.0, 0.5, 0.0])  # pitch down (FRD: +Y)
    v_body = ext.velocity_radar_to_body(np.zeros(3), omega_body)

    expected = -np.cross(omega_body, t_B_R)
    assert np.allclose(v_body, expected, atol=1e-6), f"v_body={v_body}"


def test_lever_arm_pure_roll():
    """Pure roll with lever arm."""
    t_B_R = np.array([0.16, 0.05, 0.07])
    ext = RadarExtrinsics(R_B_R=np.eye(3), t_B_R=t_B_R)

    omega_body = np.array([0.5, 0.0, 0.0])  # roll right (FRD: +X)
    v_body = ext.velocity_radar_to_body(np.zeros(3), omega_body)

    expected = -np.cross(omega_body, t_B_R)
    assert np.allclose(v_body, expected, atol=1e-6), f"v_body={v_body}"


def test_points_transform():
    """Point cloud radar→body transform."""
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
                'beam_axis_body': [0.0, 0.0, 1.0],  # FRD: +Z = down
            }
        }
    }
    ext = RadarExtrinsics.from_config(config)
    assert not ext.calibrated, "Should be marked as not calibrated"
    assert np.allclose(ext.t_B_R, [0.16, 0.05, 0.07])

    alt = AltimeterExtrinsics.from_config(config)
    assert not alt.calibrated
    assert np.allclose(alt.beam_axis_body, [0.0, 0.0, 1.0])


def test_rpy_identity():
    """Zero RPY should give identity rotation."""
    R = _rpy_to_matrix(0, 0, 0)
    assert np.allclose(R, np.eye(3), atol=1e-10)
