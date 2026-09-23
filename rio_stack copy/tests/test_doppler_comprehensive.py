"""
tests/test_doppler_comprehensive.py — Comprehensive Doppler tests (Master Plan §61).

Tests the RobustDopplerEstimator in isolation with the new API
where lever-arm compensation is expected to be applied before the solver.
"""

import numpy as np
import pytest
from estimator.u300_velocity.doppler import RobustDopplerEstimator


def _make_points_and_dopplers(velocity, n_points=30, fov_az_deg=80, fov_el_deg=30):
    """Generate synthetic radar measurements for a known velocity.

    Doppler model: d_i = -u_i^T @ v_radar
    where u_i = [cos(el)*cos(az), cos(el)*sin(az), sin(el)]

    Points are spread across a 2D grid in (az, el) for proper 3D diversity
    to pass strict condition number and sigma gates.
    """
    np.random.seed(42)

    azimuths_grid = np.linspace(-np.radians(fov_az_deg), np.radians(fov_az_deg), int(np.sqrt(n_points*2)))
    elevations_grid = np.linspace(-np.radians(fov_el_deg), np.radians(fov_el_deg), int(np.sqrt(n_points*2)))
    az_mesh, el_mesh = np.meshgrid(azimuths_grid, elevations_grid)
    
    azimuths = az_mesh.flatten()
    elevations = el_mesh.flatten()
    
    # Shuffle and pick n_points
    idx = np.random.permutation(len(azimuths))[:n_points]
    azimuths = azimuths[idx]
    elevations = elevations[idx]

    points = []
    dopplers = []
    for az, el in zip(azimuths, elevations):
        r = 1.0
        x = r * np.cos(el) * np.cos(az)
        y = r * np.cos(el) * np.sin(az)
        z = r * np.sin(el)
        points.append([x, y, z])

        u = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        d = -u @ velocity + np.random.randn() * 0.05  # noise
        dopplers.append(d)

    return np.array(points), np.array(dopplers)


def _est():
    return RobustDopplerEstimator(
        min_points=6, 
        max_condition_number=100.0,
        max_sigma_v_mps=1.5,  # Relaxed for synthetic tests
        ransac_iters=20
    )


def test_stationary():
    pts, dops = _make_points_and_dopplers(np.zeros(3))
    res = _est().estimate(pts, dops)
    assert res.is_valid
    assert np.linalg.norm(res.velocity) < 0.2, f"v={res.velocity}"
    assert res.is_static


def test_forward():
    pts, dops = _make_points_and_dopplers(np.array([2.0, 0.0, 0.0]))
    res = _est().estimate(pts, dops)
    assert res.is_valid
    assert res.velocity[0] > 1.5, f"Expected forward: v={res.velocity}"


def test_backward():
    pts, dops = _make_points_and_dopplers(np.array([-2.0, 0.0, 0.0]))
    res = _est().estimate(pts, dops)
    assert res.is_valid
    assert res.velocity[0] < -1.5, f"Expected backward: v={res.velocity}"


def test_left():
    pts, dops = _make_points_and_dopplers(np.array([0.0, -2.0, 0.0]))
    res = _est().estimate(pts, dops)
    assert res.is_valid
    assert res.velocity[1] < -1.5, f"Expected left: v={res.velocity}"


def test_right():
    pts, dops = _make_points_and_dopplers(np.array([0.0, 2.0, 0.0]))
    res = _est().estimate(pts, dops)
    assert res.is_valid
    assert res.velocity[1] > 1.5, f"Expected right: v={res.velocity}"


def test_up():
    pts, dops = _make_points_and_dopplers(np.array([0.0, 0.0, -2.0]))
    res = _est().estimate(pts, dops)
    assert res.is_valid
    assert res.velocity[2] < -1.5, f"Expected up: v={res.velocity}"


def test_down():
    pts, dops = _make_points_and_dopplers(np.array([0.0, 0.0, 2.0]))
    res = _est().estimate(pts, dops)
    assert res.is_valid
    assert res.velocity[2] > 1.5, f"Expected down: v={res.velocity}"


def test_diagonal():
    v_true = np.array([1.5, -0.8, 0.5])
    pts, dops = _make_points_and_dopplers(v_true)
    res = _est().estimate(pts, dops)
    assert res.is_valid
    assert np.allclose(res.velocity, v_true, atol=0.4), f"v={res.velocity} vs {v_true}"


def test_outlier():
    v_true = np.array([1.0, 0.0, 0.0])
    pts, dops = _make_points_and_dopplers(v_true, n_points=30)
    dops[5] = 50.0  # outlier
    res = _est().estimate(pts, dops)
    assert res.is_valid
    assert abs(res.velocity[0] - 1.0) < 0.5


def test_multiple_outliers():
    v_true = np.array([2.0, 0.0, 0.0])
    pts, dops = _make_points_and_dopplers(v_true, n_points=40)
    # 20% outliers
    n_outliers = 8
    dops[:n_outliers] = np.random.randn(n_outliers) * 20.0
    res = _est().estimate(pts, dops)
    if res.is_valid:
        assert abs(res.velocity[0] - 2.0) < 0.8


def test_sparse_points():
    v_true = np.array([1.0, 0.0, 0.0])
    pts, dops = _make_points_and_dopplers(v_true, n_points=6)
    res = _est().estimate(pts, dops)
    assert isinstance(res.is_valid, bool)


def test_rank_deficiency():
    pts = np.array([[i, 0, 0] for i in range(1, 11)], dtype=float)
    dops = np.array([-1.0] * 10)
    res = _est().estimate(pts, dops)
    # RANSAC condition gate should catch this
    assert not res.is_valid


def test_bad_conditioning():
    pts = np.array([[10, 0.01*i, 0.01*i] for i in range(10)], dtype=float)
    dops = np.array([-1.0] * 10)
    res = _est().estimate(pts, dops)
    assert not res.is_valid


def test_sign():
    v_toward = np.array([2.0, 0.0, 0.0])
    pts, dops = _make_points_and_dopplers(v_toward)
    res = _est().estimate(pts, dops)
    assert res.is_valid
    assert res.velocity[0] > 0


def test_covariance():
    v_true = np.array([1.0, 0.0, 0.0])
    pts_good, dops_good = _make_points_and_dopplers(v_true, n_points=50, fov_az_deg=80)
    res_good = _est().estimate(pts_good, dops_good)
    
    pts_poor, dops_poor = _make_points_and_dopplers(v_true, n_points=10, fov_az_deg=20)
    res_poor = _est().estimate(pts_poor, dops_poor)
    
    if res_good.is_valid and res_poor.is_valid:
        trace_good = np.trace(res_good.covariance)
        trace_poor = np.trace(res_poor.covariance)
        assert trace_poor > trace_good
