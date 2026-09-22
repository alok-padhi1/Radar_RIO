#!/usr/bin/env python3
"""
mapping/deskew.py — IMU-based SE(3) motion compensation.

Blueprint §15: Rebuild deskew. Do NOT use `xyz -= v_body * dt`.
Use IMU-propagated SE(3) interpolation over the radar accumulation interval.

For every point timestamp t_i:
    T_WB(t_i) is interpolated from the IMU state trajectory.
    p_W = T_WB(t_i) @ T_BR @ p_R(t_i)

The correct data flow is:
    IMU trajectory → continuous pose interpolation → radar point time → SE(3) compensation
NOT: latest velocity × time
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class PoseStamped:
    """A timestamped SE(3) pose."""
    timestamp: float
    rotation: np.ndarray = field(default_factory=lambda: np.eye(3))  # (3,3) R_WB
    translation: np.ndarray = field(default_factory=lambda: np.zeros(3))  # t_WB

    @property
    def T(self) -> np.ndarray:
        """Full 4x4 homogeneous transform."""
        T = np.eye(4)
        T[:3, :3] = self.rotation
        T[:3, 3] = self.translation
        return T


def skew(v: np.ndarray) -> np.ndarray:
    """Skew-symmetric matrix from a 3-vector (for cross product)."""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0],
    ])


def exp_so3(omega: np.ndarray) -> np.ndarray:
    """Exponential map: so(3) → SO(3) via Rodrigues formula.

    Args:
        omega: (3,) rotation vector [rad]

    Returns:
        (3,3) rotation matrix
    """
    theta = np.linalg.norm(omega)
    if theta < 1e-10:
        return np.eye(3) + skew(omega)
    k = omega / theta
    K = skew(k)
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * K @ K


def interpolate_pose(t: float, t0: float, t1: float,
                     R0: np.ndarray, p0: np.ndarray,
                     R1: np.ndarray, p1: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Interpolate SE(3) pose between two timestamps.

    Uses linear interpolation for translation and SLERP-equivalent
    (logarithmic interpolation) for rotation.

    Args:
        t: Query timestamp
        t0, t1: Boundary timestamps (t0 <= t <= t1)
        R0, p0: Rotation and translation at t0
        R1, p1: Rotation and translation at t1

    Returns:
        (R_interp, p_interp) — interpolated rotation and translation
    """
    if t1 <= t0:
        return R0.copy(), p0.copy()

    alpha = (t - t0) / (t1 - t0)
    alpha = max(0.0, min(1.0, alpha))

    # Linear translation interpolation
    p = p0 + alpha * (p1 - p0)

    # Rotation interpolation via log map
    dR = R0.T @ R1
    # Extract rotation vector from dR
    cos_angle = (np.trace(dR) - 1) / 2
    cos_angle = max(-1.0, min(1.0, cos_angle))

    if abs(cos_angle - 1.0) < 1e-10:
        R = R0.copy()
    else:
        angle = math.acos(cos_angle)
        if abs(angle) < 1e-10:
            R = R0.copy()
        else:
            # Axis from skew-symmetric part of dR
            axis = np.array([
                dR[2, 1] - dR[1, 2],
                dR[0, 2] - dR[2, 0],
                dR[1, 0] - dR[0, 1],
            ]) / (2 * math.sin(angle))
            omega = axis * angle * alpha
            R = R0 @ exp_so3(omega)

    return R, p


class IMUTrajectory:
    """Maintains a short IMU-propagated pose trajectory for deskewing.

    Blueprint §15: Provides continuous SE(3) interpolation from IMU data
    for per-point motion compensation within a radar frame.
    """

    def __init__(self, max_history_s: float = 2.0):
        self.max_history_s = max_history_s
        self._poses: list[PoseStamped] = []
        self._lock = None  # For thread safety if needed

    def add_pose(self, t: float, R: np.ndarray, p: np.ndarray):
        """Add a new pose to the trajectory."""
        self._poses.append(PoseStamped(timestamp=t, rotation=R.copy(), translation=p.copy()))

        # Prune old poses
        if len(self._poses) > 2:
            cutoff = t - self.max_history_s
            while len(self._poses) > 2 and self._poses[0].timestamp < cutoff:
                self._poses.pop(0)

    def query(self, t: float) -> Optional[tuple[np.ndarray, np.ndarray]]:
        """Query interpolated pose at time t.

        Returns:
            (R, p) or None if t is outside trajectory bounds
        """
        if len(self._poses) < 2:
            if len(self._poses) == 1:
                return self._poses[0].rotation.copy(), self._poses[0].translation.copy()
            return None

        # Find bracketing poses
        if t <= self._poses[0].timestamp:
            return self._poses[0].rotation.copy(), self._poses[0].translation.copy()
        if t >= self._poses[-1].timestamp:
            return self._poses[-1].rotation.copy(), self._poses[-1].translation.copy()

        for i in range(len(self._poses) - 1):
            if self._poses[i].timestamp <= t <= self._poses[i + 1].timestamp:
                p0 = self._poses[i]
                p1 = self._poses[i + 1]
                return interpolate_pose(
                    t, p0.timestamp, p1.timestamp,
                    p0.rotation, p0.translation,
                    p1.rotation, p1.translation,
                )

        return None

    @property
    def time_span(self) -> float:
        if len(self._poses) < 2:
            return 0.0
        return self._poses[-1].timestamp - self._poses[0].timestamp


def deskew_points(points_radar: np.ndarray,
                  point_timestamps: np.ndarray,
                  trajectory: IMUTrajectory,
                  T_BR: np.ndarray,
                  reference_time: float = None) -> np.ndarray:
    """Deskew radar points using IMU-propagated SE(3) trajectory.

    Blueprint §15: For every point timestamp t_i:
        T_WB(t_i) = interpolated from IMU trajectory
        p_W = T_WB(t_i) @ T_BR @ p_R(t_i)

    All points are transformed to a common reference frame.

    Args:
        points_radar: (N, 3) points in radar frame
        point_timestamps: (N,) timestamps for each point
        trajectory: IMU-propagated pose trajectory
        T_BR: (4, 4) radar-to-body transform
        reference_time: Time to use as the common frame (default: last point)

    Returns:
        (N, 3) deskewed points in world frame at reference_time
    """
    N = len(points_radar)
    if N == 0:
        return np.empty((0, 3))

    if reference_time is None:
        reference_time = point_timestamps[-1]

    # Get reference pose
    ref_result = trajectory.query(reference_time)
    if ref_result is None:
        logger.warning("Cannot deskew: no trajectory at reference time")
        # Fallback: apply T_BR only, no motion compensation
        pts_h = np.hstack([points_radar, np.ones((N, 1))])
        return (T_BR @ pts_h.T)[:3].T

    R_ref, p_ref = ref_result
    T_ref = np.eye(4)
    T_ref[:3, :3] = R_ref
    T_ref[:3, 3] = p_ref
    T_ref_inv = np.eye(4)
    T_ref_inv[:3, :3] = R_ref.T
    T_ref_inv[:3, 3] = -R_ref.T @ p_ref

    # Deskew each point
    result = np.zeros((N, 3))
    pts_h = np.hstack([points_radar, np.ones((N, 1))])

    for i in range(N):
        t_i = point_timestamps[i]
        pose_i = trajectory.query(t_i)

        if pose_i is None:
            # No trajectory coverage — use reference pose
            p_world = T_BR @ pts_h[i]
            result[i] = p_world[:3]
            continue

        R_i, p_i = pose_i
        T_WB_i = np.eye(4)
        T_WB_i[:3, :3] = R_i
        T_WB_i[:3, 3] = p_i

        # p_W = T_WB(t_i) @ T_BR @ p_R(t_i)
        p_world = T_WB_i @ T_BR @ pts_h[i]

        # Transform back to reference frame
        p_ref_frame = T_ref_inv @ p_world
        result[i] = p_ref_frame[:3]

    return result
