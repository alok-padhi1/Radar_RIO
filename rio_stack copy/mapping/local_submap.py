#!/usr/bin/env python3
"""
mapping/local_submap.py — Local radar submap manager.

Blueprint §14: A submap contains 20-50 radar frames (configurable).
Each point stores timestamp, frame_id, radar coordinates, world pose
at measurement, measurement covariance, Doppler, static probability.

§14.1: Do NOT accumulate an unbounded global point cloud. Use rolling/local submaps.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class SubmapPoint:
    """A single point within a submap — Blueprint §14.

    Stores full provenance for debugging and re-processing.
    """
    position: np.ndarray       # (3,) world-frame position [m]
    radar_coords: np.ndarray   # (3,) original radar-frame coordinates [m]
    timestamp: float           # Measurement timestamp
    frame_id: int              # Source radar frame ID
    covariance: Optional[np.ndarray] = None  # (3,3) measurement covariance
    doppler_mps: float = 0.0   # Radial velocity [m/s]
    static_probability: float = 1.0  # Probability of being static [0,1]
    world_pose: Optional[np.ndarray] = None  # (4,4) T_WB at measurement time


@dataclass
class SubmapStats:
    """Quality statistics for a submap."""
    n_points: int = 0
    n_static_points: int = 0
    n_frames: int = 0
    time_span_s: float = 0.0
    mean_range_m: float = 0.0
    spatial_extent: np.ndarray = field(default_factory=lambda: np.zeros(3))  # XYZ spread


class LocalSubmap:
    """A single local submap containing accumulated radar frames.

    Blueprint §14: Each submap stores pose, point cloud, descriptor,
    quality statistics, time span.
    """

    def __init__(self, submap_id: int, max_frames: int = 50):
        self.submap_id = submap_id
        self.max_frames = max_frames
        self.points: list[SubmapPoint] = []
        self.frame_ids: list[int] = []
        self.frame_timestamps: list[float] = []
        self.pose: Optional[np.ndarray] = None  # (4,4) submap reference pose
        self._finalized = False

    @property
    def n_frames(self) -> int:
        return len(self.frame_ids)

    @property
    def n_points(self) -> int:
        return len(self.points)

    @property
    def is_full(self) -> bool:
        return self.n_frames >= self.max_frames

    @property
    def time_span(self) -> float:
        if len(self.frame_timestamps) < 2:
            return 0.0
        return self.frame_timestamps[-1] - self.frame_timestamps[0]

    def add_frame(self, points_world: np.ndarray,
                  points_radar: np.ndarray,
                  timestamp: float,
                  frame_id: int,
                  world_pose: np.ndarray = None,
                  covariances: np.ndarray = None,
                  doppler: np.ndarray = None,
                  static_prob: np.ndarray = None):
        """Add a radar frame's points to this submap.

        Args:
            points_world: (N, 3) deskewed world-frame points
            points_radar: (N, 3) original radar-frame points
            timestamp: Frame timestamp
            frame_id: Frame sequence number
            world_pose: (4, 4) T_WB at measurement time
            covariances: (N, 3, 3) per-point covariances
            doppler: (N,) Doppler velocities
            static_prob: (N,) static probabilities
        """
        if self._finalized:
            logger.warning(f"Submap {self.submap_id} is finalized, cannot add frames")
            return

        N = len(points_world)
        for i in range(N):
            sp = SubmapPoint(
                position=points_world[i].copy(),
                radar_coords=points_radar[i].copy() if points_radar is not None else np.zeros(3),
                timestamp=timestamp,
                frame_id=frame_id,
                covariance=covariances[i] if covariances is not None else None,
                doppler_mps=float(doppler[i]) if doppler is not None else 0.0,
                static_probability=float(static_prob[i]) if static_prob is not None else 1.0,
                world_pose=world_pose.copy() if world_pose is not None else None,
            )
            self.points.append(sp)

        self.frame_ids.append(frame_id)
        self.frame_timestamps.append(timestamp)

        if self.pose is None and world_pose is not None:
            self.pose = world_pose.copy()

    def get_points_array(self) -> np.ndarray:
        """Get all points as (N, 3) array in world frame."""
        if not self.points:
            return np.empty((0, 3))
        return np.array([p.position for p in self.points])

    def get_static_points(self, threshold: float = 0.5) -> np.ndarray:
        """Get static points only, filtered by probability."""
        pts = [p.position for p in self.points if p.static_probability >= threshold]
        return np.array(pts) if pts else np.empty((0, 3))

    def get_stats(self) -> SubmapStats:
        pts = self.get_points_array()
        stats = SubmapStats(
            n_points=len(pts),
            n_static_points=len(self.get_static_points()),
            n_frames=self.n_frames,
            time_span_s=self.time_span,
        )
        if len(pts) > 0:
            ranges = np.linalg.norm(pts, axis=1)
            stats.mean_range_m = float(np.mean(ranges))
            stats.spatial_extent = pts.max(axis=0) - pts.min(axis=0)
        return stats

    def finalize(self):
        """Mark submap as complete — no more frames can be added."""
        self._finalized = True


class SubmapManager:
    """Manages a rolling collection of local submaps.

    Blueprint §14.1: Do NOT accumulate an unbounded global cloud.
    Use rolling/local submaps.

    Structure:
        SUBMAP 0 → SUBMAP 1 → SUBMAP 2 → ...
    Old submaps are dropped as new ones are created.
    """

    def __init__(self, frames_per_submap: int = 30,
                 max_submaps: int = 10):
        self.frames_per_submap = frames_per_submap
        self.max_submaps = max_submaps
        self._submaps: list[LocalSubmap] = []
        self._current: Optional[LocalSubmap] = None
        self._next_id = 0

    @property
    def current_submap(self) -> Optional[LocalSubmap]:
        return self._current

    @property
    def submaps(self) -> list[LocalSubmap]:
        return list(self._submaps)

    @property
    def n_submaps(self) -> int:
        return len(self._submaps)

    def add_frame(self, points_world: np.ndarray,
                  points_radar: np.ndarray = None,
                  timestamp: float = 0.0,
                  frame_id: int = 0,
                  world_pose: np.ndarray = None,
                  covariances: np.ndarray = None,
                  doppler: np.ndarray = None,
                  static_prob: np.ndarray = None) -> Optional[LocalSubmap]:
        """Add a frame to the current submap.

        Returns the finalized submap if one was just completed, else None.
        """
        # Start new submap if needed
        if self._current is None or self._current.is_full:
            if self._current is not None:
                self._current.finalize()
            self._current = LocalSubmap(
                submap_id=self._next_id,
                max_frames=self.frames_per_submap,
            )
            self._submaps.append(self._current)
            self._next_id += 1

            # Prune old submaps
            while len(self._submaps) > self.max_submaps:
                self._submaps.pop(0)

        self._current.add_frame(
            points_world=points_world,
            points_radar=points_radar,
            timestamp=timestamp,
            frame_id=frame_id,
            world_pose=world_pose,
            covariances=covariances,
            doppler=doppler,
            static_prob=static_prob,
        )

        # Return finalized submap if just completed
        if self._current.is_full:
            completed = self._current
            return completed

        return None

    def get_registration_target(self) -> Optional[LocalSubmap]:
        """Get the latest completed submap for scan-to-submap registration.

        Blueprint §16: Start with scan/submap, not scan/scan.
        """
        completed = [s for s in self._submaps if s._finalized]
        return completed[-1] if completed else None
