#!/usr/bin/env python3
"""
filters.py
Preprocessing pipeline that turns a raw, per-frame Points_float body-frame
cloud into a clean, structured cloud suitable for k-NN Generalized ICP.

Why this exists (see PIPELINE_NOTES.md for the full critique): a single raw
U300 frame at 20 Hz is not "a point cloud" in the LiDAR sense -- it is a
sparse, high-variance detection list where near-field TX/RX leakage,
multipath ghosts, sidelobe clutter, and (during static bench testing)
range-bin/angular-smearing artifacts all masquerade as real geometry. GICP
run directly on that input either finds no valid correspondences or, worse,
converges confidently to a wrong transform because it fit noise. Every
function below is a *rejection* stage -- the philosophy throughout this
project (doppler_rio.py's RANSAC gate, slam_node.py's correspondence-count
gate) is "throw the frame away if it's not trustworthy," and this module is
that same philosophy applied one layer earlier, to individual points.

Stages, applied in order by `preprocess_frame()`:
  1. Near-field leakage gate      -- drop points inside the TX/RX coupling zone
  2. Range gate                    -- drop points outside the sensor's valid envelope
  3. Doppler static-consistency    -- drop points whose radial velocity is
                                       inconsistent with the (known or
                                       assumed-near-zero) ego-velocity model
  4. Temporal persistence filter   -- drop points that don't re-appear in
                                       roughly the same place across
                                       consecutive frames (kills multipath
                                       ghosts and one-off sidelobe hits)
  5. Statistical outlier removal   -- drop points whose local neighborhood
                                       density is anomalously low
  6. Voxel downsample              -- collapse remaining points to a
                                       manageable, roughly-uniform density
"""

from dataclasses import dataclass, field

import numpy as np
import open3d as o3d


@dataclass
class FilterConfig:
    # Stage 1: near-field leakage. The U300's own TX/RX coupling and antenna
    # near-field produce spurious near-zero-range detections; these are a
    # sensor artifact, not target returns.
    leakage_radius_m: float = 0.35

    # Stage 2: valid operating envelope (U300 rated max range from the
    # monograph). Anything beyond this is almost certainly multipath (a
    # ghost bounced off two surfaces reads out at a longer apparent range)
    # or a range-ambiguous alias, not a first-bounce return.
    min_range_m: float = 0.3
    max_range_m: float = 350.0

    # Stage 3: Doppler static-consistency. eps is the same tolerance
    # doppler_rio.py's RANSAC uses -- keep them in sync so "what RIO trusts"
    # and "what SLAM trusts" don't silently diverge.
    # NOTE: 0.40 m/s is correct for dynamic walking at ~1 m/s with IMU rotation
    # compensation; the extra headroom absorbs lever-arm projection errors from
    # uncalibrated AHRS pitch. Use 0.20 m/s for static bench testing only.
    doppler_eps_mps: float = 0.40

    # Stage 4: temporal persistence. A point must have a neighbor within
    # `persistence_radius_m` in at least `persistence_min_hits` of the last
    # `persistence_window` frames to survive. Ghosts from multi-bounce
    # multipath are geometrically inconsistent frame-to-frame (the apparent
    # ghost position depends on the exact bounce geometry, which shifts as
    # the platform moves); real static structure is not.
    persistence_radius_m: float = 1.0
    persistence_window: int = 4
    persistence_min_hits: int = 2

    # Stage 5: statistical outlier removal (Open3D).
    sor_nb_neighbors: int = 8
    sor_std_ratio: float = 1.5

    # Stage 6: voxel downsample.
    voxel_size_m: float = 1.0


# --------------------------------------------------------------- Stage 1-2

def gate_range(xyz: np.ndarray, cfg: FilterConfig) -> np.ndarray:
    """Returns a boolean keep-mask. Combines the leakage gate and the max-range
    gate since both are simple range-bound tests on the same quantity."""
    r = np.linalg.norm(xyz, axis=1)
    return (r > max(cfg.leakage_radius_m, cfg.min_range_m)) & (r < cfg.max_range_m)


# --------------------------------------------------------------- Stage 3

def gate_doppler_consistency(xyz: np.ndarray, v_radial: np.ndarray,
                              v_body_hint: np.ndarray | None,
                              omega_hint: np.ndarray | None,
                              cfg: FilterConfig,
                              lever_arm: np.ndarray | None = None) -> np.ndarray:
    """Rejects points whose measured radial velocity disagrees with the
    static-world Doppler model V_i = -u_i^T v_body (Eq. 7 of the monograph).

    On the bench (handheld/stationary, v_body_hint ~ [0,0,0] or None), this
    reduces to a zero-Doppler-consistency test: real static clutter/ground
    SHOULD read V~0 here -- that's a pass, not a fail. What this stage
    actually removes is anything moving relative to the sensor (a walking
    person, a passing car, foliage in wind) plus a class of multipath ghost
    whose bounce geometry gives it a Doppler value inconsistent with any
    single rigid-body velocity. Do not confuse "removes clutter" with
    "removes everything at V=0" -- V=0 IS the expected value for real static
    ground during a static bench test.
    """
    r = np.linalg.norm(xyz, axis=1)
    r = np.clip(r, 1e-3, None)
    u = xyz / r[:, None]
    if v_body_hint is None or np.linalg.norm(v_body_hint) < 0.1:
        return np.ones(len(xyz), dtype=bool)
        
    v_hint = v_body_hint
    
    if omega_hint is not None:
        lever = lever_arm if lever_arm is not None else np.zeros(3)
        omega_cross_p = np.cross(omega_hint, lever)
        v_rot_comp = np.sum(u * omega_cross_p, axis=1)
        predicted = -(u @ v_hint) - v_rot_comp
    else:
        predicted = -(u @ v_hint)
        
    residual = np.abs(v_radial - predicted)
    return residual < cfg.doppler_eps_mps


# --------------------------------------------------------------- Stage 4

class PersistenceTracker:
    """Frame-to-frame nearest-neighbor persistence check. Stateful: call
    `filter()` once per incoming (already range/Doppler-gated) frame, in
    order; it keeps a short rolling history internally."""

    def __init__(self, cfg: FilterConfig):
        self.cfg = cfg
        self._history: list[np.ndarray] = []  # list of (N_k, 3) past frames, newest last

    def filter(self, xyz: np.ndarray, v_body: np.ndarray = None, dt: float = 0.0, omega: np.ndarray = None) -> np.ndarray:
        cfg = self.cfg
        if not self._history:
            self._history.append(xyz)
            # First frame in the window: nothing to compare against yet.
            # Pass everything through rather than discarding an entire
            # frame's worth of real structure while the tracker warms up.
            return np.ones(len(xyz), dtype=bool)

        # Deskew history to current body frame: the vehicle moved forward by (v_body * dt)
        # so objects in the past frames appear to have moved backwards by -(v_body * dt)
        if v_body is not None and dt > 0:
            shift = -v_body * dt
            if omega is not None:
                # Rotate points to compensate for yawing/pitching
                self._history = [past - np.cross(omega, past) * dt + shift for past in self._history]
            else:
                self._history = [past + shift for past in self._history]

        recent = self._history[-cfg.persistence_window:]
        hits = np.zeros(len(xyz), dtype=int)
        for past in recent:
            if len(past) == 0:
                continue
            pcd_past = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(past))
            kdtree = o3d.geometry.KDTreeFlann(pcd_past)
            for i, p in enumerate(xyz):
                k, _idx, dist2 = kdtree.search_radius_vector_3d(p, cfg.persistence_radius_m)
                if k > 0:
                    hits[i] += 1

        # Warm-up fix: cfg.persistence_min_hits is only achievable once at
        # least that many PAST frames exist. Before the window is full,
        # e.g. on the 2nd call there is only 1 past frame to compare
        # against, so a fixed threshold of persistence_min_hits=2 would
        # reject every real, persistent point simply because there hasn't
        # been time to accumulate enough history yet -- not because the
        # points are actually transient. Scale the effective requirement
        # down to whatever history is actually available; it converges to
        # the configured strictness once `persistence_window` frames have
        # been seen.
        effective_min_hits = min(cfg.persistence_min_hits, len(recent))
        keep = hits >= effective_min_hits
        self._history.append(xyz)
        if len(self._history) > cfg.persistence_window:
            self._history.pop(0)
        return keep

    def reset(self):
        self._history = []


# --------------------------------------------------------------- Stage 5-6

def statistical_outlier_removal(pcd: o3d.geometry.PointCloud, cfg: FilterConfig):
    if len(pcd.points) < cfg.sor_nb_neighbors + 1:
        return pcd, np.arange(len(pcd.points))
    clean, idx = pcd.remove_statistical_outlier(
        nb_neighbors=cfg.sor_nb_neighbors, std_ratio=cfg.sor_std_ratio)
    return clean, np.asarray(idx)


def voxel_downsample(pcd: o3d.geometry.PointCloud, cfg: FilterConfig):
    return pcd.voxel_down_sample(cfg.voxel_size_m)


# --------------------------------------------------------------- pipeline

@dataclass
class PreprocessResult:
    pcd: o3d.geometry.PointCloud
    n_raw: int
    n_after_range: int
    n_after_doppler: int
    n_after_persistence: int
    n_after_sor: int
    n_final: int


def preprocess_frame(xyz_body: np.ndarray, v_radial: np.ndarray,
                      tracker: PersistenceTracker,
                      v_body_hint: np.ndarray | None,
                      omega_hint: np.ndarray | None,
                      cfg: FilterConfig,
                      lever_arm: np.ndarray | None = None,
                      dt: float = 0.0) -> PreprocessResult:
    n_raw = len(xyz_body)

    keep = gate_range(xyz_body, cfg)
    xyz1, v1 = xyz_body[keep], v_radial[keep]
    n_after_range = len(xyz1)

    keep = gate_doppler_consistency(xyz1, v1, v_body_hint, omega_hint, cfg, lever_arm=lever_arm)
    xyz2 = xyz1[keep]
    n_after_doppler = len(xyz2)

    keep = tracker.filter(xyz2, v_body=v_body_hint, dt=dt, omega=omega_hint)
    xyz3 = xyz2[keep]
    n_after_persistence = len(xyz3)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz3)
    pcd, _idx = statistical_outlier_removal(pcd, cfg)
    n_after_sor = len(pcd.points)

    pcd = voxel_downsample(pcd, cfg)
    n_final = len(pcd.points)

    return PreprocessResult(
        pcd=pcd, n_raw=n_raw, n_after_range=n_after_range,
        n_after_doppler=n_after_doppler, n_after_persistence=n_after_persistence,
        n_after_sor=n_after_sor, n_final=n_final,
    )
