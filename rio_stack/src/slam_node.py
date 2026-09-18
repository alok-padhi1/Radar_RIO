#!/usr/bin/env python3
"""
slam_node.py
Dense 4D radar SLAM front-end for sparse Points_float clouds (Linpowave
U300-class), Stage A per ARCHITECTURE.md Section 3: incremental
keyframe-to-local-submap GICP with adaptive per-point covariance and a
Doppler-consistency soft prior. No loop closure yet (see Section 3, Stage B
for the GTSAM/iSAM2 upgrade path) -- this stage delivers a
locally-drift-corrected pose chain, which is what you need through Test
Stage 3-4.

Input:  UDP frames from radar_fanout.py's SLAM path (already decimated to
        ~5 Hz), each carrying (x,y,z,v) in RADAR frame, plus the tilt-mount
        transform applied here (same Eq.(2) as doppler_rio.py).
        Optionally consumes RIOWorker's forwarded v_body over a second UDP
        port to (a) motion-compensate (deskew) points accumulated within a
        keyframe window and (b) weight the Doppler-consistency term in the
        GICP cost, Eq.(23) of the monograph.

Output: incremental pose chain (SE(3) per keyframe) and the running
        accumulated map, forwarded over UDP for any downstream viewer
        (adapt radar_3d_viewer.py to subscribe here instead of raw points),
        plus optional VISION_POSITION_ESTIMATE hand-off to mavlink_bridge.py
        once flight-validated (do not enable before Test Stage 4).

Requires: pip install open3d numpy --break-system-packages
"""

import logging
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
import argparse
import math
import os
import socket
import struct
import threading
import time
import queue
from dataclasses import dataclass, field

import numpy as np
import open3d as o3d

from filters import FilterConfig, PersistenceTracker, preprocess_frame

UDP_HEADER = struct.Struct('<I')
# Extended RIO packet — must match doppler_rio.py FORWARD_PKT exactly.
# t, vx, vy, vz, n_inliers, cxx, cyy, czz, n_total, flags, cond (46 bytes)
# flags bit0=is_static, bit1=airborne, bit2=vz_prior_used, bit3=accel_gate_armed
RIO_PKT = struct.Struct('<dfffIfffIBf')   # 46 bytes
# IMU packet from imu_bridge.py: t_mono, roll, pitch, yaw, omega_x, omega_y, omega_z
# IMU packet from imu_bridge.py (37 bytes): t, roll, pitch, yaw, wx, wy, wz, airborne, vz_ned
IMU_PKT = struct.Struct('<dffffffBf')  # 37 bytes
# t, n_map_points, fwd_obstacle_range_m ; followed by 16 float64 (4x4 row-major T)
POSE_PKT_HDR = struct.Struct('<dId')


# ---------------------------------------------------------------- geometry

@dataclass
class TiltMount:
    theta_tilt_deg: float = 40.0
    lever_arm: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # See doppler_rio.py's TiltMount for the full derivation/citations. Must
    # be kept IDENTICAL between doppler_rio.py and slam_node.py -- RIO and
    # SLAM have to agree on what "body frame" means, or their outputs
    # silently disagree about the world (this was root-cause #3 of the
    # Stage 1 bench failure: SLAM inherited RIO's bad transform too, then
    # inherited RIO's spurious velocity as a deskew/consistency hint on top
    # of it).
    lateral_sign: float = 1.0

    def __post_init__(self):
        th = math.radians(self.theta_tilt_deg)
        c, s = math.cos(th), math.sin(th)
        R_tilt = np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])  # Eq.(1)
        # Native U300 radar frame (X=lateral, Y=forward, Z=up) -> pre-tilt
        # body-aligned (X=forward, Y=lateral, Z=down). Confirmed against
        # Linpowave_visualizer_UART userguide_Points_float.pdf.
        self.P = np.array([
            [0.0,              1.0, 0.0],
            [self.lateral_sign, 0.0, 0.0],
            [0.0,              0.0, -1.0],
        ])
        self.R_static = R_tilt @ self.P

    def get_R(self, attitude: np.ndarray) -> np.ndarray:
        """Returns the dynamic rotation matrix R_{Body -> Earth} from IMU attitude."""
        roll, pitch, _ = attitude
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        
        # R_{Body -> Earth} = R_y(pitch) @ R_x(roll)
        # Note: In aerospace FRD, positive pitch is nose UP, so forward vector moves UP (-Z).
        Ry = np.array([
            [ cp, 0.0,  sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0,  cp],
        ])
        Rx = np.array([
            [1.0, 0.0, 0.0],
            [0.0,  cr, -sr],
            [0.0,  sr,  cr],
        ])
        return Ry @ Rx

    def to_body(self, xyz_radar: np.ndarray, attitude: np.ndarray = None,
                imu_level_points: bool = True) -> np.ndarray:
        """Transform radar points to body frame.
        
        imu_level_points: if True (default), apply full pitch+roll leveling from IMU.
          Set False to use only the static mechanical tilt (R_static) for point
          coordinates, which prevents accumulated map distortion when the IMU pitch
          reference is unreliable (e.g. handheld mounts with uncalibrated AHRS trim).
          Yaw and deskew still use the IMU — only the pitch/roll leveling is disabled.
        """
        if attitude is not None and imu_level_points:
            R_level = self.get_R(attitude)
            # R_tilt (mechanical boresight tilt) is independent of, and in addition
            # to, the IMU's dynamic leveling — it must not be dropped here.
            R_dynamic = R_level @ self.R_static   # R_static == R_tilt @ P
            # The lever arm must ALSO be leveled!
            return xyz_radar @ R_dynamic.T + (R_level @ self.lever_arm)

        # Static-only path: apply mechanical tilt only, no IMU pitch/roll.
        # This is the safe default when AHRS pitch calibration is unknown.
        return xyz_radar @ self.R_static.T + self.lever_arm  # Eq.(2)


def parse_udp_packet(data: bytes) -> np.ndarray | None:
    if len(data) < UDP_HEADER.size:
        return None
    (points_num,) = UDP_HEADER.unpack_from(data, 0)
    expected_size = UDP_HEADER.size + points_num * 16
    if len(data) != expected_size:
        return None
    pts = np.frombuffer(data, dtype='<f4', count=points_num * 4, offset=UDP_HEADER.size)
    return pts.reshape(-1, 4)


class UdpReceiver(threading.Thread):
    def __init__(self, sock: socket.socket, q_size: int = 100):
        super().__init__(daemon=True)
        self.sock = sock
        self.q = queue.Queue(maxsize=q_size)

    def run(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
                try:
                    self.q.put_nowait(data)
                except queue.Full:
                    pass
            except Exception:
                time.sleep(0.001)

    def drain(self):
        pkts = []
        while True:
            try:
                pkts.append(self.q.get_nowait())
            except queue.Empty:
                break
        return pkts


# ------------------------------------------------------------ keyframe accumulator

class KeyframeAccumulator:
    """Buffers body-frame points across a short time window, motion-compensating
    (deskewing) each raw frame's points against the mean RIO velocity spanning
    the window before merging -- this is what turns '~30 sparse points' into a
    '150-500 point' keyframe cloud dense enough for GICP (Architecture Sec. 3.1)."""

    def __init__(self, window_s: float = 0.3):
        self.window_s = window_s
        self._frames = []  # list of (t, xyz_body (N,3), v_radial (N,))
        self.t_last_kf = 0.0

    def add(self, t: float, xyz_body: np.ndarray, v_radial: np.ndarray):
        self._frames.append((t, xyz_body, v_radial))
        self._frames = [f for f in self._frames if t - f[0] <= self.window_s * 1.5]

    def ready(self, min_frames: int = 3) -> bool:
        if len(self._frames) < min_frames:
            return False
        if self.t_last_kf == 0.0:
            return True
        return (self._frames[-1][0] - self.t_last_kf) >= self.window_s

    def build(self, v_body_hint: np.ndarray | None,
              omega_hint: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        """Returns (points_Nx3, v_radial_N) in the reference frame of the
        newest sample in the window, with older frames translated back along
        v_body_hint to deskew ('un-move' the vehicle's own motion during the
        window) if a velocity hint is available. When omega_hint is provided
        (from the IMU), also applies a first-order rotational correction to
        older frames. v_radial is passed through unchanged -- it's a
        frame-invariant radial measurement (see the monograph, Sec. 1.4),
        so deskewing the xyz doesn't touch it."""
        if not self._frames:
            return np.empty((0, 3)), np.empty((0,))
        t_ref = self._frames[-1][0]
        pts_all, v_all = [], []
        for t, xyz, v in self._frames:
            dt = t_ref - t
            if dt > 0:
                # Rotational deskew (Stage 3): rotate older points back by
                # omega * dt using the Rodrigues small-angle approximation.
                if omega_hint is not None:
                    # Points must rotate opposite the sensor's own rotation to land
                    # in the newer frame: p(t_ref) = Exp(-ω·dt) @ p(t).
                    dtheta = -omega_hint * dt
                    angle = np.linalg.norm(dtheta)
                    if angle > 1e-6:
                        k = dtheta / angle
                        K = np.array([[0, -k[2], k[1]],
                                      [k[2], 0, -k[0]],
                                      [-k[1], k[0], 0]])
                        R_dt = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
                        xyz = xyz @ R_dt.T  # rotate points into reference frame
                # Translational deskew
                if v_body_hint is not None:
                    xyz = xyz - v_body_hint * dt
            pts_all.append(xyz)
            v_all.append(v)
        merged = np.vstack(pts_all)
        v_merged = np.concatenate(v_all)
        return merged, v_merged

    def clear(self):
        self._frames = []


# ---------------------------------------------------------------- SLAM core

class RadarSLAM:
    def __init__(self, mount: TiltMount, voxel_size: float = 1.5,
                 gicp_max_corr_dist: float = 6.0, min_correspondences: int = 15,
                 filter_cfg: FilterConfig | None = None,
                 plane_threshold: float = 0.6,
                 trust_imu_yaw: bool = True,
                 lambda_min_observable: float = 3.0,
                 observable_ratio: float = 0.05,
                 imu_level_points: bool = False):
        self.mount = mount
        self.plane_threshold = plane_threshold
        self.voxel_size = voxel_size
        self.gicp_max_corr_dist = gicp_max_corr_dist
        self.min_correspondences = min_correspondences
        self.trust_imu_yaw = trust_imu_yaw
        self.lambda_min_observable = lambda_min_observable
        self.observable_ratio = observable_ratio
        # imu_level_points: apply pitch+roll leveling to point coords (default OFF for
        # handheld/uncalibrated AHRS mounts — only yaw is used). Enable only when AHRS
        # trim is properly calibrated for the physical mount.
        self.imu_level_points = imu_level_points

        self.T_world = np.eye(4)          # current pose, world <- body
        self.map_cloud = o3d.geometry.PointCloud()
        self.pose_chain = [self.T_world.copy()]
        self.last_v_body = None           # most recent RIO estimate, for deskew + Doppler prior
        self.last_kf_t = None
        self.last_attitude = None         # (3,) np.ndarray: [roll, pitch, yaw] from IMU (rad)

        # Sec. "Pipeline" addendum: raw keyframes are pre-cleaned (leakage,
        # range, Doppler-consistency, temporal-persistence, SOR) before ever
        # reaching voxel downsample / ground-plane split / GICP below. Voxel
        # size here should match filter_cfg.voxel_size_m -- keeping one
        # FilterConfig as the source of truth avoids the two silently drifting.
        self.filter_cfg = filter_cfg or FilterConfig(voxel_size_m=voxel_size)
        self.persistence_tracker = PersistenceTracker(self.filter_cfg)
        self.initial_yaw_imu = None

    def update_velocity(self, v_body: np.ndarray):
        self.last_v_body = v_body

    def update_attitude(self, attitude: np.ndarray):
        """Store the latest IMU fused attitude [roll, pitch, yaw] in radians."""
        self.last_attitude = attitude

    def _gravity_correct(self, T: np.ndarray, agl_m: float | None = None) -> np.ndarray:
        """Clamp pitch and roll to 0. Since the incoming point clouds are now
        dynamically leveled by the IMU *before* GICP, the matched pose should
        be perfectly flat. This prevents numerical noise from accumulating into
        catastrophic Z-drift (observed: 191 m in 96 s).
        """
        R_gicp = T[:3, :3]
        # Levelling assumption check: if GICP has drifted more than ~10 deg off
        # level, the yaw extraction below is no longer valid and something
        # upstream (levelling, deskew, correspondence) has failed.
        if abs(R_gicp[2, 0]) > 0.17:            # |sin(pitch)| > 10 deg
            logging.warning(f"[slam] GICP pose is {math.degrees(math.asin(abs(R_gicp[2,0]))):.0f} "
                             f"deg off level -- levelling or deskew is broken.")
        yaw_gicp = math.atan2(R_gicp[1, 0], R_gicp[0, 0])

        if self.trust_imu_yaw and self.last_attitude is not None:
            yaw_gicp = self.last_attitude[2]

        cy, sy = math.cos(yaw_gicp),  math.sin(yaw_gicp)

        # Pure Yaw rotation matrix
        R_corrected = np.array([
            [ cy, -sy, 0.0],
            [ sy,  cy, 0.0],
            [0.0, 0.0, 1.0],
        ])

        T_out = T.copy()
        T_out[:3, :3] = R_corrected

        # HARD Z CONSTRAINT. The rotation clamp alone does not bound Z (the
        # translation column was previously passed through untouched, despite
        # the docstring's claim). The belly altimeter is the only absolute
        # height reference in the system; without it Z is a free-running
        # integrator. Map frame is Z-DOWN, so world Z = -(AGL) + takeoff offset.
        if agl_m is not None and math.isfinite(agl_m):
            if not hasattr(self, 'agl_at_bootstrap') or self.agl_at_bootstrap is None:
                self.agl_at_bootstrap = agl_m
            z_meas = -(agl_m - self.agl_at_bootstrap)
            # Complementary blend, not a hard snap -- a hard snap injects a step
            # into the pose chain that GICP then tries to undo on the next frame.
            alpha = 0.15
            T_out[2, 3] = (1.0 - alpha) * T_out[2, 3] + alpha * z_meas

        return T_out

    def _build_point_to_plane_hessian(self, source, target, correspondence_set, T):
        """Stage 4B: Build the 6x6 point-to-plane information matrix from GICP correspondences.
        
        Unlike Open3D's get_information_matrix_from_point_clouds (which uses
        point-to-point counting and CANNOT detect translational degeneracy),
        this builds H = Σ J_i^T J_i from point-to-plane Jacobians:
          J_i = [(R·p_i) × n_i | n_i]
        where p_i is the transformed source point and n_i is the target normal.
        """
        pts = np.asarray(source.points)
        tgt_normals = np.asarray(target.normals)
        corr = np.asarray(correspondence_set)
        
        if len(corr) == 0:
            return np.zeros((6, 6))
            
        src_idx = corr[:, 0]
        tgt_idx = corr[:, 1]
        
        p = pts[src_idx]
        n = tgt_normals[tgt_idx]
        
        # Transform source points by current GICP estimate T
        R = T[:3, :3]
        p_rot = p @ R.T  # (N, 3)
        
        # J_rot = p_rot × n
        J_rot = np.cross(p_rot, n)
        J = np.hstack((J_rot, n))  # (N, 6)
        
        r_ref = 10.0
        r2 = np.clip(np.sum(p_rot**2, axis=1), r_ref**2, None)
        W = (r_ref ** 2) / r2
        H = J.T @ (W[:, None] * J)  # (6, 6)
        return H

    def _project_onto_observable_subspace(self, T_pred: np.ndarray, T_gicp: np.ndarray,
                                           H: np.ndarray):
        """Stage 4B: accept GICP's translation correction only along
        eigen-directions of its own information matrix that are actually
        well-constrained; unobservable directions keep the RIO/IMU-predicted
        prior.
        """
        H_tt = H[3:6, 3:6]
        H_tt = 0.5 * (H_tt + H_tt.T)  # enforce exact symmetry (float safety before eigh)
        try:
            eigvals, V = np.linalg.eigh(H_tt)
        except np.linalg.LinAlgError:
            T_out = T_pred.copy()
            T_out[:3, :3] = T_gicp[:3, :3]
            return T_out, np.zeros(3, dtype=bool)

        eigvals = np.clip(eigvals, 0.0, None)          # PSD safety net
        eig_max = max(float(eigvals.max()), 1e-9)
        observable = (eigvals >= self.lambda_min_observable) & (eigvals >= self.observable_ratio * eig_max)

        t_pred_w = T_pred[:3, 3]
        t_gicp_w = T_gicp[:3, 3]
        delta_eig = V.T @ (t_gicp_w - t_pred_w)
        delta_eig_filtered = delta_eig * observable
        delta_filtered = V @ delta_eig_filtered

        T_out = T_gicp.copy()          # GICP's rotation is kept as-is
        T_out[:3, 3] = t_pred_w + delta_filtered
        return T_out, observable

    def _ground_plane_split(self, pcd: o3d.geometry.PointCloud):
        """Sec. 2.3: separate the dominant ground plane from off-plane structure.
        Ground constrains the vertical axis; off-plane points are the ones that
        actually disambiguate in-plane (x,y) translation."""
        if len(pcd.points) < 20:
            return pcd, o3d.geometry.PointCloud()
        try:
            plane_model, inliers = pcd.segment_plane(
                distance_threshold=self.plane_threshold, ransac_n=3, num_iterations=200)
        except RuntimeError:
            return pcd, o3d.geometry.PointCloud()
        ground = pcd.select_by_index(inliers)
        off_plane = pcd.select_by_index(inliers, invert=True)
        return ground, off_plane

    def _adaptive_covariances(self, pcd: o3d.geometry.PointCloud, k: int = 8):
        """Eq.(21): per-point covariance from k-NN, feeding GICP's generalized
        cost instead of an isotropic sensor-noise assumption."""
        pcd.estimate_covariances(
            search_param=o3d.geometry.KDTreeSearchParamKNN(knn=min(k, max(3, len(pcd.points) - 1))))
        return pcd

    @staticmethod
    def _forward_obstacle_range(pcd: o3d.geometry.PointCloud,
                                 half_angle_deg: float = 20.0,
                                 min_x: float = 0.3) -> float:
        """Closest filtered point (any class -- ground or off-plane, since a
        wall and a rising terrain feature are both something you don't want
        to fly into) within a forward safety cone around the body +x axis.
        This is a cheap, low-latency reactive-avoidance signal computed once
        per keyframe and piggybacked on the pose packet, NOT a substitute for
        the SLAM map -- it's the fast path for nav_node.py's braking logic,
        the map is the slow path for planning around what the cone can't see
        yet. Returns +inf if nothing is in the cone within sensor range."""
        pts = np.asarray(pcd.points)
        if pts.shape[0] == 0:
            return float('inf')
        x = pts[:, 0]
        r = np.linalg.norm(pts, axis=1)
        r_safe = np.clip(r, 1e-3, None)
        cos_half = math.cos(math.radians(half_angle_deg))
        in_cone = (x > min_x) & ((x / r_safe) > cos_half)
        if not np.any(in_cone):
            return float('inf')
        return float(np.min(r[in_cone]))

    def process_keyframe(self, xyz_body: np.ndarray, v_radial: np.ndarray,
                          t: float, omega: np.ndarray = None, agl_m: float | None = None) -> dict:
        if xyz_body.shape[0] < 8:
            return {'valid': False, 'reason': 'too_few_points'}

        pre = preprocess_frame(xyz_body, v_radial, self.persistence_tracker,
                                self.last_v_body, omega, self.filter_cfg)
        pcd = pre.pcd
        if len(pcd.points) < 8:
            return {
                'valid': False, 'reason': 'too_sparse_after_filtering',
                'n_raw': pre.n_raw, 'n_after_range': pre.n_after_range,
                'n_after_doppler': pre.n_after_doppler,
                'n_after_persistence': pre.n_after_persistence,
                'n_after_sor': pre.n_after_sor,
            }

        ground, off_plane = self._ground_plane_split(pcd)
        # Weight off-plane structure higher for in-plane observability (Sec. 3):
        # keep both, but off-plane points are what should carry the correspondence
        # search when available.
        source = pcd
        source = self._adaptive_covariances(source)
        fwd_range = self._forward_obstacle_range(pcd)

        if len(self.map_cloud.points) < 8:
            # Bootstrap: first keyframe seeds the map at the current pose.
            if self.trust_imu_yaw and self.last_attitude is not None:
                yaw_imu = self.last_attitude[2]
                cy, sy = math.cos(yaw_imu), math.sin(yaw_imu)
                self.T_world[:3, :3] = np.array([
                    [cy, -sy, 0.0],
                    [sy,  cy, 0.0],
                    [0.0, 0.0, 1.0]
                ])
                
            self._merge_into_map(source, self.T_world)
            self.pose_chain[0] = self.T_world.copy()
            return {'valid': True, 'bootstrap': True, 'T': self.T_world.copy(),
                    'n_ground': len(ground.points), 'n_off_plane': len(off_plane.points),
                    'n_raw': pre.n_raw, 'n_final': pre.n_final, 'fwd_range': fwd_range}

        target = self._adaptive_covariances(self.map_cloud)

        # Initial guess: constant-velocity prediction from RIO, if available --
        # this is the cheap substitute for Eq.(23)'s Doppler-consistency term:
        # rather than a custom weighted cost function, seed GICP close enough
        # to the Doppler-consistent solution that geometric registration
        # converges to it instead of a plane-degenerate local minimum.
        dt = (t - self.last_kf_t) if self.last_kf_t is not None else (1.0 / 5.0)
        self.last_kf_t = t

        # Frame-to-Map requires the absolute predicted pose as the initial guess.
        T_pred = self.T_world.copy()
        if self.last_v_body is not None and len(self.pose_chain) >= 1:
            # Shift translation by RIO velocity (velocity is in local body frame, 
            # so we rotate it into the world frame before adding)
            t_shift = self.T_world[:3, :3] @ (self.last_v_body * dt)
            T_pred[:3, 3] += t_shift
            
            if omega is not None:
                # FIX: Integrate gyro yaw to prevent the "frozen yaw" bug when
                # walking down a featureless hallway.
                dyaw = omega[2] * dt
                cy, sy = math.cos(dyaw), math.sin(dyaw)
                R_yaw = np.array([
                    [ cy, -sy, 0.0],
                    [ sy,  cy, 0.0],
                    [0.0, 0.0, 1.0],
                ])
                T_pred[:3, :3] = T_pred[:3, :3] @ R_yaw

        # Inject IMU gravity into the initial guess so GICP starts searching
        # from a gravity-consistent orientation (prevents tilted local minima).
        if self.imu_level_points and self.last_attitude is not None:
            T_pred = self._gravity_correct(T_pred, agl_m=agl_m)

        # -- Stage 4B: GICP always runs; degeneracy is now DISCOVERED from its
        #    own Hessian, not predicted in advance from point-cloud shape. --
        try:
            gicp_t0 = time.monotonic()
            result = o3d.pipelines.registration.registration_generalized_icp(
                source, target, self.gicp_max_corr_dist, T_pred,
                o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))
            gicp_ms = (time.monotonic() - gicp_t0) * 1000.0
            if gicp_ms > 150:
                print(f"[slam_node] ⚠️  GICP took {gicp_ms:.0f}ms "
                      f"(source={len(source.points)}, target={len(target.points)})")
        except RuntimeError as e:
            return {'valid': False, 'reason': f'gicp_exception: {e}'}

        n_corr = len(result.correspondence_set)
        if n_corr < self.min_correspondences:
            # Reject, don't force -- same philosophy as doppler_rio.py's RANSAC gate.
            return {'valid': False, 'reason': 'insufficient_correspondences', 'n_corr': n_corr,
                    'fitness': result.fitness, 'rmse': result.inlier_rmse}
        
        try:
            if not source.has_normals():
                source.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn=8))
            if not target.has_normals():
                target.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn=8))
            H = self._build_point_to_plane_hessian(source, target, result.correspondence_set, result.transformation)
            self.T_world, observable_axes = self._project_onto_observable_subspace(
                T_pred, result.transformation, H)
            n_observable = int(observable_axes.sum())
        except Exception as e:
            logging.warning(f"[slam_node] Hessian observability check failed ({e}); "
                             f"trusting RIO/IMU prior for this keyframe's translation.")
            self.T_world = T_pred
            self.T_world[:3, :3] = result.transformation[:3, :3]
            n_observable = 0

        fitness, rmse = result.fitness, result.inlier_rmse

        if self.imu_level_points and self.last_attitude is not None:
            self.T_world = self._gravity_correct(self.T_world, agl_m=agl_m)

        self.pose_chain.append(self.T_world.copy())
        self._merge_into_map(source, self.T_world)

        return {
            'valid': True, 'T': self.T_world.copy(), 't': t,
            'n_corr': n_corr, 'fitness': fitness, 'rmse': rmse,
            'n_observable_axes': n_observable,
            'n_ground': len(ground.points), 'n_off_plane': len(off_plane.points),
            'n_raw': pre.n_raw, 'n_final': pre.n_final, 'fwd_range': fwd_range,
        }

    def _merge_into_map(self, pcd_body: o3d.geometry.PointCloud, T: np.ndarray,
                         max_map_points: int = 200_000):
        pcd_world = o3d.geometry.PointCloud(pcd_body)
        pcd_world.transform(T)
        self.map_cloud += pcd_world
        self.map_cloud = self.map_cloud.voxel_down_sample(self.voxel_size)
        if len(self.map_cloud.points) > max_map_points:
            idx = np.random.default_rng().choice(
                len(self.map_cloud.points), max_map_points, replace=False)
            self.map_cloud = self.map_cloud.select_by_index(idx)


# ---------------------------------------------------------------- runtime wiring

def _save_map(slam, save_path):
    """Save the accumulated SLAM map as a .pcd file."""
    if save_path and len(slam.map_cloud.points) > 0:
        import datetime
        # If user gave a directory or no extension, auto-name with timestamp
        if os.path.isdir(save_path) or not save_path.endswith('.pcd'):
            dirname = save_path if os.path.isdir(save_path) else os.path.dirname(save_path) or '.'
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            save_path = os.path.join(dirname, f'radar_map_{ts}.pcd')
        os.makedirs(os.path.dirname(save_path) or '.', exist_ok=True)
        o3d.io.write_point_cloud(save_path, slam.map_cloud)
        n = len(slam.map_cloud.points)
        print(f"\n[slam_node] ✅ Map saved: {save_path} ({n} points)")
    elif save_path:
        print(f"\n[slam_node] ⚠️  No map points to save.")


def run(args):
    mount = TiltMount(theta_tilt_deg=args.theta_tilt_deg,
                       lever_arm=np.array([args.lever_x, args.lever_y, args.lever_z]),
                       lateral_sign=args.lateral_sign)
    filter_cfg = FilterConfig(
        leakage_radius_m=args.leakage_radius,
        min_range_m=args.min_range, max_range_m=args.max_range,
        doppler_eps_mps=args.doppler_eps,
        persistence_radius_m=args.persistence_radius,
        persistence_window=args.persistence_window,
        persistence_min_hits=args.persistence_min_hits,
        sor_nb_neighbors=args.sor_neighbors, sor_std_ratio=args.sor_std_ratio,
        voxel_size_m=args.voxel_size,
    )
    slam = RadarSLAM(mount, voxel_size=args.voxel_size,
                      gicp_max_corr_dist=args.max_corr_dist,
                      min_correspondences=args.min_correspondences,
                      filter_cfg=filter_cfg,
                      plane_threshold=args.plane_threshold,
                      trust_imu_yaw=args.trust_imu_yaw,
                      lambda_min_observable=args.lambda_min_observable,
                      observable_ratio=args.observable_ratio,
                      imu_level_points=args.imu_level_points)
    accum = KeyframeAccumulator(window_s=args.window_s)

    points_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    points_sock.bind((args.listen_ip, args.listen_port))
    points_sock.settimeout(0.5)

    rio_sock = None
    if args.rio_port:
        rio_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rio_sock.bind((args.rio_ip, args.rio_port))
        rio_sock.settimeout(0.5)

    # IMU listener (optional, Stage 3+): same latest-value-grab pattern as RIO
    imu_sock = None
    latest_omega = None   # (3,) np.ndarray or None — gyroscope angular velocity
    latest_attitude = None  # (3,) np.ndarray or None — fused [roll, pitch, yaw]
    if args.imu_port and args.imu_port > 0:
        imu_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        imu_sock.bind((args.imu_ip, args.imu_port))
        imu_sock.settimeout(0.5)
        print(f"[slam_node] IMU listener on {args.listen_ip}:{args.imu_port}")

    alt_sock = None
    latest_agl = None
    if args.alt_port and args.alt_port > 0:
        alt_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        alt_sock.bind((args.listen_ip, args.alt_port))
        alt_sock.settimeout(0.5)
        print(f"[slam_node] Altimeter listener on {args.listen_ip}:{args.alt_port}")

    # Parse comma-separated pose ports (e.g. "5011,5014") for multi-consumer
    # forwarding: nav_node, visualizer, and gps_logger each get their own port.
    pose_ports = []
    if args.pose_port:
        for tok in str(args.pose_port).split(','):
            tok = tok.strip()
            if tok:
                pose_ports.append(int(tok))
    pose_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if pose_ports else None

    # Background receivers to avoid UDP buffer overflows during GICP blocking
    rio_receiver = UdpReceiver(rio_sock) if rio_sock else None
    if rio_receiver: rio_receiver.start()
    imu_receiver = UdpReceiver(imu_sock) if imu_sock else None
    if imu_receiver: imu_receiver.start()
    alt_receiver = UdpReceiver(alt_sock) if alt_sock else None
    if alt_receiver: alt_receiver.start()
    points_receiver = UdpReceiver(points_sock)
    points_receiver.start()

    print(f"[slam_node] listening points on {args.listen_ip}:{args.listen_port}, "
          f"theta_tilt={args.theta_tilt_deg} deg, keyframe window={args.window_s}s")

    frame_i = 0
    last_imu_t = 0.0
    try:
        while True:
            # Drain any pending RIO velocity updates (non-blocking, best-effort).
            if rio_receiver is not None:
                for data in rio_receiver.drain():
                    _t, vx, vy, vz, _n, _cxx, _cyy, _czz, _ntot, _flags, _cond = RIO_PKT.unpack(data)
                    slam.update_velocity(np.array([vx, vy, vz]))

            # Drain any pending IMU updates (non-blocking, latest-value grab).
            if imu_receiver is not None:
                for data in imu_receiver.drain():
                    if len(data) >= IMU_PKT.size:
                        _t, _r, _p, _y, ox, oy, oz, _ab, _vz = IMU_PKT.unpack(data[:IMU_PKT.size])
                        latest_omega = np.array([ox, oy, oz])
                        latest_attitude = np.array([_r, _p, _y])

            if alt_receiver is not None:
                for data in alt_receiver.drain():
                    if len(data) >= 4:
                        (latest_agl,) = struct.unpack('<f', data[:4])
                        slam.update_attitude(latest_attitude)
                        last_imu_t = time.monotonic()
                        
            if latest_attitude is not None and time.monotonic() - last_imu_t > 0.1:
                latest_attitude = None
                latest_omega = None
                slam.last_attitude = None

            pkts = points_receiver.drain()
            if not pkts:
                time.sleep(0.005)
                continue
            
            for data in pkts:
                pts_radar = parse_udp_packet(data)
                if pts_radar is None:
                    continue
                t = time.monotonic()
                # Levelling is REQUIRED for flight: _gravity_correct() assumes
                # the incoming cloud is already gravity-aligned, and RIO must
                # operate in the SAME frame or the constant-velocity prediction
                # at line 474 mixes frames. Driven by the supervisor flag, and
                # fatal if attitude is missing -- silently falling back to the
                # unlevelled path is how the two nodes end up disagreeing.
                if slam.imu_level_points and latest_attitude is None:
                    logging.error("imu_level_points requested but no IMU attitude "
                                   "available -- dropping frame rather than silently "
                                   "processing it unlevelled.")
                    continue
                xyz_body = mount.to_body(pts_radar[:, :3], latest_attitude,
                                         imu_level_points=slam.imu_level_points)
                v_radial = pts_radar[:, 3]
                accum.add(t, xyz_body, v_radial)
                frame_i += 1

            if not accum.ready(min_frames=args.min_frames_per_keyframe):
                continue

            if latest_omega is not None and latest_attitude is not None:
                omega_leveled = mount.get_R(latest_attitude) @ latest_omega
            else:
                omega_leveled = latest_omega

            merged_xyz, merged_v = accum.build(slam.last_v_body, omega_hint=omega_leveled)
            accum.clear()
            accum.t_last_kf = t

            result = slam.process_keyframe(merged_xyz, merged_v, t, omega=omega_leveled, agl_m=latest_agl)
            if result['valid']:
                n_map = len(slam.map_cloud.points)
                fwd = result.get('fwd_range', float('inf'))
                print(f"[slam_node] keyframe OK  raw={result.get('n_raw')} -> "
                      f"final={result.get('n_final')}  map_pts={n_map} "
                      f"corr={result.get('n_corr', '-')} "
                      f"fitness={result.get('fitness', 0):.3f} "
                      f"rmse={result.get('rmse', 0):.3f} "
                      f"obs_axes={result.get('n_observable_axes', 0)}/3 "
                      f"ground/off_plane={result.get('n_ground')}/{result.get('n_off_plane')} "
                      f"fwd_range={fwd:.1f}m")
                if pose_out is not None:
                    T = result['T'].astype('<f8').tobytes()
                    fwd_send = fwd if math.isfinite(fwd) else 1.0e6  # 1e6 = "nothing in cone"
                    pkt = POSE_PKT_HDR.pack(t, n_map, fwd_send) + T
                    for pp in pose_ports:
                        pose_out.sendto(pkt, (args.pose_ip, pp))
            else:
                extra = ""
                if 'n_raw' in result:
                    extra = (f" (raw={result.get('n_raw')} range={result.get('n_after_range')} "
                              f"doppler={result.get('n_after_doppler')} "
                              f"persist={result.get('n_after_persistence')} "
                              f"sor={result.get('n_after_sor')})")
                elif 'n_corr' in result:
                    extra = f" (n_corr={result.get('n_corr')}/{args.min_correspondences} fitness={result.get('fitness', 0):.3f})"
                logging.info(f"keyframe REJECTED: {result.get('reason')}{extra}")
    except KeyboardInterrupt:
        print("\n[slam_node] Interrupted by user.")
    finally:
        n_poses = len(slam.pose_chain)
        n_map = len(slam.map_cloud.points)
        logging.info(f"Session summary: {n_poses} keyframes, {n_map} map points")
        if hasattr(args, 'save_pcd') and args.save_pcd:
            _save_map(slam, args.save_pcd)
        points_sock.close()
        if rio_sock:
            rio_sock.close()
        if imu_sock:
            imu_sock.close()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--listen-ip', default='127.0.0.1')
    p.add_argument('--listen-port', type=int, default=5010,
                    help="matches radar_fanout.py --slam-port default")
    p.add_argument('--rio-ip', default='127.0.0.1')
    p.add_argument('--rio-port', type=int, default=5021,
                    help="UDP port to listen for RIO's body-frame velocity (port+1)")
    p.add_argument('--alt-port', type=int, default=5032,
                    help="UDP port to listen for belly altimeter Z-updates")
    p.add_argument('--pose-ip', default='127.0.0.1')
    p.add_argument('--pose-port', default='5011',
                    help="Comma-separated UDP ports for pose forwarding "
                         "(e.g. '5011,5014'). 0 to disable.")
    p.add_argument('--theta-tilt-deg', type=float, default=90.0,
                    help="Physical mount pitch-down angle. MUST match the bench-measured "
                         "value (see Stage 5A) -- do not run with the default in production.")
    p.add_argument('--trust-imu-yaw', action=argparse.BooleanOptionalAction, default=False,
                    help="Trust IMU absolute yaw (magnetometer) instead of GICP yaw for heading")
    p.add_argument('--imu-level-points', action=argparse.BooleanOptionalAction, default=False,
                    help="Apply IMU pitch+roll leveling to point cloud coordinates. "
                         "Disable (default) for handheld/uncalibrated AHRS mounts — "
                         "only yaw is used for heading, pitch+roll are ignored for stability.")
    p.add_argument('--lateral-sign', type=float, default=1.0, choices=[1.0, -1.0],
                    help="must match doppler_rio.py's --lateral-sign exactly")
    p.add_argument('--lever-x', type=float, default=0.12)
    p.add_argument('--lever-y', type=float, default=0.0)
    p.add_argument('--lever-z', type=float, default=0.05)
    p.add_argument('--window-s', type=float, default=0.3,
                    help="keyframe accumulation window; ~4-8 raw frames at 20Hz source rate")
    p.add_argument('--min-frames-per-keyframe', type=int, default=3)
    p.add_argument('--voxel-size', type=float, default=1.5, help="meters")
    p.add_argument('--max-corr-dist', type=float, default=6.0, help="meters, GICP correspondence radius")
    p.add_argument('--min-correspondences', type=int, default=15,
                    help="reject a keyframe with fewer GICP correspondences than this; "
                         "lower for bench-scale scenes with few total points (e.g. 6-8)")
    # filters.py pipeline knobs -- defaults tuned per PIPELINE_NOTES.md; the
    # bench-test starting point is deliberately conservative (wider gates)
    # since Phase 1 has no ego-motion to help disambiguate real vs. ghost.
    p.add_argument('--leakage-radius', type=float, default=0.35, help="meters, TX/RX near-field gate")
    p.add_argument('--min-range', type=float, default=0.3, help="meters")
    p.add_argument('--max-range', type=float, default=350.0, help="meters")
    p.add_argument('--doppler-eps', type=float, default=0.20,
                    help="m/s tolerance for static-world Doppler consistency; keep in sync with doppler_rio.py's --eps")
    p.add_argument('--persistence-radius', type=float, default=1.0, help="meters")
    p.add_argument('--persistence-window', type=int, default=4, help="frames")
    p.add_argument('--persistence-min-hits', type=int, default=2, help="of persistence-window")
    p.add_argument('--sor-neighbors', type=int, default=8)
    p.add_argument('--sor-std-ratio', type=float, default=1.5)
    p.add_argument('--plane-threshold', type=float, default=0.6,
                    help="RANSAC plane distance threshold in meters for ground "
                         "segmentation; increase for flight altitude (e.g. 1.0 at 50m AGL)")
    p.add_argument('--save-pcd', default=None,
                    help="Save accumulated map as .pcd on exit. "
                         "Pass a filepath (e.g. map.pcd) or directory.")
    p.add_argument('--imu-ip', default='127.0.0.1')
    p.add_argument('--imu-port', type=int, default=0,
                    help="UDP port to receive IMU data from imu_bridge.py "
                         "(default 0 = disabled, no rotation deskew)")
    p.add_argument('--lambda-min-observable', type=float, default=10.0,
                    help="Stage 4B: Min eigenvalue of point-to-plane Hessian to consider an axis observable")
    p.add_argument('--observable-ratio', type=float, default=0.05,
                    help="Stage 4B: Min ratio to max eigenvalue to consider an axis observable")
    args = p.parse_args()
    run(args)


if __name__ == '__main__':
    main()
