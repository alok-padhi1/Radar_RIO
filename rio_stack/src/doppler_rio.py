#!/usr/bin/env python3
"""
doppler_rio.py
Doppler-RANSAC + weighted-least-squares 3D ego-velocity estimator for a
pitch-tilted 4D FMCW radar (Linpowave U300-class), consuming the UDP
Points_float stream produced by your radar_streamer.py.

Pipeline per frame:
  1. Parse (x,y,z,v) points in RADAR frame from the UDP packet.
  2. Transform points to BODY frame:  P_B = R_tilt @ P_R + lever_arm   (Eq. 2)
  3. Build body-frame LOS unit vectors u_i = P_B_i / |P_B_i|
  4. Doppler-RANSAC over {(u_i, V_i)} to reject movers / multipath ghosts
  5. Weighted least-squares refit over the inlier set -> v_body, cov(v_body)
  6. Emit (t, v_body, cov, n_inliers, n_total) at the radar's native rate

This module has no autopilot dependency; mavlink_bridge.py consumes its
output over a small local UDP forwarding link.

Known deviation from the monograph: the vendor's Points_float TLV (type 1)
carries only (x, y, z, v) -- no per-point SNR (confirmed against your own
Points_float_PySDK_UART.py, which never reads an SNR/side-info TLV). The
monograph's w_i ~ SNR_i / R_i^2 weighting is therefore not literally
available in this firmware mode. This implementation weights by 1/R_i^2
instead, which still down-weights noisy far returns without requiring a
side-info TLV. If your U300 firmware exposes a Side Info TLV, wire it in
and switch back to true SNR/R^2 weights.
"""

import logging
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
import argparse
import socket
import struct
import threading
import time
import math
from dataclasses import dataclass, field
import numpy as np

from filters import gate_doppler_consistency, FilterConfig

UDP_HEADER = struct.Struct('<I')       # points_num, matches radar_streamer.py
# Extended packet: t, vx, vy, vz, n_inliers, cxx, cyy, czz, n_total, flags, cond
# flags bit0=is_static, bit1=airborne, bit2=vz_prior_used, bit3=accel_gate_armed
FORWARD_PKT = struct.Struct('<dfffIfffIBf')  # 45 bytes
# IMU packet from imu_bridge.py (53 bytes): t, roll, pitch, yaw, wx, wy, wz, airborne, vz_ned, t_vz, t_air
IMU_PKT = struct.Struct('<dffffffBfdd')   # 53 bytes


class IMUListener:
    """Background thread that drains the latest IMU packet from imu_bridge.py.
    Implements the 'latest-value grab' synchronization pattern: the IMU runs
    at 50 Hz, doppler_rio runs at 10-20 Hz, so the freshest reading is always
    < 20 ms old."""

    def __init__(self, port: int, ip: str = '127.0.0.1'):
        self._port = port
        self._ip = ip
        self._lock = threading.Lock()
        self._omega = None      # (3,) np.ndarray: [omega_x, omega_y, omega_z] rad/s
        self._attitude = None   # (3,) np.ndarray: [roll, pitch, yaw] rad
        self._t_mono = 0.0
        self._airborne = 0      # 1 = FC confirmed IN_AIR
        self._vz_ned = 0.0      # FC EKF vertical velocity, NED +down (m/s)
        self._t_vz_ned_fc = 0.0
        self._t_airborne_fc = 0.0
        self._thread = None
        self._stop = threading.Event()

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True, name='IMUListener')
        self._thread.start()

    def _run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self._ip, self._port))
        sock.settimeout(0.5)
        print(f"[doppler_rio] IMU listener started on {self._ip}:{self._port}")
        while not self._stop.is_set():
            try:
                data, _ = sock.recvfrom(64)
                if len(data) >= IMU_PKT.size:
                    t_mono, roll, pitch, yaw, ox, oy, oz, airborne, vz_ned, t_vz, t_air = IMU_PKT.unpack(data[:IMU_PKT.size])
                    with self._lock:
                        self._omega = np.array([ox, oy, oz])
                        self._attitude = np.array([roll, pitch, yaw])
                        self._t_mono = t_mono
                        self._airborne = int(airborne)
                        self._vz_ned = float(vz_ned)
                        self._t_vz_ned_fc = t_vz
                        self._t_airborne_fc = t_air
            except socket.timeout:
                continue
            except Exception:
                continue
        sock.close()

    def get_omega(self, max_age_s: float = 0.1):
        """Returns the latest angular velocity (3,) or None if stale/unavailable."""
        with self._lock:
            if self._omega is None:
                return None
            age = time.monotonic() - self._t_mono
            if age > max_age_s:
                return None
            return self._omega.copy()

    def get_attitude(self, max_age_s: float = 0.1):
        """Returns the latest fused attitude [roll, pitch, yaw] or None if stale."""
        with self._lock:
            if self._attitude is None:
                return None
            age = time.monotonic() - self._t_mono
            if age > max_age_s:
                return None
            return self._attitude.copy()

    def get_airborne(self) -> bool:
        """Returns True if the FC reports the vehicle is in the air."""
        with self._lock:
            if time.monotonic() - self._t_airborne_fc > 2.0:
                return True # Safe default if unknown
            return bool(self._airborne)

    def get_vz_ned(self, max_age_s: float = 0.3) -> float | None:
        """Returns the FC EKF vertical velocity (NED, +down) or None if stale.
        sigma_vz on a Cube Orange with healthy baro is ~0.3 m/s; use 0.5 m/s
        as the soft-prior weight to avoid fighting genuine radar information."""
        with self._lock:
            age = time.monotonic() - self._t_vz_ned_fc
            if age > max_age_s:
                return None
            return self._vz_ned

    def stop(self):
        self._stop.set()


@dataclass
class TiltMount:
    theta_tilt_deg: float = 40.0
    lever_arm: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # Confirmed via Linpowave_visualizer_UART userguide_Points_float.pdf:
    # "Maximum distance X ... X range is +/-50 m" (symmetric -> lateral/azimuth)
    # vs "Y ... range is [0,100] m" (one-sided -> forward/range-gate). Z is
    # elevation (the visualizer's own tip formula, pitch=atan2(z,hypot(x,y)),
    # only makes sense if z is "up/down relative to boresight", and the
    # protocol PDF's own worked example shows a point at Z=-1.75 m with the
    # radar tilted down over a floor -- i.e. Z is up-positive, and this was
    # a floor return sitting BELOW the sensor). So the native radar frame is
    # (X=lateral, Y=forward, Z=up), NOT (X=forward, Y=lateral, Z=vertical) --
    # the previous TiltMount silently assumed the latter and applied the
    # pitch-down rotation to the wrong axes entirely, which is root-cause #1
    # of the Stage 1 bench failure (see SYSTEM_REFERENCE/bug report): a
    # boresight target at 1.2 m range was placed at body Y=+1.2 (1.2 m to
    # the vehicle's right) instead of body X=+1.2 (1.2 m ahead), leaving
    # forward/vertical velocity structurally unobservable.
    #
    # lateral_sign: X_R's polarity (which physical direction is "+X") isn't
    # nailed down by the documents alone. Default +1 assumes a standard
    # right-handed sensor frame (X=right, Y=forward, Z=up -- the same
    # handedness as ENU, X x Y = Z checks out). VALIDATE THIS ON THE BENCH:
    # place a single corner reflector/target physically to the RIGHT of the
    # radar's forward boresight and confirm the resulting body-frame Y comes
    # out positive (right, per the FRD convention used everywhere else in
    # this project). If it comes out negative, set lateral_sign=-1.0.
    lateral_sign: float = 1.0

    def __post_init__(self):
        th = math.radians(self.theta_tilt_deg)
        c, s = math.cos(th), math.sin(th)
        # R_tilt: Eq. (1), pure pitch-down rotation about body +y, defined
        # for an INPUT already in (forward, lateral, down) axes.
        R_tilt = np.array([
            [c,   0.0, -s],
            [0.0, 1.0, 0.0],
            [s,   0.0,  c],
        ])
        # P: native radar (X_lat, Y_fwd, Z_up) -> pre-tilt body-aligned
        # (X_fwd, Y_lat, Z_down). This is the fix -- everything else about
        # the tilt math (Eq. 1-3 of the monograph) is unchanged, it was just
        # being fed the wrong input axes.
        self.P = np.array([
            [0.0,              1.0, 0.0],   # X_B0 =  Y_R   (forward)
            [self.lateral_sign, 0.0, 0.0],  # Y_B0 = ±X_R   (lateral/right)
            [0.0,              0.0, -self.lateral_sign],  # Z_B0 = -+Z_R   (down)
        ])
        # R_R^B(theta_tilt) = R_tilt @ P: apply the axis permutation first,
        # then the mechanical pitch-down tilt, exactly as Eq.(2) expects.
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

    def current_rotation(self, attitude: np.ndarray = None) -> np.ndarray:
        """Returns the rotation matrix to_body() would use for this attitude state."""
        if attitude is not None:
            return self.get_R(attitude) @ self.R_static
        return self.R_static

    def to_body(self, points_radar_xyz: np.ndarray, attitude: np.ndarray = None,
                imu_level_points: bool = True) -> np.ndarray:
        """Transforms radar points to body frame. If attitude [roll, pitch, yaw] is provided
        AND imu_level_points is True, dynamically rotates the points into a gravity-leveled frame.
        
        When imu_level_points=False (recommended for handheld/uncalibrated AHRS), only the
        static mechanical tilt (R_static) is used. This prevents systematic Vz bias from
        uncalibrated AHRS pitch offsets.
        """
        if attitude is not None and imu_level_points:
            R_used = self.current_rotation(attitude)
            R_level = self.get_R(attitude)
            # The lever arm must ALSO be leveled! 
            return points_radar_xyz @ R_used.T + (R_level @ self.lever_arm)
        
        # Static-only path: apply mechanical tilt only, no IMU pitch/roll.
        return points_radar_xyz @ self.R_static.T + self.lever_arm


def parse_udp_packet(data: bytes):
    """Matches the wire format written by radar_streamer.py:
       uint32 points_num, then points_num * (f32 x,y,z,v), radar frame."""
    if len(data) < 4:
        return None
    (points_num,) = UDP_HEADER.unpack_from(data, 0)
    expected = 4 + points_num * 16
    if points_num == 0 or len(data) < expected:
        return None
    pts = np.frombuffer(data, dtype='<f4', count=points_num * 4, offset=4)
    return pts.reshape(points_num, 4)  # columns: x, y, z, v (radar frame)


def doppler_ransac(u_body: np.ndarray, v_radial: np.ndarray,
                    eps: float = 0.15, iters: int = 80,
                    min_inlier_ratio: float = 0.35, max_speed_mps: float = 25.0,
                    static_margin_frac: float = 0.12, static_margin_min: int = 3,
                    cond_reject_threshold: float = 30.0, rng=None, force_2d: bool = False,
                    # ── Static-hypothesis hardening (2026-09-18 forensics) ──
                    airborne: bool = False,
                    static_min_ratio: float = 0.70,
                    static_min_points: int = 10,
                    min_points_moving: int = 8):
    """
    Vectorized Doppler-RANSAC with a static-hypothesis margin requirement
    and a condition-number gate against near-planar geometry degeneracy.

    u_body:   (N,3) unit LOS vectors in body frame
    v_radial: (N,)  measured radial speed per point (frame-invariant)
    Model: V_i = -u_i^T v_body  =>  inlier test |V_i + u_i^T v_k| < eps

    Why the plain "highest inlier count wins" RANSAC above degenerates on a
    flat floor at close range: when all points come from a narrow, nearly
    planar patch (Sec. 1.4/3.4's "narrow angular diversity ill-conditions
    A^T W A"), essentially every u_i is nearly parallel. In that regime, a
    RANDOM 3-point velocity hypothesis can coincidentally satisfy the
    residual test on as many (or more) points as the TRUE v=0 hypothesis,
    purely because the geometry can't actually distinguish "no motion" from
    "some particular large motion along the narrow direction the cone
    can't see." Observed on a static bench: v=0 explaining 14-16/24 points,
    followed a few iterations later by a random triad solve of 4-6 m/s
    explaining 15-16/24 -- a difference of ONE point, well within noise,
    but enough for a strict ">" comparison to throw out the correct answer.

    Two independent guards:
      1. Margin requirement (the "BIC-lite" ask): a moving hypothesis is
         only preferred over the static (0-parameter) hypothesis if it
         explains clearly more points, not merely one more by chance --
         `score_moving >= score_static + max(static_margin_min,
         ceil(static_margin_frac * N))`. This is the practical stand-in for
         an information-criterion penalty (moving has 3 free params vs
         static's 0) without needing an explicit likelihood model.
      2. Condition-number gate on the ACCEPTED moving hypothesis' own
         inlier LOS matrix: even after clearing the margin, if the winning
         inlier set's LOS vectors are still nearly parallel (large
         cond(A)), the 3D solve is not actually well-constrained and is
         rejected outright -- reported as a gap, not published as a
         velocity.
      3. (Inside the sampling loop) degenerate-triad guard: minimal 3-point
         samples whose own condition number is extreme are skipped before
         even attempting a solve -- they're numerically unstable and are
         exactly the source of the spurious high-velocity hypotheses above.

    Returns (mask, is_static: bool, cond_number: float) or None if rejected.
    """
    n = u_body.shape[0]
    if n < 3:
        return None
    rng = rng or np.random.default_rng()

    res_zero = np.abs(v_radial)
    mask_zero = res_zero < eps
    score_zero = int(mask_zero.sum())

    # ── Static-hypothesis credibility gate (2026-09-18 forensics) ──
    # Forensic analysis proved that 8-9 inliers with near-zero Doppler appear
    # on 100% of zero-publishing frames in all five flights, at every altitude,
    # speed and mount angle. These are airframe self-returns (landing gear,
    # rotor hub, near-field leakage) — not ground. They give a permanently
    # available 'static' hypothesis that wins the margin test whenever real
    # ground returns thin out at altitude. The old threshold (score_zero/n >=
    # 0.25 OR score_zero >= 3) let 3 self-return points out of 12 declare the
    # vehicle parked mid-flight. The new gate requires near-unanimity.
    static_credible = (
        score_zero >= static_min_points and
        (score_zero / n) >= static_min_ratio
    )
    if airborne:
        # In free-flight a true static world is essentially impossible.
        # Demand overwhelming evidence and prefer a declared gap (which
        # lets the EKF coast on IMU) over a confident fabricated zero.
        static_credible = False

    if n < min_points_moving:
        # Too few points for a well-conditioned 3-DOF solve. Do NOT publish
        # a fabricated zero; report a gap instead.
        if static_credible and not airborne:
            return mask_zero, True, 1.0
        return None

    best_mask, best_score = None, -1
    for _ in range(iters):
        idx = rng.choice(n, size=3, replace=False)
        A_sub, b_sub = -u_body[idx], v_radial[idx]
        # Degenerate-triad guard: skip minimal samples whose 3 LOS vectors
        # are nearly coplanar/parallel -- solving through them is
        # numerically unstable and is exactly what produces spurious
        # multi-m/s "hypotheses" on a near-planar floor patch.
        A_cond_sub = A_sub[:, :2] if force_2d else A_sub
        sing = np.linalg.svd(A_cond_sub, compute_uv=False)
        if sing[-1] < 1e-3 or (sing[0] / max(sing[-1], 1e-9)) > cond_reject_threshold:
            continue
        try:
            if force_2d:
                v_k_2d, _, _, _ = np.linalg.lstsq(A_sub[:, :2], b_sub, rcond=1e-2)
                v_k = np.array([v_k_2d[0], v_k_2d[1], 0.0])
            else:
                v_k, _, _, _ = np.linalg.lstsq(A_sub, b_sub, rcond=1e-2)
        except (np.linalg.LinAlgError, ValueError):
            continue
        if np.linalg.norm(v_k) > max_speed_mps:
            continue
        residual = np.abs(v_radial + u_body @ v_k)
        mask = residual < eps
        score = int(mask.sum())
        if score > best_score:
            best_score, best_mask = score, mask

    margin = max(static_margin_min, int(math.ceil(static_margin_frac * n)))

    if best_mask is None or best_score <= score_zero + margin:
        # Nothing beat static outright or by sufficient margin.
        if static_credible:
            return mask_zero, True, 1.0
        # Prefer a gap over a fabricated zero (especially when airborne).
        return None

    if best_score / n < min_inlier_ratio:
        return None

    A_cond = -u_body[best_mask]
    if force_2d:
        A_cond = A_cond[:, :2]
    cond = float(np.linalg.cond(A_cond))
    if cond > cond_reject_threshold:
        return None

    return best_mask, False, cond


def weighted_refit(u_body, v_radial, ranges, mask, max_speed_mps: float = 25.0,
                   force_2d: bool = False, vz_prior: float | None = None):
    """Eq. (9): v = (A^T W A)^-1 A^T W b using weighted least-squares with SVD conditioning."""
    A = -u_body[mask]
    if force_2d:
        A = A[:, :2]
    b = v_radial[mask]
    w = 1.0 / np.clip(ranges[mask], 0.5, None) ** 2
    sqrt_w = np.sqrt(w)
    A_w = A * sqrt_w[:, None]
    b_w = b * sqrt_w
    # Inject vertical-velocity prior to break the Vx/Vz null direction
    if not force_2d:
        A_w, b_w = _augment_with_vz_prior(A_w, b_w, vz_prior)
    try:
        v_res, _, _, _ = np.linalg.lstsq(A_w, b_w, rcond=1e-2)
        if force_2d:
            v_body = np.array([v_res[0], v_res[1], 0.0])
        else:
            v_body = v_res[:3]
        ATA = A_w.T @ A_w
        cov_sub = np.linalg.pinv(ATA, rcond=1e-2)
        if force_2d:
            cov = np.zeros((3, 3))
            cov[:2, :2] = cov_sub
            cov[2, 2] = 1.0
        else:
            cov = cov_sub
    except (np.linalg.LinAlgError, ValueError):
        return None, None

    if np.linalg.norm(v_body) > max_speed_mps:
        return None, None
    return v_body, cov


def polar_uncertainty_weights(xyz_radar: np.ndarray, u_body: np.ndarray, ranges: np.ndarray,
                               R_used: np.ndarray, v_estimate: np.ndarray,
                               sigma_r_m: float, sigma_az_rad: float, sigma_el_rad: float,
                               sigma_v_mps: float, r_floor: float = 0.5) -> np.ndarray:
    """Stage 4C: maps the radar's native spherical measurement uncertainty
    into a per-point scalar weight for the velocity solve."""
    r_native = np.clip(np.linalg.norm(xyz_radar, axis=1), r_floor, None)
    x, y, z = xyz_radar[:, 0], xyz_radar[:, 1], xyz_radar[:, 2]
    az = np.arctan2(x, y)
    el = np.arctan2(z, np.hypot(x, y))
    caz, saz = np.cos(az), np.sin(az)
    cel, sel = np.cos(el), np.sin(el)

    n = len(r_native)
    J = np.zeros((n, 3, 3))
    J[:, 0, 0] = cel * saz
    J[:, 0, 1] = r_native * cel * caz
    J[:, 0, 2] = -r_native * sel * saz
    J[:, 1, 0] = cel * caz
    J[:, 1, 1] = -r_native * cel * saz
    J[:, 1, 2] = -r_native * sel * caz
    J[:, 2, 0] = sel
    J[:, 2, 1] = 0.0
    J[:, 2, 2] = r_native * cel

    Sigma_polar = np.zeros((n, 3, 3))
    Sigma_polar[:, 0, 0] = sigma_r_m ** 2
    Sigma_polar[:, 1, 1] = sigma_az_rad ** 2
    Sigma_polar[:, 2, 2] = sigma_el_rad ** 2

    Sigma_xyz_radar = np.einsum('nij,njk,nlk->nil', J, Sigma_polar, J)
    Sigma_xyz_body = np.einsum('ij,njk,lk->nil', R_used, Sigma_xyz_radar, R_used)

    ranges_safe = np.clip(ranges, r_floor, None)
    u_dot_v = u_body @ v_estimate
    v_perp = v_estimate[None, :] - u_dot_v[:, None] * u_body           # (N,3)
    sigma_dir2 = np.einsum('ni,nij,nj->n', v_perp, Sigma_xyz_body, v_perp) / (ranges_safe ** 2)

    sigma_total2 = sigma_v_mps ** 2 + sigma_dir2
    return 1.0 / np.clip(sigma_total2, 1e-6, None)


def irls_refit(u_body, v_radial, xyz_radar, ranges, R_used, v_seed,
                sigma_r_m, sigma_az_rad, sigma_el_rad, sigma_v_mps,
                huber_delta_mps=0.20, max_iters=4, tol_mps=1e-3,
                gross_outlier_mult=10.0, max_speed_mps=25.0, force_2d=False,
                vz_prior: float | None = None):
    """Stage 4A: IRLS refinement of the RANSAC-seeded velocity."""
    v = np.asarray(v_seed, dtype=float).copy()
    max_iters = int(np.clip(max_iters, 3, 5))

    resid_seed = np.abs(v_radial + u_body @ v)
    keep = resid_seed <= gross_outlier_mult * huber_delta_mps
    if keep.sum() < 3:
        return None, None

    u_k, v_k, xyz_k, r_k = u_body[keep], v_radial[keep], xyz_radar[keep], ranges[keep]
    A_full = -u_k
    if force_2d:
        A_full = A_full[:, :2]
    b_full = v_k
    cov = None

    for _ in range(max_iters):
        if force_2d:
            v_full = np.array([v[0], v[1], 0.0])
            w_meas = polar_uncertainty_weights(xyz_k, u_k, r_k, R_used, v_full,
                                                sigma_r_m, sigma_az_rad, sigma_el_rad, sigma_v_mps)
            resid = b_full + u_k @ v_full
        else:
            w_meas = polar_uncertainty_weights(xyz_k, u_k, r_k, R_used, v,
                                                sigma_r_m, sigma_az_rad, sigma_el_rad, sigma_v_mps)
            resid = b_full + u_k @ v
        abs_r = np.abs(resid)
        w_huber = np.ones_like(abs_r)
        far = abs_r > huber_delta_mps
        w_huber[far] = huber_delta_mps / np.clip(abs_r[far], 1e-6, None)
        w = w_meas * w_huber
        sqrt_w = np.sqrt(np.clip(w, 0.0, None))

        A_w = A_full * sqrt_w[:, None]
        b_w = b_full * sqrt_w
        # Inject Vz prior each IRLS iteration to keep the null direction pinned
        if not force_2d:
            A_w, b_w = _augment_with_vz_prior(A_w, b_w, vz_prior)
        try:
            v_new_sub, _, _, _ = np.linalg.lstsq(A_w, b_w, rcond=1e-2)
            if force_2d:
                v_new = np.array([v_new_sub[0], v_new_sub[1], 0.0])
            else:
                v_new = v_new_sub[:3]
            ATA = A_w.T @ A_w
            cov_sub = np.linalg.pinv(ATA, rcond=1e-2)
            if force_2d:
                cov = np.zeros((3, 3))
                cov[:2, :2] = cov_sub
                cov[2, 2] = 1.0
            else:
                cov = cov_sub
        except (np.linalg.LinAlgError, ValueError):
            cov = None
            break

        if np.linalg.norm(v_new) > max_speed_mps:
            break

        delta = np.linalg.norm(v_new - v)
        v = v_new
        if delta < tol_mps:
            break

    if cov is None or np.linalg.norm(v) > max_speed_mps:
        return None, None
    return v, cov


def _augment_with_vz_prior(A_w: np.ndarray, b_w: np.ndarray,
                            vz_prior: float | None,
                            sigma_vz: float = 0.5) -> tuple[np.ndarray, np.ndarray]:
    """Append a soft pseudo-measurement [0,0,1]·v = vz_prior with weight 1/sigma_vz.

    WHY (2026-09-18 forensics §3.4 Layer 1):
    At 25-45° tilt the Vx/Vz correlation ρ is -0.90 to -0.965.  The LOS cone
    has a near-null direction n̂ = (sin θ, 0, −cos θ) along which the solver
    slides freely — forensics showed spikes aligning with n̂ at |cos|=0.78-0.87
    while GPS read 0.00 m/s.  A finite-weight Vz row breaks that freedom without
    clamping Vz to a fixed value (which is what --force-2d did and which leaked
    phantom Vx).  sigma_vz=0.5 matches Cube Orange baro+rangefinder accuracy.
    Do NOT set it below 0.2 — that starts fighting genuine radar information.

    Sign discipline: LOCAL_POSITION_NED.vz is NED +down.  FRD body frame also
    uses +down for Z.  They agree — pass vz_ned straight through, do not negate.
    """
    if vz_prior is None or not math.isfinite(vz_prior):
        return A_w, b_w
    w = 1.0 / max(sigma_vz, 0.2)
    A_w = np.vstack([A_w, np.array([[0.0, 0.0, 1.0]]) * w])
    b_w = np.concatenate([b_w, [vz_prior * w]])
    return A_w, b_w


class DopplerRIO:
    def __init__(self, mount: TiltMount, min_range=0.3, max_range=350.0,
                 eps=0.40, iters=60, min_inlier_ratio=0.35,
                 static_margin_frac=0.12, static_margin_min=3,
                 cond_reject_threshold=30.0, deadband_mps=0.05,
                 imu_listener: IMUListener | None = None,
                 huber_delta_mps=0.20, irls_max_iters=4, irls_tol_mps=1e-3,
                 gross_outlier_mult=10.0,
                 sigma_r_m=0.10, sigma_az_rad=math.radians(2.0),
                 sigma_el_rad=math.radians(4.0), sigma_v_mps=0.05,
                 imu_level_points: bool = False,
                 force_2d: bool = False, max_speed_mps: float = 25.0,
                 static_min_ratio: float = 0.70, static_min_points: int = 10,
                 min_points_moving: int = 8, max_accel_mps2: float = 15.0,
                 max_sigma_v_mps: float = 0.60, leakage_radius_m: float = 0.35):
        self.mount = mount
        self.force_2d = force_2d
        self.max_speed_mps = max_speed_mps
        if force_2d:
            logging.warning("force_2d ENABLED: Vz is being clamped to zero. "
                            "This is only valid for a NADIR mount. At any tilt "
                            "between 10 and 80 deg it injects large phantom Vx. "
                            "DO NOT FLY WITH THIS ON.")
        self.min_range, self.max_range = min_range, max_range
        self.leakage_radius_m = leakage_radius_m
        self.eps, self.iters, self.min_inlier_ratio = eps, iters, min_inlier_ratio
        self.static_margin_frac = static_margin_frac
        self.static_margin_min = static_margin_min
        self.cond_reject_threshold = cond_reject_threshold
        self.deadband_mps = deadband_mps
        self.imu_listener = imu_listener
        self.huber_delta_mps = huber_delta_mps
        self.irls_max_iters = irls_max_iters
        self.irls_tol_mps = irls_tol_mps
        self.gross_outlier_mult = gross_outlier_mult
        self.sigma_r_m = sigma_r_m
        self.sigma_az_rad = sigma_az_rad
        self.sigma_el_rad = sigma_el_rad
        self.sigma_v_mps = sigma_v_mps
        self.imu_level_points = imu_level_points
        # ── Forensics fixes (2026-09-18) ──
        # R1 static hardening knobs (passed to doppler_ransac)
        self.static_min_ratio   = static_min_ratio
        self.static_min_points  = static_min_points
        self.min_points_moving  = min_points_moving
        # R2-L2 acceleration gate
        self._v_prev: np.ndarray | None = None
        self._t_prev: float | None = None
        self._yaw_prev: float | None = None
        self.max_accel_mps2: float = max_accel_mps2
        self._n_accel_rejected: int = 0
        # R2-L3 posterior sigma gate
        self.max_sigma_v_mps: float = max_sigma_v_mps

    def process_frame(self, points_radar: np.ndarray, t_frame: float):
        """points_radar: (N,4) [x,y,z,v] in RADAR frame. Returns a result dict."""
        if points_radar.shape[0] < 3:
            return {'t': t_frame, 'valid': False, 'n_total': int(points_radar.shape[0])}

        attitude = None
        if self.imu_listener is not None:
            attitude = self.imu_listener.get_attitude()

        xyz_radar_native = points_radar[:, 0:3]
        xyz_b = self.mount.to_body(xyz_radar_native, attitude,
                                    imu_level_points=self.imu_level_points)
        v_meas = points_radar[:, 3]
        ranges = np.linalg.norm(xyz_b, axis=1)

        keep = (ranges > max(self.min_range, self.leakage_radius_m)) & (ranges < self.max_range)
        if keep.sum() < 3:
            return {'t': t_frame, 'valid': False, 'n_total': int(keep.sum())}
        xyz_b, v_meas, ranges = xyz_b[keep], v_meas[keep], ranges[keep]
        xyz_radar_native = xyz_radar_native[keep]
        u_body = xyz_b / ranges[:, None]

        # ── IMU rotation compensation (Stage 3) ──
        # When the radar rotates at angular velocity omega, each point p_i
        # experiences an apparent Doppler shift from the rotational velocity
        # omega × p_i projected onto the LOS direction u_i. We subtract this
        # contribution so RANSAC/WLS see only translational velocity.
        #   v_adjusted_i = v_meas_i + dot(u_i, cross(omega, p_i))
        omega = None
        if self.imu_listener is not None:
            omega = self.imu_listener.get_omega()

        if self._v_prev is not None:
            fc = FilterConfig(doppler_eps_mps=self.eps)
            doppler_keep = gate_doppler_consistency(
                xyz_b, v_meas, self._v_prev, omega, fc, self.mount.lever_arm
            )
            if doppler_keep.sum() < 3:
                return {'t': t_frame, 'valid': False, 'n_total': int(doppler_keep.sum()), 'reason': 'doppler_gate'}
            xyz_b, v_meas, ranges, u_body = xyz_b[doppler_keep], v_meas[doppler_keep], ranges[doppler_keep], u_body[doppler_keep]

        if omega is not None:
            # Correct for the sensor's own velocity due to body rotation about
            # the lever arm:  v_sensor = v_body + omega x r_lever.
            #
            # NOTE: the previous implementation used (omega x p_i) where p_i is
            # the POINT position. That term is identically zero, because
            # (omega x p) is perpendicular to p and u = p/|p|. It compensated
            # nothing. The rotational term enters ONLY through the lever arm,
            # because the lever arm is what actually moves the sensor.
            # See implementation_plan.md defect B7.
            if attitude is not None and self.imu_level_points:
                omega_used = self.mount.get_R(attitude) @ omega
                lever_used = self.mount.get_R(attitude) @ self.mount.lever_arm
            else:
                omega_used = omega
                lever_used = self.mount.lever_arm
            v_lever = np.cross(omega_used, lever_used)        # (3,) m/s
            v_adjusted = v_meas + (u_body @ v_lever)          # (N,)
        else:
            v_adjusted = v_meas

        # Determine airborne state from the IMU listener (safe default: False)
        is_airborne = False
        if self.imu_listener is not None:
            is_airborne = self.imu_listener.get_airborne()

        ransac_result = doppler_ransac(
            u_body, v_adjusted, self.eps, self.iters, self.min_inlier_ratio,
            static_margin_frac=self.static_margin_frac,
            static_margin_min=self.static_margin_min,
            cond_reject_threshold=self.cond_reject_threshold,
            force_2d=self.force_2d,
            airborne=is_airborne,
            max_speed_mps=self.max_speed_mps,
            static_min_ratio=self.static_min_ratio,
            static_min_points=self.static_min_points,
            min_points_moving=self.min_points_moving)
        if ransac_result is None:
            # Sec. 1.3/3.4: expected, recoverable gap -- not a fault. Caller
            # (the EKF) should widen covariance / coast, not disarm.
            return {'t': t_frame, 'valid': False, 'n_total': int(keep.sum())}
        mask, is_static, cond = ransac_result

        if is_static:
            v_body = np.zeros(3)
            # Tight covariance on the bench is fine, but in flight it's a flyaway
            # mechanism because degenerate geometry triggers the static fallback.
            # Using 0.25 (σ = 0.5 m/s) unconditionally allows the bench ZUPT to
            # mostly work while removing the flyaway mechanism in flight.
            cov = np.eye(3) * 0.25
        else:
            # ── R2-L1: Vz prior — break the Vx/Vz null-direction degeneracy ──
            vz_prior = None
            if self.imu_listener is not None:
                vz_prior = self.imu_listener.get_vz_ned()

            v_seed, _ = weighted_refit(u_body, v_adjusted, ranges, mask,
                                        force_2d=self.force_2d,
                                        max_speed_mps=self.max_speed_mps,
                                        vz_prior=vz_prior)
            if v_seed is None:
                return {'t': t_frame, 'valid': False, 'n_total': int(keep.sum())}

            R_used = self.mount.current_rotation(attitude)
            v_body, cov = irls_refit(
                u_body, v_adjusted, xyz_radar_native, ranges, R_used, v_seed,
                sigma_r_m=self.sigma_r_m, sigma_az_rad=self.sigma_az_rad,
                sigma_el_rad=self.sigma_el_rad, sigma_v_mps=self.sigma_v_mps,
                huber_delta_mps=self.huber_delta_mps, max_iters=self.irls_max_iters,
                tol_mps=self.irls_tol_mps, gross_outlier_mult=self.gross_outlier_mult,
                force_2d=self.force_2d, max_speed_mps=self.max_speed_mps,
                vz_prior=vz_prior)

            if v_body is None:
                return {'t': t_frame, 'valid': False, 'n_total': int(keep.sum())}

            # ── R2-L3: Posterior sigma gate ──
            # The covariance already computed by irls_refit reflects how well
            # the inlier set constrains each velocity axis. A large sigma on
            # any axis means the null direction is still active.
            sigma = np.sqrt(np.clip(np.diag(cov), 0.0, None))
            if np.any(sigma > self.max_sigma_v_mps):
                logging.info(
                    f"RIO posterior sigma gate: sigma_v={sigma.round(3)} m/s "
                    f"exceeds {self.max_sigma_v_mps} -- null direction active, rejecting frame")
                return {'t': t_frame, 'valid': False, 'n_total': int(keep.sum()),
                        'reason': 'sigma_gate'}
            if v_body is None:
                return {'t': t_frame, 'valid': False, 'n_total': int(keep.sum())}

        # ── R2-L2: Acceleration gate ──
        # A multirotor cannot change horizontal velocity by >1.5 g between
        # 100 ms frames. Every spike in the 2026-09-18 dataset violated this
        # by 5-20x. v_prev is only updated on ACCEPTED frames so the gate
        # cannot 'walk along' behind a spike.
        if self._v_prev is not None and self._t_prev is not None and self._yaw_prev is not None:
            _dt = t_frame - self._t_prev
            if 0.0 < _dt < 0.5:
                # Rotate velocities to NED for comparison to remove yaw-turn artifacts
                curr_yaw = attitude[2] if attitude is not None else 0.0
                c1, s1 = math.cos(self._yaw_prev), math.sin(self._yaw_prev)
                R1 = np.array([[c1, -s1, 0], [s1, c1, 0], [0, 0, 1]])
                c2, s2 = math.cos(curr_yaw), math.sin(curr_yaw)
                R2 = np.array([[c2, -s2, 0], [s2, c2, 0], [0, 0, 1]])
                
                v1_ned = R1 @ self._v_prev
                v2_ned = R2 @ v_body
                implied_accel = float(np.linalg.norm(v2_ned - v1_ned) / _dt)
                if implied_accel > self.max_accel_mps2:
                    self._n_accel_rejected += 1
                    logging.warning(
                        f"RIO accel gate: {implied_accel:.1f} m/s² > "
                        f"{self.max_accel_mps2} m/s² limit "
                        f"(v_prev={self._v_prev.round(2)} → v={v_body.round(2)}, "
                        f"dt={_dt:.3f}s) — gap, not a fault")
                    return {'t': t_frame, 'valid': False, 'n_total': int(keep.sum()),
                            'reason': 'accel_gate'}

        # Deadband: snap near-zero solves to exactly zero. Below this speed
        # you're inside the sensor/estimator noise floor, not measuring real
        # motion -- publishing e.g. 0.03 m/s as "motion" just adds noise to
        # anything integrating this downstream (dead-reckoning, SLAM deskew).
        if np.linalg.norm(v_body) < self.deadband_mps:
            v_body = np.zeros(3)

        # Update kinematic state only on accepted frames
        self._v_prev = v_body.copy()
        self._t_prev = t_frame
        self._yaw_prev = attitude[2] if attitude is not None else 0.0

        return {
            't': t_frame, 'valid': True,
            'v_body': v_body, 'cov_v': cov,
            'n_inliers': int(mask.sum()), 'n_total': int(keep.sum()),
            'is_static': is_static, 'cond': cond,
            'omega': omega,
            'airborne': is_airborne,
            'vz_prior_used': (vz_prior is not None) if not is_static else False,
            'attitude': attitude,
        }


def _parse_forward_destinations(args) -> list[tuple[str, int]]:
    """Build the list of (ip, port) destinations for RIO velocity fan-out.
    Accepts both the legacy --forward-port (single destination) and the new
    --forward-ports (comma-separated ip:port or just port list)."""
    dests = []
    # Legacy single-destination flag
    if args.forward_port:
        dests.append((args.forward_ip, args.forward_port))
    # New multi-destination flag
    if args.forward_ports:
        for spec in args.forward_ports.split(','):
            spec = spec.strip()
            if not spec:
                continue
            if ':' in spec:
                ip, port_s = spec.rsplit(':', 1)
                dests.append((ip, int(port_s)))
            else:
                dests.append((args.forward_ip, int(spec)))
    return dests


def run_udp_loop(args):
    mount = TiltMount(theta_tilt_deg=args.theta_tilt_deg,
                       lever_arm=np.array([args.lever_x, args.lever_y, args.lever_z]),
                       lateral_sign=args.lateral_sign)

    # ── IMU listener (optional, Stage 3+) ──
    imu_listener = None
    if args.imu_port and args.imu_port > 0:
        imu_listener = IMUListener(port=args.imu_port, ip=args.listen_ip)
        imu_listener.start()

    if getattr(args, 'force_2d', False) and 10.0 < args.theta_tilt_deg < 80.0:
        raise SystemExit(
            f"REFUSING: --force-2d with --theta-tilt-deg {args.theta_tilt_deg} is "
            f"unsafe. At this tilt the Vx/Vz correlation exceeds 0.9 and clamping "
            f"Vz injects phantom forward velocity. See implementation_plan.md Sec 2.8.")

    rio = DopplerRIO(mount, max_range=args.max_range, eps=args.eps,
                      min_inlier_ratio=args.min_inlier_ratio,
                      static_margin_frac=args.static_margin_frac,
                      force_2d=getattr(args, 'force_2d', False),
                      static_margin_min=args.static_margin_min,
                      cond_reject_threshold=args.cond_reject_threshold,
                      deadband_mps=args.deadband,
                      imu_listener=imu_listener,
                      huber_delta_mps=args.huber_delta,
                      irls_max_iters=args.irls_max_iters,
                      irls_tol_mps=args.irls_tol,
                      gross_outlier_mult=args.gross_outlier_mult,
                      sigma_r_m=args.sigma_r,
                      sigma_az_rad=math.radians(args.sigma_az_deg),
                      sigma_el_rad=math.radians(args.sigma_el_deg),
                      sigma_v_mps=args.sigma_v,
                      imu_level_points=getattr(args, 'imu_level_points', False),
                      max_speed_mps=args.max_speed_mps,
                      static_min_ratio=args.static_min_ratio,
                      static_min_points=args.static_min_points,
                      min_points_moving=args.min_points_moving,
                      max_accel_mps2=args.max_accel_mps2,
                      max_sigma_v_mps=args.max_sigma_v_mps,
                      leakage_radius_m=args.leakage_radius_m)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.listen_ip, args.listen_port))
    sock.settimeout(0.5)
    print(f"[doppler_rio] listening on {args.listen_ip}:{args.listen_port}, "
          f"theta_tilt={args.theta_tilt_deg} deg, lever_arm="
          f"[{args.lever_x},{args.lever_y},{args.lever_z}]")

    forward_dests = _parse_forward_destinations(args)
    out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if forward_dests else None
    if forward_dests:
        print(f"[doppler_rio] forwarding RIO velocity to {len(forward_dests)} destination(s): "
              f"{', '.join(f'{ip}:{p}' for ip, p in forward_dests)}")

    while True:
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        t_frame = time.monotonic()
        pts = parse_udp_packet(data)
        if pts is None:
            continue
        result = rio.process_frame(pts, t_frame)
        if result['valid']:
            vx, vy, vz = result['v_body']
            tag = 'STATIC' if result.get('is_static') else f"cond={result.get('cond', 0):.1f}"
            omega = result.get('omega')
            imu_tag = f" ω={np.linalg.norm(omega):.2f}" if omega is not None else ""
            print(f"t={t_frame:.3f}  v_body=[{vx:+.3f} {vy:+.3f} {vz:+.3f}] m/s  "
                  f"inliers={result['n_inliers']}/{result['n_total']}  {tag}{imu_tag}")
            if out_sock:
                cov = result['cov_v']
                flags = (
                    (int(result.get('is_static', False)))
                    | (int(result.get('airborne', False)) << 1)
                    | (int(result.get('vz_prior_used', False)) << 2)
                    | (int(rio._v_prev is not None) << 3)
                )

                attitude = result.get('attitude')
                if rio.imu_level_points and attitude is not None:
                    R_level = rio.mount.get_R(attitude)
                    v_raw = R_level.T @ np.array([vx, vy, vz])
                    vx, vy, vz = v_raw[0], v_raw[1], v_raw[2]
                    cov = R_level.T @ cov @ R_level

                cxx, cyy, czz = cov[0,0], cov[1,1], cov[2,2]
                if result.get('vz_prior_used', False):
                    czz = 100.0  # Strip the prior from published covariance
                    
                max_sigma = math.sqrt(max(cxx, cyy, czz))
                if max_sigma > rio.max_sigma_v_mps:
                    print(f"t={t_frame:.3f}  RIO frame rejected (max_sigma={max_sigma:.2f} > {rio.max_sigma_v_mps})")
                    continue
                
                pkt = FORWARD_PKT.pack(
                    t_frame, vx, vy, vz, result['n_inliers'],
                    cxx, cyy, czz,
                    result['n_total'], flags,
                    float(result.get('cond', 0.0)))
                for dest_ip, dest_port in forward_dests:
                    out_sock.sendto(pkt, (dest_ip, dest_port))
        else:
            print(f"t={t_frame:.3f}  RIO frame rejected (n={result['n_total']}) "
                  f"-- gap, not a fault; downstream EKF should widen covariance")


def _selftest_jacobians():
    """Finite-difference check of polar_uncertainty_weights' analytic
    Jacobian against numerical differentiation."""
    rng = np.random.default_rng(0)
    r, az, el = 12.0, 0.3, -0.15
    def fwd(r, az, el):
        return np.array([r*math.cos(el)*math.sin(az),
                          r*math.cos(el)*math.cos(az),
                          r*math.sin(el)])
    h = 1e-6
    J_num = np.column_stack([
        (fwd(r+h, az, el) - fwd(r-h, az, el)) / (2*h),
        (fwd(r, az+h, el) - fwd(r, az-h, el)) / (2*h),
        (fwd(r, az, el+h) - fwd(r, az, el-h)) / (2*h),
    ])
    caz, saz, cel, sel = math.cos(az), math.sin(az), math.cos(el), math.sin(el)
    J_analytic = np.array([
        [cel*saz,  r*cel*caz, -r*sel*saz],
        [cel*caz, -r*cel*saz, -r*sel*caz],
        [sel,       0.0,       r*cel],
    ])
    assert np.allclose(J_num, J_analytic, atol=1e-4), "Stage 4C polar Jacobian mismatch"
    logging.info("[self_test] Stage 4C polar Jacobian PASS")


def self_test():
    """Synthetic validation: known ego-velocity + a ground-scan-shaped point
    cloud (forward-and-below, per Sec. 1.3/1.4) + injected outliers (movers /
    multipath) -> recovered velocity must match ground truth."""
    _selftest_jacobians()
    rng = np.random.default_rng(42)
    mount = TiltMount(theta_tilt_deg=40.0, lever_arm=np.array([0.12, 0.0, 0.05]))
    rio = DopplerRIO(mount, eps=0.15, iters=80, min_inlier_ratio=0.3)

    v_true = np.array([12.5, 0.8, -0.6])  # forward cruise, slight side-slip, slight climb
    n_static, n_outliers = 220, 40

    az = rng.uniform(-0.6, 0.6, n_static)
    el = rng.uniform(math.radians(28), math.radians(52), n_static)  # matches ~40deg tilt +-12deg cone
    r = rng.uniform(50, 300, n_static)
    xyz_body_static = np.stack([
        r * np.cos(el) * np.cos(az), r * np.cos(el) * np.sin(az), r * np.sin(el)
    ], axis=1)
    xyz_radar_static = (xyz_body_static - mount.lever_arm) @ mount.R_static  # inverse of Eq.(2)
    u_body_static = xyz_body_static / np.linalg.norm(xyz_body_static, axis=1, keepdims=True)
    v_radial_static = -(u_body_static @ v_true) + rng.normal(0, 0.03, n_static)

    az_o = rng.uniform(-0.6, 0.6, n_outliers)
    el_o = rng.uniform(math.radians(20), math.radians(60), n_outliers)
    r_o = rng.uniform(30, 300, n_outliers)
    xyz_body_out = np.stack([
        r_o * np.cos(el_o) * np.cos(az_o), r_o * np.cos(el_o) * np.sin(az_o), r_o * np.sin(el_o)
    ], axis=1)
    xyz_radar_out = (xyz_body_out - mount.lever_arm) @ mount.R_static
    v_radial_out = rng.uniform(-15, 15, n_outliers)  # unrelated to v_true: movers/ghosts

    xyz_radar = np.vstack([xyz_radar_static, xyz_radar_out])
    v_radial = np.concatenate([v_radial_static, v_radial_out])
    points_radar = np.column_stack([xyz_radar, v_radial])
    rng.shuffle(points_radar)  # RANSAC shouldn't care about ordering -- prove it

    result = rio.process_frame(points_radar, t_frame=0.0)
    assert result['valid'], "RIO rejected a well-conditioned synthetic frame"
    err = np.linalg.norm(result['v_body'] - v_true)
    logging.info(f"v_true      = {v_true}")
    logging.info(f"v_estimated = {result['v_body']}")
    logging.info(f"|error|     = {err:.4f} m/s")
    print(f"[self_test] inliers     = {result['n_inliers']}/{result['n_total']} "
          f"(injected {n_outliers} outliers among {n_static + n_outliers})")
    assert err < 0.15, f"velocity error too large: {err}"
    assert n_static - 10 <= result['n_inliers'] <= n_static + 10, "inlier count off from ground truth"
    logging.info("PASS")


if __name__ == '__main__':
    import sys
    if '--selftest' in sys.argv:
        self_test()
        sys.exit(0)

    p = argparse.ArgumentParser()
    p.add_argument('--selftest', action='store_true')
    p.add_argument('--listen-ip', default='127.0.0.1')
    p.add_argument('--listen-port', type=int, default=5005)
    p.add_argument('--forward-ip', default='127.0.0.1',
                    help='default IP for --forward-port and bare-port entries in --forward-ports')
    p.add_argument('--forward-port', type=int, default=0,
                    help='legacy single-destination forward; use --forward-ports for multi-consumer')
    p.add_argument('--forward-ports', type=str, default=None,
                    help='comma-separated fan-out destinations, e.g. '
                         '"5006,5007,5008" or "127.0.0.1:5006,127.0.0.1:5007"')
    p.add_argument('--theta-tilt-deg', type=float, required=True,
                    help="Physical mount pitch-down angle in degrees, MEASURED with a digital "
                         "inclinometer on the levelled airframe (implementation_plan.md Sec 3.1). "
                         "No default: a wrong tilt rotates the entire velocity vector.")
    p.add_argument('--lateral-sign', type=float, default=1.0, choices=[1.0, -1.0],
                    help="U300 native X-axis polarity; flip to -1.0 if the bench "
                         "left/right validation (see TiltMount docstring) shows it inverted")
    p.add_argument('--lever-x', type=float, required=True)
    p.add_argument('--lever-y', type=float, required=True)
    p.add_argument('--lever-z', type=float, required=True)
    p.add_argument('--max-range', type=float, default=150.0,
                    help="Reject returns beyond this range. 350 m is the datasheet's "
                         "HIGH-RCS POINT-TARGET figure, not a diffuse-ground figure. "
                         "Beyond ~1.5x the far-beam-edge ground range, returns are "
                         "multipath ghosts with near-horizontal LOS vectors that "
                         "corrupt the Vx/Vz split. See implementation_plan.md Sec 1.3.")
    p.add_argument('--leakage-radius-m', type=float, default=0.35, dest='leakage_radius_m',
                    help="Reject near-field returns inside this radius to prevent tracking self-reflections.")
    p.add_argument('--eps', type=float, default=0.40,
                    help="m/s tolerance for Doppler consistency (matches filters.py doppler_eps_mps)")
    p.add_argument('--min-inlier-ratio', type=float, default=0.25,
                    help="minimum fraction of returns that must fit hypothesis")
    p.add_argument('--static-margin-frac', type=float, default=0.12,
                    help="a moving hypothesis must beat the static (v=0) hypothesis "
                         "by at least this fraction of points, on top of --static-margin-min")
    p.add_argument('--static-margin-min', type=int, default=3)
    p.add_argument('--cond-reject-threshold', type=float, default=50.0,
                    help="reject a moving-hypothesis frame if its inlier LOS matrix "
                         "condition number exceeds this (near-planar/degenerate geometry)")
    p.add_argument('--deadband', type=float, default=0.05,
                    help="m/s; snap |v_body| below this to exactly zero")
    p.add_argument('--imu-port', type=int, default=0,
                    help="UDP port to receive IMU data from imu_bridge.py "
                         "(default 0 = disabled, no rotation compensation)")
    p.add_argument('--huber-delta', type=float, default=0.20,
                    help="Stage 4A: Huber IRLS delta in m/s")
    p.add_argument('--irls-max-iters', type=int, default=4, choices=[3, 4, 5])
    p.add_argument('--irls-tol', type=float, default=1e-3, help="m/s, IRLS early-stop tolerance")
    p.add_argument('--gross-outlier-mult', type=float, default=10.0,
                    help="hard-exclude points beyond this many multiples of --huber-delta")
    p.add_argument('--sigma-r', type=float, default=0.10, help="Stage 4C: range std-dev, meters")
    p.add_argument('--sigma-az-deg', type=float, default=2.0, help="Stage 4C: azimuth std-dev, degrees")
    p.add_argument('--sigma-el-deg', type=float, default=4.0, help="Stage 4C: elevation std-dev, degrees")
    p.add_argument('--sigma-v', type=float, default=0.05, help="Stage 4C: Doppler measurement std-dev, m/s")
    p.add_argument('--imu-level-points', action=argparse.BooleanOptionalAction, default=False,
                    help="Apply IMU pitch+roll leveling to radar points before velocity solve. "
                         "Default OFF for handheld/uncalibrated AHRS mounts.")
    p.add_argument('--force-2d', action='store_true',
                    help="DEBUG/NADIR-ONLY. Clamp Vz to zero in the velocity solve. "
                         "Refuses to combine with a tilt angle between 10 and 80 deg.")
    p.add_argument('--max-speed-mps', type=float, default=18.0,
                    help="Reject any velocity hypothesis above this. Default lowered from "
                         "25 to 18 m/s based on 2026-09-18 forensics: the U300 tracks "
                         "correctly to 12 m/s; 18 gives margin while blocking the null-direction "
                         "hallucinations that reached 17 m/s at GPS speed = 0.")
    p.add_argument('--static-min-ratio', type=float, default=0.70,
                    help="Fraction of returns that must read near-zero Doppler before the "
                         "STATIC hypothesis is accepted. Forensics showed the old 0.25 let "
                         "airframe self-returns (8-9 fixed points) publish v=0 for 50-74%% "
                         "of every flight.")
    p.add_argument('--static-min-points', type=int, default=10,
                    help="Absolute minimum number of near-zero-Doppler returns required to "
                         "accept the static hypothesis.")
    p.add_argument('--min-points-moving', type=int, default=8,
                    help="Minimum points to attempt a 3-DOF moving solve. Below this, "
                         "report a gap rather than a fabricated zero.")
    p.add_argument('--max-accel-mps2', type=float, default=15.0,
                    help="Acceleration gate: reject any frame implying more than this "
                         "m/s² change from the previous accepted frame. ~1.5 g is "
                         "generous for a multirotor.")
    p.add_argument('--max-sigma-v-mps', type=float, default=0.60,
                    help="Posterior sigma gate: reject frame if any velocity axis "
                         "uncertainty exceeds this value. High sigma means the null "
                         "direction is active and the solve is unconstrained.")
    args = p.parse_args()

    run_udp_loop(args)
