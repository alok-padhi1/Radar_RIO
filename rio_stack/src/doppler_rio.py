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


UDP_HEADER = struct.Struct('<I')       # points_num, matches radar_streamer.py
FORWARD_PKT = struct.Struct('<dfffI')  # t, vx, vy, vz, n_inliers -> mavlink_bridge.py
# IMU packet from imu_bridge.py: t_mono, roll, pitch, yaw, omega_x, omega_y, omega_z
IMU_PKT = struct.Struct('<dffffff')   # 32 bytes


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
                    t_mono, roll, pitch, yaw, ox, oy, oz = IMU_PKT.unpack(data[:IMU_PKT.size])
                    with self._lock:
                        self._omega = np.array([ox, oy, oz])
                        self._attitude = np.array([roll, pitch, yaw])
                        self._t_mono = t_mono
            except socket.timeout:
                continue
            except Exception:
                continue
        sock.close()

    def get_omega(self, max_age_s: float = 0.5):
        """Returns the latest angular velocity (3,) or None if stale/unavailable."""
        with self._lock:
            if self._omega is None:
                return None
            age = time.monotonic() - self._t_mono
            if age > max_age_s:
                return None
            return self._omega.copy()

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
        P = np.array([
            [0.0,              1.0, 0.0],   # X_B0 =  Y_R   (forward)
            [self.lateral_sign, 0.0, 0.0],  # Y_B0 = ±X_R   (lateral/right)
            [0.0,              0.0, -1.0],  # Z_B0 = -Z_R   (down = -elevation)
        ])
        # R_R^B(theta_tilt) = R_tilt @ P: apply the axis permutation first,
        # then the mechanical pitch-down tilt, exactly as Eq.(2) expects.
        self.R = R_tilt @ P

    def to_body(self, points_radar_xyz: np.ndarray) -> np.ndarray:
        # Eq. (2): P^B = R_R^B P^R + t^B_R  (vectorized over rows)
        return points_radar_xyz @ self.R.T + self.lever_arm


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
                    cond_reject_threshold: float = 30.0, rng=None):
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

    # Low-count guard: with fewer than 8 points, 3-point minimal sampling
    # overfits sensor noise (solving 3 unknowns from 3-7 points has little
    # or no statistical redundancy). Only evaluate the static hypothesis (v=0).
    # If static is supported, accept it; otherwise reject as a gap (valid=False).
    # Never solve a moving triad on < 8 points.
    if n < 8:
        if (score_zero / n >= min_inlier_ratio) or score_zero >= 3:
            return mask_zero, True, 1.0
        return None

    # Dominant-static guard: by geometry, a moving vehicle (speed > eps) can have
    # at most ~20% of its returns in the zero-Doppler band (|u^T v| < eps).
    # If >= 70% of returns are already consistent with zero Doppler, the platform
    # is physically static -- any moving hypothesis that fits residual noise
    # is an artifact of FMCW Doppler quantization/clutter.
    if n > 0 and (score_zero / n >= 0.70) and (score_zero >= 3):
        return mask_zero, True, 1.0

    best_mask, best_score = None, -1
    for _ in range(iters):
        idx = rng.choice(n, size=3, replace=False)
        A_sub, b_sub = -u_body[idx], v_radial[idx]
        # Degenerate-triad guard: skip minimal samples whose 3 LOS vectors
        # are nearly coplanar/parallel -- solving through them is
        # numerically unstable and is exactly what produces spurious
        # multi-m/s "hypotheses" on a near-planar floor patch.
        sing = np.linalg.svd(A_sub, compute_uv=False)
        if sing[-1] < 1e-3 or (sing[0] / max(sing[-1], 1e-9)) > cond_reject_threshold:
            continue
        try:
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
    static_ok = (n > 0) and (score_zero / n >= min_inlier_ratio)

    if best_mask is None or best_score <= score_zero + margin:
        # Nothing beat static outright, or didn't beat it by enough margin
        # to trust the more complex (3-parameter) explanation over it.
        if static_ok:
            return mask_zero, True, 1.0  # static accepted, cond. number moot
        return None

    if best_score / n < min_inlier_ratio:
        return None

    cond = float(np.linalg.cond(-u_body[best_mask]))
    if cond > cond_reject_threshold:
        return None

    return best_mask, False, cond


def weighted_refit(u_body, v_radial, ranges, mask, max_speed_mps: float = 25.0):
    """Eq. (9): v = (A^T W A)^-1 A^T W b using weighted least-squares with SVD conditioning."""
    A = -u_body[mask]
    b = v_radial[mask]
    w = 1.0 / np.clip(ranges[mask], 0.5, None) ** 2
    sqrt_w = np.sqrt(w)
    A_w = A * sqrt_w[:, None]
    b_w = b * sqrt_w
    try:
        v_body, _, _, _ = np.linalg.lstsq(A_w, b_w, rcond=1e-3)
        ATA = A_w.T @ A_w
        cov = np.linalg.pinv(ATA, rcond=1e-3)
    except (np.linalg.LinAlgError, ValueError):
        return None, None

    if np.linalg.norm(v_body) > max_speed_mps:
        return None, None
    return v_body, cov


class DopplerRIO:
    def __init__(self, mount: TiltMount, min_range=0.3, max_range=350.0,
                 eps=0.15, iters=60, min_inlier_ratio=0.35,
                 static_margin_frac=0.12, static_margin_min=3,
                 cond_reject_threshold=30.0, deadband_mps=0.05,
                 imu_listener: IMUListener | None = None):
        self.mount = mount
        self.min_range, self.max_range = min_range, max_range
        self.eps, self.iters, self.min_inlier_ratio = eps, iters, min_inlier_ratio
        self.static_margin_frac = static_margin_frac
        self.static_margin_min = static_margin_min
        self.cond_reject_threshold = cond_reject_threshold
        self.deadband_mps = deadband_mps
        self.imu_listener = imu_listener

    def process_frame(self, points_radar: np.ndarray, t_frame: float):
        """points_radar: (N,4) [x,y,z,v] in RADAR frame. Returns a result dict."""
        if points_radar.shape[0] < 3:
            return {'t': t_frame, 'valid': False, 'n_total': int(points_radar.shape[0])}

        xyz_b = self.mount.to_body(points_radar[:, 0:3])
        v_meas = points_radar[:, 3]
        ranges = np.linalg.norm(xyz_b, axis=1)

        keep = (ranges > self.min_range) & (ranges < self.max_range)
        if keep.sum() < 3:
            return {'t': t_frame, 'valid': False, 'n_total': int(keep.sum())}
        xyz_b, v_meas, ranges = xyz_b[keep], v_meas[keep], ranges[keep]
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
        if omega is not None:
            omega_cross_p = np.cross(omega, xyz_b)          # (N, 3)
            v_rot_comp = np.sum(u_body * omega_cross_p, axis=1)  # (N,)
            v_adjusted = v_meas + v_rot_comp
        else:
            v_adjusted = v_meas

        ransac_result = doppler_ransac(
            u_body, v_adjusted, self.eps, self.iters, self.min_inlier_ratio,
            static_margin_frac=self.static_margin_frac,
            static_margin_min=self.static_margin_min,
            cond_reject_threshold=self.cond_reject_threshold)
        if ransac_result is None:
            # Sec. 1.3/3.4: expected, recoverable gap -- not a fault. Caller
            # (the EKF) should widen covariance / coast, not disarm.
            return {'t': t_frame, 'valid': False, 'n_total': int(keep.sum())}
        mask, is_static, cond = ransac_result

        if is_static:
            v_body = np.zeros(3)
            # Tight covariance: a validated static hypothesis (cleared the
            # min_inlier_ratio gate on its own, no ambiguous moving
            # hypothesis beat it by the required margin) is high confidence.
            cov = np.eye(3) * 1e-4
        else:
            v_body, cov = weighted_refit(u_body, v_adjusted, ranges, mask)
            if v_body is None:
                return {'t': t_frame, 'valid': False, 'n_total': int(keep.sum())}

        # Deadband: snap near-zero solves to exactly zero. Below this speed
        # you're inside the sensor/estimator noise floor, not measuring real
        # motion -- publishing e.g. 0.03 m/s as "motion" just adds noise to
        # anything integrating this downstream (dead-reckoning, SLAM deskew).
        if np.linalg.norm(v_body) < self.deadband_mps:
            v_body = np.zeros(3)

        return {
            't': t_frame, 'valid': True,
            'v_body': v_body, 'cov_v': cov,
            'n_inliers': int(mask.sum()), 'n_total': int(keep.sum()),
            'is_static': is_static, 'cond': cond,
            'omega': omega,
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

    rio = DopplerRIO(mount, max_range=args.max_range, eps=args.eps,
                      min_inlier_ratio=args.min_inlier_ratio,
                      static_margin_frac=args.static_margin_frac,
                      static_margin_min=args.static_margin_min,
                      cond_reject_threshold=args.cond_reject_threshold,
                      deadband_mps=args.deadband,
                      imu_listener=imu_listener)

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
        t_frame = time.time()
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
                pkt = FORWARD_PKT.pack(t_frame, vx, vy, vz, result['n_inliers'])
                for dest_ip, dest_port in forward_dests:
                    out_sock.sendto(pkt, (dest_ip, dest_port))
        else:
            print(f"t={t_frame:.3f}  RIO frame rejected (n={result['n_total']}) "
                  f"-- gap, not a fault; downstream EKF should widen covariance")


def self_test():
    """Synthetic validation: known ego-velocity + a ground-scan-shaped point
    cloud (forward-and-below, per Sec. 1.3/1.4) + injected outliers (movers /
    multipath) -> recovered velocity must match ground truth."""
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
    xyz_radar_static = (xyz_body_static - mount.lever_arm) @ mount.R  # inverse of Eq.(2)
    u_body_static = xyz_body_static / np.linalg.norm(xyz_body_static, axis=1, keepdims=True)
    v_radial_static = -(u_body_static @ v_true) + rng.normal(0, 0.03, n_static)

    az_o = rng.uniform(-0.6, 0.6, n_outliers)
    el_o = rng.uniform(math.radians(20), math.radians(60), n_outliers)
    r_o = rng.uniform(30, 300, n_outliers)
    xyz_body_out = np.stack([
        r_o * np.cos(el_o) * np.cos(az_o), r_o * np.cos(el_o) * np.sin(az_o), r_o * np.sin(el_o)
    ], axis=1)
    xyz_radar_out = (xyz_body_out - mount.lever_arm) @ mount.R
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
    p.add_argument('--theta-tilt-deg', type=float, default=40.0)
    p.add_argument('--lateral-sign', type=float, default=1.0, choices=[1.0, -1.0],
                    help="U300 native X-axis polarity; flip to -1.0 if the bench "
                         "left/right validation (see TiltMount docstring) shows it inverted")
    p.add_argument('--lever-x', type=float, default=0.12)
    p.add_argument('--lever-y', type=float, default=0.0)
    p.add_argument('--lever-z', type=float, default=0.05)
    p.add_argument('--max-range', type=float, default=350.0)
    p.add_argument('--eps', type=float, default=0.20,
                    help="m/s tolerance for Doppler consistency (matches filters.py doppler_eps_mps)")
    p.add_argument('--min-inlier-ratio', type=float, default=0.25,
                    help="minimum fraction of returns that must fit hypothesis")
    p.add_argument('--static-margin-frac', type=float, default=0.12,
                    help="a moving hypothesis must beat the static (v=0) hypothesis "
                         "by at least this fraction of points, on top of --static-margin-min")
    p.add_argument('--static-margin-min', type=int, default=3)
    p.add_argument('--cond-reject-threshold', type=float, default=12.0,
                    help="reject a moving-hypothesis frame if its inlier LOS matrix "
                         "condition number exceeds this (near-planar/degenerate geometry)")
    p.add_argument('--deadband', type=float, default=0.05,
                    help="m/s; snap |v_body| below this to exactly zero")
    p.add_argument('--imu-port', type=int, default=0,
                    help="UDP port to receive IMU data from imu_bridge.py "
                         "(default 0 = disabled, no rotation compensation)")
    args = p.parse_args()

    self_test() if args.selftest else run_udp_loop(args)
