#!/usr/bin/env python3
"""
rio_eval.py -- verified evaluation core for the RIO/SLAM radar stack.

This module holds every number-producing function used by analyze_run_v2.py.
It is deliberately separate from the report printer so that each function can
be unit-tested against synthetic data with a known answer (see selftest.py /
`analyze_run_v2.py --selftest`).

=============================================================================
FRAME CONVENTIONS  (fixed, single source of truth -- everything else follows)
=============================================================================
BODY   : FRD.  x = forward (nose), y = right, z = down.
         doppler_rio.py publishes v_body in this frame.
NED    : n = north, e = east, d = down.
         R_nb = Rz(yaw) @ Ry(pitch) @ Rx(roll)   (ZYX intrinsic)
         v_ned = R_nb @ v_body
ENU    : e = east, n = north, u = up.   (gps_logger.py writes GPS in ENU)
         v_enu = [v_ned[1], v_ned[0], -v_ned[2]]
SLAM   : slam_node.py's T_world is a *map* frame whose axes are
         x = forward-at-bootstrap, y = right-at-bootstrap, z = down,
         UNLESS --trust-imu-yaw was on and an absolute yaw was available at
         bootstrap, in which case x = north, y = east, z = down.
         The analyzer NEVER assumes which one it is: it estimates the map
         yaw offset from the data and reports it (see slam_alignment()).

Every angle in this file is radians unless the name ends in `_deg`.
=============================================================================
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

import numpy as np

# WGS-84
_WGS84_A = 6378137.0
_WGS84_E2 = 6.69437999014e-3


# ═════════════════════════════════════════════════════════════════════════
#  Rotations
# ═════════════════════════════════════════════════════════════════════════

def R_body_to_ned(roll, pitch, yaw):
    """ZYX intrinsic DCM. v_ned = R @ v_body, body = FRD, ned = NED."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def R_body_to_ned_batch(rpy):
    """Vectorised R_body_to_ned for an (N,3) array of [roll,pitch,yaw]."""
    r, p, y = rpy[:, 0], rpy[:, 1], rpy[:, 2]
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    R = np.empty((len(rpy), 3, 3))
    R[:, 0, 0] = cy * cp
    R[:, 0, 1] = cy * sp * sr - sy * cr
    R[:, 0, 2] = cy * sp * cr + sy * sr
    R[:, 1, 0] = sy * cp
    R[:, 1, 1] = sy * sp * sr + cy * cr
    R[:, 1, 2] = sy * sp * cr - cy * sr
    R[:, 2, 0] = -sp
    R[:, 2, 1] = cp * sr
    R[:, 2, 2] = cp * cr
    return R


NED_TO_ENU = np.array([[0.0, 1.0, 0.0],
                       [1.0, 0.0, 0.0],
                       [0.0, 0.0, -1.0]])


def ned_to_enu(v):
    """(...,3) NED -> ENU. Involutive: ned_to_enu(ned_to_enu(x)) == x."""
    return np.asarray(v) @ NED_TO_ENU.T


def euler_from_R(R):
    """ZYX intrinsic Euler angles (roll, pitch, yaw) from a DCM."""
    pitch = -math.asin(float(np.clip(R[2, 0], -1.0, 1.0)))
    if abs(R[2, 0]) < 0.9999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:                                   # gimbal lock
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


# ═════════════════════════════════════════════════════════════════════════
#  Geodesy -- proper WGS-84 local tangent plane (replaces the flat-earth
#  constants hard-coded in gps_logger.py, which are only valid near 30 N)
# ═════════════════════════════════════════════════════════════════════════

def enu_scale_factors(lat0_deg):
    """Metres per degree of latitude / longitude at lat0 on WGS-84."""
    lat0 = math.radians(lat0_deg)
    s = math.sin(lat0)
    denom = math.sqrt(1.0 - _WGS84_E2 * s * s)
    M = _WGS84_A * (1.0 - _WGS84_E2) / denom ** 3      # meridional radius
    N = _WGS84_A / denom                                # prime-vertical radius
    return M * math.pi / 180.0, N * math.cos(lat0) * math.pi / 180.0


def lla_to_enu(lat, lon, alt, lat0, lon0, alt0):
    m_per_deg_lat, m_per_deg_lon = enu_scale_factors(lat0)
    return ((lon - lon0) * m_per_deg_lon,
            (lat - lat0) * m_per_deg_lat,
            alt - alt0)


# ═════════════════════════════════════════════════════════════════════════
#  Log loading
# ═════════════════════════════════════════════════════════════════════════

@dataclass
class Run:
    """A loaded flight log, already converted into arrays."""
    path: str = ""
    meta: dict = field(default_factory=dict)

    # GPS / FC EKF position (ENU, metres, relative to first fix)
    gps_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    gps_enu: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    gps_sats: np.ndarray = field(default_factory=lambda: np.zeros(0))
    gps_v_enu: np.ndarray | None = None        # only if the log carries it

    # RIO (body FRD)
    rio_t: np.ndarray = field(default_factory=lambda: np.zeros(0))     # t_frame
    rio_t_recv: np.ndarray = field(default_factory=lambda: np.zeros(0))
    rio_v: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    rio_inliers: np.ndarray = field(default_factory=lambda: np.zeros(0))
    rio_ntotal: np.ndarray = field(default_factory=lambda: np.zeros(0))
    rio_cond: np.ndarray = field(default_factory=lambda: np.zeros(0))
    rio_static: np.ndarray = field(default_factory=lambda: np.zeros(0, bool))
    rio_airborne: np.ndarray = field(default_factory=lambda: np.zeros(0, bool))
    rio_vzprior: np.ndarray = field(default_factory=lambda: np.zeros(0, bool))

    # SLAM (map frame, z-down)
    slam_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    slam_t_recv: np.ndarray = field(default_factory=lambda: np.zeros(0))
    slam_pos: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    slam_nmap: np.ndarray = field(default_factory=lambda: np.zeros(0))
    slam_fwd: np.ndarray = field(default_factory=lambda: np.zeros(0))

    # IMU
    imu_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    imu_rpy: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    imu_omega: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))

    # Altimeter (raw slant range along body -z)
    alt_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    alt_r: np.ndarray = field(default_factory=lambda: np.zeros(0))

    # Optional v2 fields (present only if the patched gps_logger wrote them)
    rio_sigma: np.ndarray | None = None        # (N,3) published sigma, m/s
    rio_rej_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    rio_rej_reason: list = field(default_factory=list)
    rio_rej_ntotal: np.ndarray = field(default_factory=lambda: np.zeros(0))
    slam_fitness: np.ndarray | None = None
    slam_rmse: np.ndarray | None = None
    slam_ncorr: np.ndarray | None = None
    slam_nobs: np.ndarray | None = None
    slam_rej_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    slam_rej_reason: list = field(default_factory=list)

    warnings: list = field(default_factory=list)

    @property
    def t0(self):
        cands = [a[0] for a in (self.gps_t, self.rio_t, self.slam_t,
                                self.imu_t, self.alt_t) if len(a)]
        return min(cands) if cands else 0.0

    @property
    def duration(self):
        cands = [a[-1] for a in (self.gps_t, self.rio_t, self.slam_t,
                                 self.imu_t, self.alt_t) if len(a)]
        return (max(cands) - self.t0) if cands else 0.0


def _col(rows, *keys, default=np.nan):
    out = []
    for r in rows:
        for k in keys:
            if k in r:
                out.append(r[k])
                break
        else:
            out.append(default)
    return np.asarray(out, dtype=float)


def load_run(path: str) -> Run:
    """Parse a gps_logger.py JSONL file into a Run. Tolerant of partial lines."""
    buckets = {'gps': [], 'rio': [], 'slam': [], 'imu': [], 'altimeter': [],
               'rio_reject': [], 'slam_reject': []}
    meta, bad = {}, 0
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                e = json.loads(line)
            except json.JSONDecodeError:
                bad += 1
                continue
            t = e.get('type')
            if t in buckets:
                buckets[t].append(e)
            elif t == 'meta':
                meta = e

    r = Run(path=path, meta=meta)
    if bad:
        r.warnings.append(f"{bad} malformed JSON line(s) skipped "
                          f"(log was probably truncated mid-write)")

    g = buckets['gps']
    if g:
        r.gps_t = _col(g, 't_mono')
        r.gps_enu = np.array([e['enu'] for e in g], dtype=float)
        r.gps_sats = _col(g, 'sats', default=0.0)
        if all('v_enu' in e for e in g):
            r.gps_v_enu = np.array([e['v_enu'] for e in g], dtype=float)

    q = buckets['rio']
    if q:
        r.rio_t_recv = _col(q, 't_mono')
        r.rio_t = _col(q, 't_frame', 't_mono')
        r.rio_v = np.column_stack([_col(q, 'vx'), _col(q, 'vy'), _col(q, 'vz')])
        r.rio_inliers = _col(q, 'inliers', default=0.0)
        r.rio_ntotal = _col(q, 'n_total', default=0.0)
        r.rio_cond = _col(q, 'cond', default=0.0)
        r.rio_static = np.array([bool(e.get('is_static', False)) for e in q])
        r.rio_airborne = np.array([bool(e.get('airborne', False)) for e in q])
        r.rio_vzprior = np.array([bool(e.get('vz_prior', False)) for e in q])

    s = buckets['slam']
    if s:
        r.slam_t_recv = _col(s, 't_mono')
        r.slam_t = _col(s, 't_slam', 't_mono')
        r.slam_pos = np.array([e['pos'] for e in s], dtype=float)
        r.slam_nmap = _col(s, 'n_map', default=0.0)
        r.slam_fwd = _col(s, 'fwd_range', default=np.inf)
        # slam_node encodes "nothing in the forward cone" as 1e6, not inf.
        r.slam_fwd = np.where(r.slam_fwd >= 9.9e5, np.inf, r.slam_fwd)

    m = buckets['imu']
    if m:
        r.imu_t = _col(m, 't_mono')
        r.imu_rpy = np.column_stack([_col(m, 'roll'), _col(m, 'pitch'), _col(m, 'yaw')])
        r.imu_omega = np.column_stack([_col(m, 'wx'), _col(m, 'wy'), _col(m, 'wz')])

    if q and all(('sx' in e) for e in q):
        r.rio_sigma = np.column_stack([_col(q, 'sx'), _col(q, 'sy'), _col(q, 'sz')])

    if s and all(('fitness' in e) for e in s):
        r.slam_fitness = _col(s, 'fitness')
        r.slam_rmse = _col(s, 'rmse')
        r.slam_ncorr = _col(s, 'n_corr')
        r.slam_nobs = _col(s, 'n_obs_axes')

    rj = buckets['rio_reject']
    if rj:
        r.rio_rej_t = _col(rj, 't_frame', 't_mono')
        r.rio_rej_reason = [e.get('reason', 'unknown') for e in rj]
        r.rio_rej_ntotal = _col(rj, 'n_total', default=0.0)

    sj = buckets['slam_reject']
    if sj:
        r.slam_rej_t = _col(sj, 't_slam', 't_mono')
        r.slam_rej_reason = [e.get('reason', 'unknown') for e in sj]

    a = buckets['altimeter']
    if a:
        r.alt_t = _col(a, 't_mono')
        r.alt_r = _col(a, 'range_m')

    for name, t in (('gps', r.gps_t), ('rio', r.rio_t), ('slam', r.slam_t),
                    ('imu', r.imu_t), ('alt', r.alt_t)):
        if len(t) > 1 and np.any(np.diff(t) < 0):
            r.warnings.append(f"{name} timestamps are not monotonic "
                              f"({int((np.diff(t) < 0).sum())} backward steps) -- "
                              f"sorting before analysis")
    return r


# ═════════════════════════════════════════════════════════════════════════
#  Interpolation helpers
# ═════════════════════════════════════════════════════════════════════════

def interp_vec(t_src, y_src, t_query):
    """Linear interpolation of an (N,K) series onto t_query. Clamped at ends."""
    t_src = np.asarray(t_src, float)
    y_src = np.asarray(y_src, float)
    if y_src.ndim == 1:
        return np.interp(t_query, t_src, y_src)
    return np.column_stack([np.interp(t_query, t_src, y_src[:, k])
                            for k in range(y_src.shape[1])])


def nearest_rpy(imu_t, imu_rpy, t_query, max_dt=0.2):
    """Nearest-in-time attitude, with unwrapped-yaw linear interpolation.

    Yaw wraps at +/-pi; naive np.interp across a wrap produces a ~2*pi error
    on a single sample, which rotates that sample's velocity by 360 deg worth
    of intermediate garbage. We unwrap first, interpolate, then re-wrap.
    """
    if len(imu_t) == 0:
        return None, None
    rpy = imu_rpy.copy()
    rpy[:, 2] = np.unwrap(rpy[:, 2])
    out = interp_vec(imu_t, rpy, t_query)
    out[:, 2] = np.arctan2(np.sin(out[:, 2]), np.cos(out[:, 2]))
    idx = np.clip(np.searchsorted(imu_t, t_query), 1, len(imu_t) - 1)
    dt = np.minimum(np.abs(imu_t[idx] - t_query), np.abs(imu_t[idx - 1] - t_query))
    return out, dt <= max_dt


# ═════════════════════════════════════════════════════════════════════════
#  GPS ground truth
# ═════════════════════════════════════════════════════════════════════════

def gps_velocity(gps_t, gps_enu, window_s=0.6, v_native=None):
    """Ground-truth ENU velocity by centred finite difference over `window_s`.

    METHOD (this is the answer to "how is GPS velocity calculated?"):
      For each sample i, find the samples j0 and j1 nearest to t_i-w/2 and
      t_i+w/2 and take (p[j1]-p[j0]) / (t[j1]-t[j0]).
      * Centred, so there is NO group delay -- a one-sided difference would
        bias the velocity by w/2 in time (0.3 s at 1 m/s^2 = 0.3 m/s error).
      * A finite window low-passes the position noise. Differentiating raw
        50 Hz EKF position gives sigma_v ~ sigma_p*sqrt(2)/dt = huge.
        With w = 0.6 s and sigma_p ~ 0.05 m, sigma_v ~ 0.12 m/s.
      * The cost is bandwidth: motion faster than ~1/w Hz is attenuated.
        Do not shrink w below ~0.3 s or the "ground truth" becomes noise.

    Returns (v_enu (N,3), valid mask).

    If `v_native` is supplied (GLOBAL_POSITION_INT.vx/vy/vz, logged by the
    patched gps_logger.py), it is returned unchanged instead: the autopilot's
    own EKF velocity is lower-noise and full-bandwidth, so differentiating
    position is strictly worse whenever the native field exists.
    """
    n = len(gps_t)
    if v_native is not None and len(v_native) == n:
        ok = np.isfinite(np.asarray(v_native, float)).all(1)
        return np.asarray(v_native, float), ok
    v = np.full((n, 3), np.nan)
    if n < 2:
        return v, np.zeros(n, bool)
    half = window_s / 2.0
    j0 = np.searchsorted(gps_t, gps_t - half, side='left')
    j1 = np.searchsorted(gps_t, gps_t + half, side='right') - 1
    j0 = np.clip(j0, 0, n - 1)
    j1 = np.clip(j1, 0, n - 1)
    dt = gps_t[j1] - gps_t[j0]
    ok = dt > 1e-6
    v[ok] = (gps_enu[j1][ok] - gps_enu[j0][ok]) / dt[ok, None]
    return v, ok


def path_length_multi(gps_t, gps_enu, steps=(0.0, 0.2, 1.0, 2.0)):
    """Path length at several time-decimations.

    WHY MORE THAN ONE NUMBER: path length of a noisy track is not a
    well-defined quantity (the coastline paradox). Decimating to 1 Hz --
    which both the old analyze_run.py and plot_run.py do -- silently chooses
    one answer and hides the sensitivity. Reporting the family lets you see
    immediately whether your "GPS path length" is signal or noise: if the
    0.0 s and 2.0 s numbers differ by <5%, the track is clean; if the raw
    number is 2x the 2 s number, most of your "distance" is jitter.
    """
    out = {}
    for s in steps:
        if s <= 0:
            idx = np.arange(len(gps_t))
        else:
            idx = [0]
            for i in range(1, len(gps_t)):
                if gps_t[i] - gps_t[idx[-1]] >= s:
                    idx.append(i)
            idx = np.asarray(idx)
        if len(idx) < 2:
            out[s] = (0.0, 0.0)
            continue
        d = np.diff(gps_enu[idx], axis=0)
        out[s] = (float(np.linalg.norm(d, axis=1).sum()),
                  float(np.linalg.norm(d[:, :2], axis=1).sum()))
    return out


def speed_integrated_path(gps_t, gps_enu, intervals=None, window_s=0.6,
                          deadband_mps=0.0):
    """Path length as the time-integral of |v|, optionally restricted to
    `intervals` [(t0,t1), ...]. This is the decimation-free, physically
    meaningful definition and it is the one used for RIO comparison, because
    RIO's own distance is also an integral of speed.

    `deadband_mps` zeroes speeds below the threshold before integrating --
    use it to strip the GPS/EKF noise floor when you want "distance actually
    travelled" rather than "arc length of the noisy track".
    """
    v, ok = gps_velocity(gps_t, gps_enu, window_s)
    sp = np.linalg.norm(v, axis=1)
    sp2 = np.linalg.norm(v[:, :2], axis=1)
    sp = np.where(ok, sp, 0.0)
    sp2 = np.where(ok, sp2, 0.0)
    if deadband_mps > 0:
        sp = np.where(sp > deadband_mps, sp, 0.0)
        sp2 = np.where(sp2 > deadband_mps, sp2, 0.0)
    w = _interval_weights(gps_t, intervals)
    return float(np.trapz(sp * w, gps_t)), float(np.trapz(sp2 * w, gps_t))


def _interval_weights(t, intervals):
    """1.0 inside any interval, 0.0 outside. None -> all ones."""
    if intervals is None:
        return np.ones_like(t)
    w = np.zeros_like(t)
    for a, b in intervals:
        w[(t >= a) & (t <= b)] = 1.0
    return w


def gps_displacement_over(gps_t, gps_enu, intervals=None):
    """Vector displacement summed over the given intervals (ENU, metres).
    With intervals=None this is simply last-minus-first."""
    if len(gps_t) < 2:
        return np.zeros(3)
    if intervals is None:
        return gps_enu[-1] - gps_enu[0]
    total = np.zeros(3)
    for a, b in intervals:
        pa = interp_vec(gps_t, gps_enu, np.array([a]))[0]
        pb = interp_vec(gps_t, gps_enu, np.array([b]))[0]
        total += pb - pa
    return total


# ═════════════════════════════════════════════════════════════════════════
#  RIO coverage + integration
# ═════════════════════════════════════════════════════════════════════════

def coverage_intervals(t, max_gap_s=0.5):
    """Split a sample-time vector into contiguous intervals, breaking wherever
    consecutive samples are more than `max_gap_s` apart.

    THIS IS THE SINGLE MOST IMPORTANT FIX versus the old analyzer.
    doppler_rio.py only transmits frames it accepts; rejected frames leave no
    trace in the log at all. Integrating only over dt <= max_gap (which the
    old analyze_run.py did) silently deletes those seconds from RIO's
    distance -- but the old analyzer then compared that partial integral
    against the FULL-flight GPS path. On the 2026-09-20 logs that alone
    manufactures a 50-70% "distance error" out of thin air.
    Every RIO-vs-GPS comparison below is restricted to these intervals.
    """
    if len(t) < 2:
        return [], 0.0
    out, start = [], t[0]
    for i in range(1, len(t)):
        if t[i] - t[i - 1] > max_gap_s:
            if t[i - 1] > start:
                out.append((start, t[i - 1]))
            start = t[i]
    if t[-1] > start:
        out.append((start, t[-1]))
    return out, float(sum(b - a for a, b in out))


def rio_to_enu(rio_t, rio_v_body, imu_t, imu_rpy, max_dt=0.2):
    """Rotate RIO body-frame velocity into ENU using the FC attitude.

    Returns (v_enu (N,3), v_ned (N,3), attitude_valid mask).
    Samples with no fresh attitude are flagged, NOT silently passed through
    as if body==world (which is what analyze_run.py and plot_run.py both do,
    and which turns a yaw of 90 deg into a 100% displacement error).
    """
    n = len(rio_t)
    if n == 0:
        return np.zeros((0, 3)), np.zeros((0, 3)), np.zeros(0, bool)
    if len(imu_t) == 0:
        return (np.full((n, 3), np.nan), np.full((n, 3), np.nan),
                np.zeros(n, bool))
    rpy, fresh = nearest_rpy(imu_t, imu_rpy, rio_t, max_dt=max_dt)
    R = R_body_to_ned_batch(rpy)
    v_ned = np.einsum('nij,nj->ni', R, rio_v_body)
    return ned_to_enu(v_ned), v_ned, fresh


def integrate_velocity(t, v, intervals):
    """Trapezoidal integral of v over the given intervals.

    Returns (displacement (3,), path_length_3d, path_length_2d).
    Only consecutive sample pairs that lie wholly inside one interval
    contribute -- no coasting across a dropout, ever.
    """
    disp = np.zeros(3)
    L3 = L2 = 0.0
    if len(t) < 2:
        return disp, 0.0, 0.0
    iv = np.asarray(intervals, float).reshape(-1, 2) if len(intervals) else np.zeros((0, 2))
    for i in range(1, len(t)):
        a, b = t[i - 1], t[i]
        dt = b - a
        if dt <= 0:
            continue
        if len(iv) and not np.any((iv[:, 0] <= a + 1e-9) & (b <= iv[:, 1] + 1e-9)):
            continue
        v_avg = 0.5 * (v[i - 1] + v[i])
        if not np.all(np.isfinite(v_avg)):
            continue
        disp += v_avg * dt
        L3 += float(np.linalg.norm(v_avg)) * dt
        L2 += float(np.linalg.norm(v_avg[:2])) * dt
    return disp, L3, L2


# ═════════════════════════════════════════════════════════════════════════
#  Velocity agreement metrics
# ═════════════════════════════════════════════════════════════════════════

def axis_stats(a, b, label=""):
    """Compare estimate `a` against truth `b` on one axis."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    m = np.isfinite(a) & np.isfinite(b)
    a, b = a[m], b[m]
    if len(a) < 3:
        return {'label': label, 'n': len(a)}
    err = a - b
    denom = float(b @ b)
    slope = float(a @ b) / denom if denom > 1e-12 else np.nan
    sa, sb = a.std(), b.std()
    corr = float(np.corrcoef(a, b)[0, 1]) if sa > 1e-9 and sb > 1e-9 else np.nan
    return {
        'label': label, 'n': int(len(a)),
        'bias': float(err.mean()),
        'rmse': float(np.sqrt((err ** 2).mean())),
        'mae': float(np.abs(err).mean()),
        'p95': float(np.percentile(np.abs(err), 95)),
        'corr': corr,
        'slope': slope,                      # a ~= slope * b ; 1.0 is perfect
        'std_est': float(sa), 'std_truth': float(sb),
    }


def estimate_time_lag(t_a, s_a, t_b, s_b, max_lag_s=1.5, step_s=0.02):
    """Cross-correlate two speed series to find the lag that best aligns them.

    Positive result => series A (RIO) lags series B (GPS) by that many
    seconds, i.e. the radar pipeline delay you must enter as
    EKF2_EV_DELAY / EK3_VIS_DELAY. A wrong delay is invisible in any
    per-frame metric but shows up directly as position error.
    """
    if len(t_a) < 10 or len(t_b) < 10:
        return None
    lags = np.arange(-max_lag_s, max_lag_s + 1e-9, step_s)
    best = (0.0, -2.0)
    for lag in lags:
        sb = np.interp(t_a - lag, t_b, s_b, left=np.nan, right=np.nan)
        m = np.isfinite(sb) & np.isfinite(s_a)
        if m.sum() < 10:
            continue
        x, y = s_a[m], sb[m]
        if x.std() < 1e-9 or y.std() < 1e-9:
            continue
        c = float(np.corrcoef(x, y)[0, 1])
        if c > best[1]:
            best = (float(lag), c)
    return best


def estimate_misalignment(v_body_est, v_body_truth, min_speed=1.0):
    """Solve for the rotation M and scale s minimising || s*M*v_truth - v_est ||.

    WHY THIS MATTERS: this is a direct, data-driven measurement of your mount
    calibration. If --theta-tilt-deg is wrong by d degrees, M comes back as a
    pitch of ~d. If --lateral-sign is inverted, M comes back as a ~180 deg
    roll (or a reflection, which is reported as such). If the scale s is not
    ~1.0, the Doppler scaling or the range gate is off. None of this is
    observable from a speed-magnitude comparison, which is all the old
    analyzer did -- a stack with vx and vy swapped scores a perfect speed
    error of 0.000 m/s.
    """
    A = np.asarray(v_body_est, float)
    B = np.asarray(v_body_truth, float)
    m = np.isfinite(A).all(1) & np.isfinite(B).all(1)
    m &= np.linalg.norm(B, axis=1) >= min_speed
    A, B = A[m], B[m]
    if len(A) < 20:
        return None
    H = B.T @ A                                  # maps B -> A
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    M = Vt.T @ D @ U.T
    rotated = B @ M.T
    denom = float((rotated * rotated).sum())
    scale = float((rotated * A).sum()) / denom if denom > 1e-12 else np.nan
    resid = A - scale * rotated
    roll, pitch, yaw = euler_from_R(M)
    return {
        'n': int(len(A)),
        'reflection': bool(d < 0),
        'roll_deg': math.degrees(roll),
        'pitch_deg': math.degrees(pitch),
        'yaw_deg': math.degrees(yaw),
        'scale': scale,
        'resid_rms': float(np.sqrt((resid ** 2).sum(1).mean())),
        'M': M,
    }


# ═════════════════════════════════════════════════════════════════════════
#  SLAM alignment + error
# ═════════════════════════════════════════════════════════════════════════

def _kabsch_2d(P, Q, allow_reflection=False):
    """Rotation R and translation t with R@P_i + t ~= Q_i (both (N,2))."""
    cP, cQ = P.mean(0), Q.mean(0)
    H = (P - cP).T @ (Q - cQ)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    refl = False
    if np.linalg.det(R) < 0:
        refl = True
        if not allow_reflection:
            Vt2 = Vt.copy()
            Vt2[1, :] *= -1
            R = Vt2.T @ U.T
    return R, cQ - R @ cP, refl


def slam_alignment(run: Run, alt_ref=False, lag_s=0.0):
    """Compare the SLAM pose chain against GPS at three levels of forgiveness.

    Level 1  RAW            : SLAM map axes assumed to be (x=N, y=E, z=D) and
                              origin-shifted to the first GPS point. This is
                              the only level that tests absolute navigation,
                              and it is ONLY meaningful if slam_node was run
                              with --trust-imu-yaw and had a valid absolute
                              yaw at bootstrap. The old analyzer reported
                              this number unconditionally and called it
                              "Absolute XY error" -- with the shipped default
                              (--no-trust-imu-yaw) it is dominated by an
                              arbitrary map-yaw offset and is meaningless.
    Level 2  YAW-ALIGNED    : one rotation + one translation, both constant,
                              fitted once over the whole run. This removes the
                              unknown map heading but keeps all drift. THIS IS
                              THE NUMBER TO JUDGE THE SLAM FRONT-END BY.
    Level 3  SHAPE (Kabsch) : Level 2 plus a best-fit centroid translation --
                              i.e. pure trajectory shape. Useful to separate
                              "wrong heading" from "wrong shape".

    Z is handled separately and is never rotated: SLAM z is DOWN, GPS u is UP.
    """
    if len(run.slam_t) < 2 or len(run.gps_t) < 2:
        return None

    t = run.slam_t + lag_s
    m = (t >= run.gps_t[0]) & (t <= run.gps_t[-1])
    if m.sum() < 2:
        return None
    t = t[m]
    sp = run.slam_pos[m]
    gp = interp_vec(run.gps_t, run.gps_enu, t)

    # SLAM map XY (x,y) treated as (north, east) -> ENU (east, north)
    slam_en = np.column_stack([sp[:, 1], sp[:, 0]])
    gps_en = gp[:, :2]

    raw_en = slam_en - slam_en[0] + gps_en[0]
    err_raw = np.linalg.norm(raw_en - gps_en, axis=1)

    # Level 2: rotation about the first point only (no centroid freedom)
    P = slam_en - slam_en[0]
    Q = gps_en - gps_en[0]
    H = P.T @ Q
    U, S, Vt = np.linalg.svd(H)
    R2 = Vt.T @ U.T
    if np.linalg.det(R2) < 0:
        Vt2 = Vt.copy(); Vt2[1, :] *= -1
        R2 = Vt2.T @ U.T
    yaw_off = math.degrees(math.atan2(R2[1, 0], R2[0, 0]))
    yaw_en = (P @ R2.T) + gps_en[0]
    err_yaw = np.linalg.norm(yaw_en - gps_en, axis=1)

    # Level 3: full Kabsch (rotation + centroid translation)
    R3, t3, refl = _kabsch_2d(slam_en, gps_en)
    shape_en = slam_en @ R3.T + t3
    err_shape = np.linalg.norm(shape_en - gps_en, axis=1)

    # Vertical: SLAM z is DOWN
    slam_up = -(sp[:, 2] - sp[0, 2])
    gps_up = gp[:, 2] - gp[0, 2]
    err_z = np.abs(slam_up - gps_up)

    alt_up = None
    err_z_alt = None
    if alt_ref and len(run.alt_t) > 1:
        a = np.interp(t, run.alt_t, run.alt_r)
        rpy, _ = nearest_rpy(run.imu_t, run.imu_rpy, t) if len(run.imu_t) else (None, None)
        if rpy is not None:
            a = a * np.cos(rpy[:, 0]) * np.cos(rpy[:, 1])   # slant -> vertical
        alt_up = a - a[0]
        err_z_alt = np.abs(slam_up - alt_up)

    # Distance-normalised drift: error per metre of ground truth travelled.
    # Far more transferable than "m/s of drift", which depends entirely on how
    # fast you happened to fly.
    gps_travel = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(gp, axis=0), axis=1))])
    drift_per_m = np.nan
    if gps_travel[-1] > 5.0:
        drift_per_m = float(np.polyfit(gps_travel, err_yaw, 1)[0])
    drift_per_s = np.nan
    if t[-1] - t[0] > 5.0:
        drift_per_s = float(np.polyfit(t - t[0], err_yaw, 1)[0])

    return {
        't': t, 'n': int(len(t)),
        'slam_en_raw': raw_en, 'slam_en_yaw': yaw_en, 'slam_en_shape': shape_en,
        'gps_en': gps_en, 'slam_up': slam_up, 'gps_up': gps_up, 'alt_up': alt_up,
        'err_raw': err_raw, 'err_yaw': err_yaw, 'err_shape': err_shape,
        'err_z': err_z, 'err_z_alt': err_z_alt,
        'map_yaw_offset_deg': yaw_off,
        'kabsch_yaw_deg': math.degrees(math.atan2(R3[1, 0], R3[0, 0])),
        'kabsch_translation_m': float(np.linalg.norm(t3 - (gps_en[0] - R3 @ slam_en[0]))),
        'reflection': refl,
        'gps_travel_m': float(gps_travel[-1]),
        'drift_per_m': drift_per_m,
        'drift_per_s': drift_per_s,
    }


# ═════════════════════════════════════════════════════════════════════════
#  Altimeter cross-check
# ═════════════════════════════════════════════════════════════════════════

def altimeter_check(run: Run):
    """Cross-check the belly altimeter against GPS/EKF altitude.

    Also quantifies the missing tilt compensation: altimeter_bridge.py
    publishes the raw slant range and slam_node.py consumes it as if it were
    a vertical height. True height = range * cos(roll) * cos(pitch).
    """
    if len(run.alt_t) < 2 or len(run.gps_t) < 2:
        return None
    t = run.alt_t
    m = (t >= run.gps_t[0]) & (t <= run.gps_t[-1])
    t = t[m]
    r = run.alt_r[m]
    if len(t) < 2:
        return None
    gps_up = np.interp(t, run.gps_t, run.gps_enu[:, 2])
    out = {'n': int(len(t)), 't': t, 'raw': r, 'gps_up': gps_up}
    if len(run.imu_t) > 1:
        rpy, _ = nearest_rpy(run.imu_t, run.imu_rpy, t)
        ctilt = np.cos(rpy[:, 0]) * np.cos(rpy[:, 1])
        corr = r * ctilt
        out['corrected'] = corr
        out['tilt_bias_m'] = float(np.mean(r - corr))
        out['tilt_bias_max_m'] = float(np.max(np.abs(r - corr)))
        d_raw = (r - r[0]) - (gps_up - gps_up[0])
        d_cor = (corr - corr[0]) - (gps_up - gps_up[0])
        out['rmse_raw_m'] = float(np.sqrt((d_raw ** 2).mean()))
        out['rmse_corrected_m'] = float(np.sqrt((d_cor ** 2).mean()))
    return out
