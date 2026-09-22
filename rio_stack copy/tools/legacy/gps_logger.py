#!/usr/bin/env python3
"""
gps_logger.py
Simultaneous GPS ground-truth and radar output logger for post-run
drift and performance analysis.

Runs alongside the radar stack (supervisor.py) and records three data
streams into a single timestamped JSONL file:

  1. GPS fixes (lat/lon/alt → ENU meters) from a NMEA serial module
  2. RIO velocity (v_body) from doppler_rio.py via UDP
  3. SLAM pose (T_world 4×4) from slam_node.py via UDP

All entries share a common `t_mono` timestamp (time.monotonic()) which
allows post-run time-alignment in analyze_run.py.

GPS positions are converted to East-North-Up (ENU) meters relative to
the first satellite fix. This puts GPS and SLAM in the same coordinate
frame and units for direct comparison.

Usage (standalone):
    python3 gps_logger.py --gps-serial /dev/ttyUSB1

Usage (via supervisor.py):
    python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 \
        --no-mavlink --gps-port /dev/ttyUSB1

Requires: pip install pynmea2 pyserial numpy
"""

import argparse
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from datetime import datetime
import logging

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

import numpy as np

try:
    from pymavlink import mavutil
except ImportError:
    logging.error("gps_logger.py requires 'pymavlink': pip install pymavlink")
    sys.exit(1)


# ─── Packet formats (must match doppler_rio.py / slam_node.py) ───────────────
# Extended RIO packet — must match doppler_rio.py FORWARD_PKT exactly (45 bytes).
# flags bit0=is_static, bit1=airborne, bit2=vz_prior_used, bit3=accel_gate_armed
RIO_PKT = struct.Struct('<dfffIfffIBf')   # t_frame, vx, vy, vz, n_inliers, cxx, cyy, czz, n_total, flags, cond
POSE_PKT_HDR = struct.Struct('<dId') # t_slam, n_map_pts, fwd_range (20 bytes)
# Followed by 128 bytes: 4×4 float64 row-major T_world
# IMU packet from imu_bridge.py (53 bytes): t, roll, pitch, yaw, wx, wy, wz, airborne, vz_ned, t_vz, t_airborne
IMU_PKT = struct.Struct('<dffffffBfdd')  # 53 bytes
ALT_PKT = struct.Struct('<f')        # altimeter range_m (4 bytes)
# Rejected-frame telemetry from doppler_rio.py --reject-ports
REJECT_PKT = struct.Struct('<dIIBff')    # t, n_total, n_inliers, reason, cond, extra
REJECT_REASONS_INV = {
    1: 'too_few_raw_points', 2: 'too_few_after_range_gate', 3: 'ransac_reject',
    4: 'wls_seed_failed', 5: 'irls_failed', 6: 'sigma_gate', 7: 'accel_gate',
    8: 'max_sigma_publish_gate',
}
# v2 pose packet from slam_node.py --pose-wire-v2
POSE_PKT_HDR_V2 = struct.Struct('<4sdIdIffBII')
POSE_MAGIC_V2 = b'SP02'
SLAM_REJECT_PKT = struct.Struct('<dBIIIIII')
SLAM_REJECT_INV = {1: 'too_few_points', 2: 'too_sparse_after_filtering',
                   3: 'insufficient_correspondences', 4: 'gicp_exception'}


# ─── Geodesy ─────────────────────────────────────────────────────────────────

_WGS84_A  = 6378137.0
_WGS84_E2 = 6.69437999014e-3


def enu_scale_factors(lat0_deg):
    """Metres per degree of latitude / longitude at lat0 on the WGS-84 ellipsoid."""
    lat0 = math.radians(lat0_deg)
    sn = math.sin(lat0)
    denom = math.sqrt(1.0 - _WGS84_E2 * sn * sn)
    M = _WGS84_A * (1.0 - _WGS84_E2) / denom ** 3     # meridional radius
    N = _WGS84_A / denom                               # prime-vertical radius
    return M * math.pi / 180.0, N * math.cos(lat0) * math.pi / 180.0


def lla_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """Local tangent-plane LLA -> ENU, WGS-84 scale factors evaluated at lat0.

    FIX: the previous version hard-coded 110_852.0 m per degree of latitude.
    That is the correct value at exactly 30 deg N and nowhere else:
        equator  110574 m/deg  -> the constant is +0.25 % high
        45 deg   111132 m/deg  -> -0.25 % low
        60 deg   111412 m/deg  -> -0.50 % low
    A 0.5 % scale error on the "ground truth" reads out as a 0.5 % RIO
    distance error that no amount of solver tuning will remove. The east
    factor also omitted the prime-vertical correction (~0.08 % at 30 deg).
    Flat-tangent-plane curvature error stays under 1 cm within ~1 km of the
    origin, which is the assumption that actually holds.
    """
    m_per_deg_lat, m_per_deg_lon = enu_scale_factors(lat0)
    east  = (lon - lon0) * m_per_deg_lon
    north = (lat - lat0) * m_per_deg_lat
    up    = alt - alt0
    return east, north, up


# ─── MAVLink GPS Reader Thread ────────────────────────────────────────────────

class MavlinkGPSReader(threading.Thread):
    """Background thread: reads GLOBAL_POSITION_INT from MAVLink."""

    def __init__(self, dest: str):
        super().__init__(daemon=True, name="MavlinkGPSReader")
        self.dest = dest
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.latest: dict | None = None
        self.fix_count: int = 0
        self.origin: tuple | None = None
        self.last_sats: int = 0
        self._conn = None

    def stop(self):
        self._stop.set()

    def run(self):
        try:
            self._conn = mavutil.mavlink_connection(self.dest)
            logging.info(f"MAVLink GPS listener opened: {self.dest}")
        except Exception as e:
            logging.error(f"ERROR opening MAVLink {self.dest}: {e}")
            return

        while not self._stop.is_set():
            try:
                msg = self._conn.recv_match(type=['GLOBAL_POSITION_INT', 'GPS_RAW_INT'], blocking=True, timeout=0.5)
                if not msg:
                    continue
                    
                if msg.get_type() == 'GPS_RAW_INT':
                    self.last_sats = msg.satellites_visible
                    
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.alt / 1000.0  # mm to m MSL
                    # FIX (instrumentation): GLOBAL_POSITION_INT already carries
                    # the EKF velocity in cm/s (NED). It was being discarded, so
                    # every ground-truth velocity downstream had to be obtained
                    # by differentiating position -- costing ~0.1 m/s of noise
                    # and ~0.3 s of bandwidth for nothing. Log the native field.
                    v_ned = (msg.vx / 100.0, msg.vy / 100.0, msg.vz / 100.0)
                    v_enu = (v_ned[1], v_ned[0], -v_ned[2])
                    rel_alt = getattr(msg, 'relative_alt', 0) / 1000.0
                    hdg = getattr(msg, 'hdg', 65535)

                    if lat == 0.0 and lon == 0.0:
                        continue  # No fix yet

                    t_mono = time.monotonic()

                    if self.origin is None:
                        self.origin = (lat, lon, alt)
                        logging.info(f"GPS origin set: {lat:.6f}, {lon:.6f}, {alt:.1f} m MSL")

                    e, n, u = lla_to_enu(lat, lon, alt, *self.origin)

                    fix = {
                        'type':    'gps',
                        't_mono':  round(t_mono, 6),
                        'lat':     round(lat, 8),
                        'lon':     round(lon, 8),
                        'alt':     round(alt, 2),
                        'enu':     [round(e, 4), round(n, 4), round(u, 4)],
                        'v_enu':   [round(v, 4) for v in v_enu],
                        'v_ned':   [round(v, 4) for v in v_ned],
                        'rel_alt': round(rel_alt, 3),
                        'hdg_deg': (None if hdg == 65535 else round(hdg / 100.0, 2)),
                        'sats':    self.last_sats,
                        'quality': 1 if self.last_sats >= 4 else 0, # rough proxy
                    }

                    with self._lock:
                        self.latest = fix
                        self.fix_count += 1
            except Exception as exc:
                logging.error(f"MAVLink GPS reader unexpected error: {exc}")
                break

        if hasattr(self, '_conn') and self._conn:
            self._conn.close()

    def consume_fix(self) -> dict | None:
        """Get and consume the latest fix (returns None if no new fix)."""
        with self._lock:
            fix = self.latest
            self.latest = None
            return fix


# ─── Main Logging Loop ───────────────────────────────────────────────────────

def run(args):
    # ── Log file setup ──
    os.makedirs(args.log_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = os.path.join(args.log_dir, f'run_{ts}.jsonl')

    # ── MAVLink GPS reader thread ──
    gps = MavlinkGPSReader(args.mavlink_dest)
    gps.start()

    # ── UDP sockets for radar data (dedicated ports, no SO_REUSEPORT needed) ──
    rio_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rio_sock.bind(('127.0.0.1', args.rio_port))
    rio_sock.setblocking(False)

    pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pose_sock.bind(('127.0.0.1', args.pose_port))
    pose_sock.setblocking(False)

    imu_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    imu_sock.bind(('127.0.0.1', args.imu_port))
    imu_sock.setblocking(False)

    alt_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    alt_sock.bind(('127.0.0.1', args.alt_port))
    alt_sock.setblocking(False)

    rej_sock = None
    if args.reject_port:
        rej_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rej_sock.bind(('127.0.0.1', args.reject_port))
        rej_sock.setblocking(False)

    logging.info("── Configuration ──")
    logging.info(f"  Log file:   {log_path}")
    logging.info(f"  GPS MAVL:   {args.mavlink_dest}")
    logging.info(f"  RIO UDP:    127.0.0.1:{args.rio_port}")
    logging.info(f"  SLAM UDP:   127.0.0.1:{args.pose_port}")
    logging.info(f"  IMU UDP:    127.0.0.1:{args.imu_port}")
    logging.info(f"  ALT UDP:    127.0.0.1:{args.alt_port}")
    logging.info(f"  Ref height: {args.ref_height_m:.2f} m (handheld)")
    logging.info("Waiting for data...")

    n_gps = 0
    n_rio = 0
    n_slam = 0
    n_imu = 0
    n_alt = 0
    n_rio_rej = 0
    n_slam_rej = 0
    t_start = time.monotonic()
    last_status = t_start

    # RIO velocity integrator (for computing odometry distance)
    rio_dist = 0.0
    rio_pos = np.zeros(3)
    rio_gap_s = 0.0
    rio_gaps = 0
    last_rio_t = None
    last_rio_v = None

    try:
        with open(log_path, 'w') as f:
            # Write a metadata header as the first JSONL line
            meta = {
                'type': 'meta',
                't_start': round(t_start, 6),
                'mavlink_dest': args.mavlink_dest,
                'ref_height_m': args.ref_height_m,
                'rio_port': args.rio_port,
                'pose_port': args.pose_port,
                'timestamp': datetime.now().isoformat(),
            }
            f.write(json.dumps(meta) + '\n')

            while True:
                time.sleep(0.005)  # 200 Hz poll loop (~5ms granularity)

                # ── GPS ──
                fix = gps.consume_fix()
                if fix:
                    f.write(json.dumps(fix) + '\n')
                    n_gps += 1

                # ── RIO velocity (drain all pending) ──
                try:
                    while True:
                        data, _ = rio_sock.recvfrom(128)
                        if len(data) < RIO_PKT.size:
                            continue
                        (t_frame, vx, vy, vz, inliers, cxx, cyy, czz,
                         n_total, flags, cond) = RIO_PKT.unpack(data)
                        t_mono = time.monotonic()

                        is_static = bool(flags & 1)
                        airborne = bool((flags >> 1) & 1)
                        vz_prior_used = bool((flags >> 2) & 1)

                        # Integrate distance for odometry comparison
                        v_curr = np.array([vx, vy, vz])
                        if last_rio_t is not None and last_rio_v is not None:
                            dt = t_mono - last_rio_t
                            # FIX: cap dt. The old code trapezoid-integrated
                            # straight across dropouts of ANY length -- a 12.5 s
                            # gap was filled in as if the last known velocity had
                            # held throughout. That is fabricated distance.
                            if 0 < dt <= 0.5:
                                v_avg = (last_rio_v + v_curr) / 2.0
                                # NOTE: v is BODY-frame. Summing it without
                                # rotating by heading is not a displacement and
                                # never was; rio_dist (a path length) is fine,
                                # rio_pos is not. Kept only for continuity of the
                                # console line -- use tools/analyze_run_v2.py,
                                # which rotates through the FC attitude.
                                rio_pos += v_avg * dt
                                rio_dist += float(np.linalg.norm(v_avg)) * dt
                            elif dt > 0.5:
                                rio_gap_s += dt
                                rio_gaps += 1
                        
                        last_rio_t = t_mono
                        last_rio_v = v_curr

                        entry = {
                            'type':     'rio',
                            't_mono':   round(t_mono, 6),
                            't_frame':  round(t_frame, 6),
                            'vx':       round(float(vx), 4),
                            'vy':       round(float(vy), 4),
                            'vz':       round(float(vz), 4),
                            'inliers':  int(inliers),
                            'n_total':  int(n_total),
                            'is_static': is_static,
                            'airborne': airborne,
                            'vz_prior': vz_prior_used,
                            'cond':     round(float(cond), 2),
                            # FIX (instrumentation): the per-frame covariance
                            # was received and then thrown away. It is the exact
                            # number the autopilot EKF weights this measurement
                            # by -- if it is wrong (see finding R-1) nothing
                            # else in the log reveals it. Logged as sigma (m/s)
                            # because that is the reviewable quantity.
                            'sx': round(float(math.sqrt(max(cxx, 0.0))), 4),
                            'sy': round(float(math.sqrt(max(cyy, 0.0))), 4),
                            'sz': round(float(math.sqrt(max(czz, 0.0))), 4),
                        }
                        f.write(json.dumps(entry) + '\n')
                        n_rio += 1
                except BlockingIOError:
                    pass

                # ── SLAM pose (drain all pending) ──
                try:
                    while True:
                        data, _ = pose_sock.recvfrom(2048)
                        t_mono = time.monotonic()

                        # SLAM keyframe REJECTION (v2 only)
                        if len(data) == SLAM_REJECT_PKT.size:
                            (ts, code, nraw, nrange, ndopp, npers, nsor,
                             ncorr) = SLAM_REJECT_PKT.unpack(data)
                            f.write(json.dumps({
                                'type': 'slam_reject',
                                't_mono': round(t_mono, 6), 't_slam': round(ts, 6),
                                'reason': SLAM_REJECT_INV.get(code, 'unknown'),
                                'n_raw': int(nraw), 'n_after_range': int(nrange),
                                'n_after_doppler': int(ndopp),
                                'n_after_persistence': int(npers),
                                'n_after_sor': int(nsor), 'n_corr': int(ncorr),
                            }) + '\n')
                            n_slam_rej += 1
                            continue

                        entry = None
                        if (len(data) >= POSE_PKT_HDR_V2.size + 128
                                and data[:4] == POSE_MAGIC_V2):
                            (_m, t_slam, n_map, fwd_range, n_corr, fitness, rmse,
                             n_obs, n_raw, n_final) = POSE_PKT_HDR_V2.unpack_from(data)
                            hdr_sz = POSE_PKT_HDR_V2.size
                            T = np.frombuffer(data[hdr_sz:hdr_sz + 128],
                                              dtype='<f8').reshape(4, 4).copy()
                            entry = {
                                'type': 'slam',
                                't_mono': round(t_mono, 6),
                                't_slam': round(t_slam, 6),
                                'pos': [round(p, 4) for p in T[:3, 3].tolist()],
                                'R': [round(float(x), 6) for x in T[:3, :3].ravel()],
                                'n_map': int(n_map),
                                'fwd_range': round(float(fwd_range), 2),
                                'n_corr': int(n_corr),
                                'fitness': round(float(fitness), 4),
                                'rmse': round(float(rmse), 4),
                                'n_obs_axes': int(n_obs),
                                'n_raw': int(n_raw), 'n_final': int(n_final),
                            }
                        elif len(data) >= POSE_PKT_HDR.size + 128:
                            hdr_sz = POSE_PKT_HDR.size
                            t_slam, n_map, fwd_range = POSE_PKT_HDR.unpack_from(data)
                            T = np.frombuffer(data[hdr_sz:hdr_sz + 128],
                                              dtype='<f8').reshape(4, 4).copy()
                            entry = {
                                'type': 'slam',
                                't_mono': round(t_mono, 6),
                                't_slam': round(t_slam, 6),
                                'pos': [round(p, 4) for p in T[:3, 3].tolist()],
                                'R': [round(float(x), 6) for x in T[:3, :3].ravel()],
                                'n_map': int(n_map),
                                'fwd_range': round(float(fwd_range), 2),
                            }
                        if entry is None:
                            continue
                        f.write(json.dumps(entry) + '\n')
                        n_slam += 1
                except BlockingIOError:
                    pass

                # ── RIO rejected frames (drain all pending) ──
                if rej_sock is not None:
                    try:
                        while True:
                            data, _ = rej_sock.recvfrom(64)
                            if len(data) != REJECT_PKT.size:
                                continue
                            ts, ntot, ninl, code, cond, extra = REJECT_PKT.unpack(data)
                            f.write(json.dumps({
                                'type': 'rio_reject',
                                't_mono': round(time.monotonic(), 6),
                                't_frame': round(ts, 6),
                                'reason': REJECT_REASONS_INV.get(code, 'unknown'),
                                'n_total': int(ntot), 'inliers': int(ninl),
                                'cond': round(float(cond), 2),
                                'extra': round(float(extra), 4),
                            }) + '\n')
                            n_rio_rej += 1
                    except BlockingIOError:
                        pass

                # ── IMU data (drain all pending) ──
                try:
                    while True:
                        data, _ = imu_sock.recvfrom(128)
                        if len(data) != IMU_PKT.size:
                            continue
                        t_mono, roll, pitch, yaw, wx, wy, wz, airborne_b, vz_ned, t_vz, t_airborne = IMU_PKT.unpack(data)
                        
                        entry = {
                            'type': 'imu',
                            't_mono': round(t_mono, 6),
                            'roll': round(roll, 4),
                            'pitch': round(pitch, 4),
                            'yaw': round(yaw, 4),
                            'wx': round(wx, 4),
                            'wy': round(wy, 4),
                            'wz': round(wz, 4),
                        }
                        f.write(json.dumps(entry) + '\n')
                        n_imu += 1
                except BlockingIOError:
                    pass

                # ── Altimeter data (drain all pending) ──
                try:
                    while True:
                        data, _ = alt_sock.recvfrom(64)
                        if len(data) != ALT_PKT.size:
                            continue
                        (range_m,) = ALT_PKT.unpack(data)
                        
                        entry = {
                            'type': 'altimeter',
                            't_mono': round(time.monotonic(), 6),
                            'range_m': round(range_m, 4),
                        }
                        f.write(json.dumps(entry) + '\n')
                        n_alt += 1
                except BlockingIOError:
                    pass

                # ── Status printout (every 3 seconds) ──
                now = time.monotonic()
                if now - last_status >= 3.0:
                    elapsed = now - t_start
                    gps_tag = (f"GPS: {n_gps} fixes, {gps.last_sats} sats"
                               if gps.origin else "GPS: waiting for fix...")
                    slam_tag = f"SLAM: {n_slam} poses"
                    rio_tag = (f"RIO: {n_rio} ok / {n_rio_rej} rej, "
                               f"dist={rio_dist:.2f}m, gaps={rio_gaps}/{rio_gap_s:.0f}s")
                    imu_tag = f"IMU: {n_imu} pkts"
                    alt_tag = f"ALT: {n_alt} pkts"
                    logging.info(f"{elapsed:5.0f}s | {gps_tag} | {rio_tag} | {slam_tag} | {imu_tag} | {alt_tag}")
                    last_status = now
                    f.flush()

    except KeyboardInterrupt:
        elapsed = time.monotonic() - t_start
        logging.info(f"Stopped after {elapsed:.1f}s")
    finally:
        gps.stop()
        rio_sock.close()
        pose_sock.close()
        imu_sock.close()
        alt_sock.close()          # FIX: alt_sock was never closed
        if rej_sock:
            rej_sock.close()

    # ── Session summary ──
    logging.info("── Session Summary ──")
    logging.info(f"  Log file:       {log_path}")
    logging.info(f"  GPS fixes:      {n_gps}")
    logging.info(f"  RIO frames:     {n_rio}")
    logging.info(f"  SLAM poses:     {n_slam}  (rejected keyframes: {n_slam_rej})")
    logging.info(f"  RIO rejected:   {n_rio_rej}"
                 + (f"  -> accept rate {100*n_rio/max(n_rio+n_rio_rej,1):.1f} %"
                    if (n_rio + n_rio_rej) else ""))
    logging.info(f"  IMU frames:     {n_imu}")
    logging.info(f"  ALT frames:     {n_alt}")
    logging.info(f"  RIO distance:   {rio_dist:.2f} m")
    logging.info(f"  RIO dropouts:   {rio_gaps} gaps totalling {rio_gap_s:.1f} s")
    logging.info(f"  (BODY-frame displacement {np.linalg.norm(rio_pos):.2f} m -- "
                 f"NOT a world displacement; see analyze_run_v2.py)")
    if gps.origin:
        logging.info(f"  GPS origin:     {gps.origin[0]:.6f}°, "
                     f"{gps.origin[1]:.6f}°, {gps.origin[2]:.1f}m MSL")
    logging.info(f"Run:  python3 tools/analyze_run_v2.py {log_path} --plot run.png")


def main():
    p = argparse.ArgumentParser(
        description="GPS ground-truth + radar output logger for drift analysis")
    p.add_argument('--mavlink-dest', default='udp:127.0.0.1:14540',
                    help="MAVLink connection string (default: udp:127.0.0.1:14540)")
    p.add_argument('--rio-port', type=int, default=5013,
                    help="UDP port to receive RIO velocity from doppler_rio.py")
    p.add_argument('--pose-port', type=int, default=5014,
                    help="UDP port to receive SLAM pose from slam_node.py")
    p.add_argument('--imu-port', type=int, default=5022,
                    help="UDP port to receive IMU data from imu_bridge.py")
    p.add_argument('--alt-port', type=int, default=5033,
                    help="UDP port to receive Altimeter data from altimeter_bridge.py")
    p.add_argument('--reject-port', type=int, default=5015,
                    help="UDP port for doppler_rio.py's rejected-frame telemetry "
                         "(launch doppler_rio with --reject-ports 5015). 0 to disable.")
    p.add_argument('--log-dir', default='logs',
                    help="Directory for JSONL log files")
    p.add_argument('--ref-height-m', type=float, default=0.9,
                    help="Approximate sensor height above ground (meters). "
                         "Used for reference only; does not affect logging. "
                         "Default 0.9m for handheld testing.")
    args = p.parse_args()
    run(args)


if __name__ == '__main__':
    main()
