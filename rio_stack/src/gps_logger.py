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
    import serial
    import pynmea2
except ImportError:
    logging.error("gps_logger.py requires 'pyserial' and 'pynmea2': pip install pyserial pynmea2")
    sys.exit(1)


# ─── Packet formats (must match doppler_rio.py / slam_node.py) ───────────────
RIO_PKT = struct.Struct('<dfffIfff')   # t_frame, vx, vy, vz, n_inliers, cxx, cyy, czz
POSE_PKT_HDR = struct.Struct('<dId') # t_slam, n_map_pts, fwd_range (20 bytes)
# Followed by 128 bytes: 4×4 float64 row-major T_world
IMU_PKT = struct.Struct('<dffffff')  # t_mono, roll, pitch, yaw, wx, wy, wz (32 bytes)


# ─── Geodesy ─────────────────────────────────────────────────────────────────

def lla_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """Flat-earth LLA → ENU conversion.

    Valid within ~10 km of origin, which is well beyond any walking or
    vehicle test (typically < 500 m from start).  Error at 500 m from
    origin is < 0.005 m -- negligible compared to GPS CEP (~2.5 m).

    Returns (east_m, north_m, up_m) relative to the (lat0, lon0, alt0)
    origin.
    """
    d_lat = lat - lat0
    d_lon = lon - lon0
    east  = d_lon * math.cos(math.radians(lat0)) * 111_320.0
    north = d_lat * 110_540.0
    up    = alt - alt0
    return east, north, up


# ─── GPS Serial Reader Thread ────────────────────────────────────────────────

class GPSReader(threading.Thread):
    """Background thread: reads NMEA from serial, stores latest fix.

    Parses $GNGGA / $GPGGA sentences via pynmea2 (same as extract_gps.py).
    The first valid fix becomes the ENU origin. Thread-safe via a lock on
    the `latest` attribute -- the main loop consumes it at the poll rate.
    """

    def __init__(self, port: str, baud: int):
        super().__init__(daemon=True, name="GPSReader")
        self.port = port
        self.baud = baud
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.latest: dict | None = None     # consumed by main loop
        self.fix_count: int = 0
        self.origin: tuple | None = None    # (lat0, lon0, alt0) on first fix
        self.last_sats: int = 0
        self._ser: serial.Serial | None = None

    def stop(self):
        self._stop.set()

    def run(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=1.0)
            logging.info(f"GPS serial opened: {self.port} @ {self.baud}")
        except serial.SerialException as e:
            logging.error(f"ERROR opening GPS serial {self.port}: {e}")
            return

        while not self._stop.is_set():
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode('ascii', errors='replace').strip()
                if not (line.startswith('$GNGGA') or line.startswith('$GPGGA')):
                    continue

                msg = pynmea2.parse(line)
                if msg.gps_qual == 0:
                    # No fix yet
                    continue

                t_mono = time.monotonic()
                lat = msg.latitude
                lon = msg.longitude
                alt = float(msg.altitude) if msg.altitude else 0.0

                # First fix becomes ENU origin
                if self.origin is None:
                    self.origin = (lat, lon, alt)
                    logging.info(f"GPS origin set: {lat:.6f} {msg.lat_dir}, "
                                 f"{lon:.6f} {msg.lon_dir}, {alt:.1f} m MSL")

                e, n, u = lla_to_enu(lat, lon, alt, *self.origin)

                fix = {
                    'type':    'gps',
                    't_mono':  round(t_mono, 6),
                    'lat':     round(lat, 8),
                    'lon':     round(lon, 8),
                    'alt':     round(alt, 2),
                    'enu':     [round(e, 4), round(n, 4), round(u, 4)],
                    'sats':    int(msg.num_sats),
                    'quality': int(msg.gps_qual),
                }

                with self._lock:
                    self.latest = fix
                    self.fix_count += 1
                    self.last_sats = int(msg.num_sats)

            except pynmea2.ParseError:
                pass  # corrupt sentence -- common over serial, ignore
            except serial.SerialException:
                if not self._stop.is_set():
                    logging.warning("GPS serial error, stopping reader")
                break
            except Exception as exc:
                logging.error(f"GPS reader unexpected error: {exc}")

        if self._ser and self._ser.is_open:
            self._ser.close()

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

    # ── GPS reader thread ──
    gps = GPSReader(args.gps_serial, args.gps_baud)
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

    logging.info("── Configuration ──")
    logging.info(f"  Log file:   {log_path}")
    logging.info(f"  GPS:        {args.gps_serial} @ {args.gps_baud} baud")
    logging.info(f"  RIO UDP:    127.0.0.1:{args.rio_port}")
    logging.info(f"  SLAM UDP:   127.0.0.1:{args.pose_port}")
    logging.info(f"  IMU UDP:    127.0.0.1:{args.imu_port}")
    logging.info(f"  Ref height: {args.ref_height_m:.2f} m (handheld)")
    logging.info("Waiting for data...")

    n_gps = 0
    n_rio = 0
    n_slam = 0
    n_imu = 0
    t_start = time.monotonic()
    last_status = t_start

    # RIO velocity integrator (for computing odometry distance)
    rio_dist = 0.0
    rio_pos = np.zeros(3)
    last_rio_t = None

    try:
        with open(log_path, 'w') as f:
            # Write a metadata header as the first JSONL line
            meta = {
                'type': 'meta',
                't_start': round(t_start, 6),
                'gps_serial': args.gps_serial,
                'gps_baud': args.gps_baud,
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
                        t_frame, vx, vy, vz, inliers, _cxx, _cyy, _czz = RIO_PKT.unpack(data)
                        t_mono = time.monotonic()

                        # Integrate distance for odometry comparison
                        v = np.array([vx, vy, vz])
                        if last_rio_t is not None:
                            dt = t_mono - last_rio_t
                            if 0 < dt < 0.5:
                                rio_pos += v * dt
                                rio_dist += float(np.linalg.norm(v)) * dt
                        last_rio_t = t_mono

                        entry = {
                            'type':     'rio',
                            't_mono':   round(t_mono, 6),
                            't_frame':  round(t_frame, 6),
                            'vx':       round(float(vx), 4),
                            'vy':       round(float(vy), 4),
                            'vz':       round(float(vz), 4),
                            'inliers':  int(n_inliers),
                        }
                        f.write(json.dumps(entry) + '\n')
                        n_rio += 1
                except BlockingIOError:
                    pass

                # ── SLAM pose (drain all pending) ──
                try:
                    while True:
                        data, _ = pose_sock.recvfrom(2048)
                        hdr_sz = POSE_PKT_HDR.size
                        if len(data) < hdr_sz + 128:
                            continue
                        t_slam, n_map, fwd_range = POSE_PKT_HDR.unpack_from(data)
                        T = np.frombuffer(data[hdr_sz:hdr_sz + 128],
                                          dtype='<f8').reshape(4, 4).copy()
                        pos = T[:3, 3].tolist()
                        t_mono = time.monotonic()

                        entry = {
                            'type':      'slam',
                            't_mono':    round(t_mono, 6),
                            't_slam':    round(t_slam, 6),
                            'pos':       [round(p, 4) for p in pos],
                            'n_map':     int(n_map),
                            'fwd_range': round(float(fwd_range), 2),
                        }
                        f.write(json.dumps(entry) + '\n')
                        n_slam += 1
                except BlockingIOError:
                    pass

                # ── IMU data (drain all pending) ──
                try:
                    while True:
                        data, _ = imu_sock.recvfrom(128)
                        if len(data) != IMU_PKT.size:
                            continue
                        t_mono, roll, pitch, yaw, wx, wy, wz = IMU_PKT.unpack(data)
                        
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

                # ── Status printout (every 3 seconds) ──
                now = time.monotonic()
                if now - last_status >= 3.0:
                    elapsed = now - t_start
                    gps_tag = (f"GPS: {n_gps} fixes, {gps.last_sats} sats"
                               if gps.origin else "GPS: waiting for fix...")
                    slam_tag = f"SLAM: {n_slam} poses"
                    rio_tag = f"RIO: {n_rio} pkts, dist={rio_dist:.2f}m"
                    imu_tag = f"IMU: {n_imu} pkts"
                    logging.info(f"{elapsed:5.0f}s | {gps_tag} | {rio_tag} | {slam_tag} | {imu_tag}")
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

    # ── Session summary ──
    logging.info("── Session Summary ──")
    logging.info(f"  Log file:       {log_path}")
    logging.info(f"  GPS fixes:      {n_gps}")
    logging.info(f"  RIO frames:     {n_rio}")
    logging.info(f"  SLAM poses:     {n_slam}")
    logging.info(f"  IMU frames:     {n_imu}")
    logging.info(f"  RIO distance:   {rio_dist:.2f} m")
    logging.info(f"  RIO displacement: {np.linalg.norm(rio_pos):.2f} m")
    if gps.origin:
        logging.info(f"  GPS origin:     {gps.origin[0]:.6f}°, "
                     f"{gps.origin[1]:.6f}°, {gps.origin[2]:.1f}m MSL")
    logging.info(f"Run:  python3 tools/analyze_run.py {log_path}")


def main():
    p = argparse.ArgumentParser(
        description="GPS ground-truth + radar output logger for drift analysis")
    p.add_argument('--gps-serial', default='/dev/ttyUSB1',
                    help="Serial port for GPS NMEA module (default: /dev/ttyUSB1)")
    p.add_argument('--gps-baud', type=int, default=9600,
                    help="GPS serial baud rate (NEO-M8N default: 9600)")
    p.add_argument('--rio-port', type=int, default=5013,
                    help="UDP port to receive RIO velocity from doppler_rio.py")
    p.add_argument('--pose-port', type=int, default=5014,
                    help="UDP port to receive SLAM pose from slam_node.py")
    p.add_argument('--imu-port', type=int, default=5022,
                    help="UDP port to receive IMU data from imu_bridge.py")
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
