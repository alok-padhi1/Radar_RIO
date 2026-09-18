#!/usr/bin/env python3
"""
mavlink_bridge.py
Formats and sends radar-derived state to a PX4 or ArduPilot flight
controller over MAVLink, using pymavlink. Consumes the UDP forwarding
packets emitted by doppler_rio.py (--forward-port) and, separately, a
downward altimeter-radar range on a second UDP port.

Message choice per autopilot -- these are NOT interchangeable, and using
the wrong one is a common integration bug, not just a style choice:

  PX4 (EKF2 external vision fusion):
      ODOMETRY (#331), frame_id=LOCAL_NED, child_frame_id=BODY_FRD,
      velocity-only (position/attitude fields NaN until you trust the
      SLAM back-end's pose). This is what PX4's mavlink module bridges
      into vehicle_visual_odometry. CAVEAT: on PX4 >=1.14 running a ROS 2
      companion stack, the currently-preferred path is publishing
      px4_msgs/VehicleOdometry directly over uXRCE-DDS instead of routing
      through the legacy MAVLink bridge -- verify against the exact PX4
      version/build you fly, this changes across releases.

  ArduPilot (EKF3 external-nav fusion):
      VISION_SPEED_ESTIMATE (#103), velocity-only, matches
      EK3_SRC1_VELXY = 6 ("ExternalNav"). ArduPilot does not consume the
      common ODOMETRY(#331) message as a position/velocity aiding source
      the way PX4 does -- sending it there is a silent no-op, not an
      error, which makes it a nasty bug to catch on the bench. Once you
      trust an integrated/SLAM-corrected pose (not just raw RIO
      velocity), add VISION_POSITION_ESTIMATE (#102) alongside it and
      set EK3_SRC1_POSXY = 6 too.

  Both:
      DISTANCE_SENSOR (#132) for the downward altimeter radar,
      orientation=PITCH_270 (straight down), type=RADAR.

This script never arms, disarms, or sends control setpoints -- it only
ever pushes aiding data into a source the vehicle's own EKF already
knows how to sanity-check and blend. Keep it that way in your test
progression (see accompanying writeup).
"""

import argparse
import socket
import struct
import time
from pymavlink import mavutil


# Extended RIO packet — must match doppler_rio.py FORWARD_PKT exactly (46 bytes)
RIO_PKT = struct.Struct('<dfffIfffIBf')  # t, vx, vy, vz, n_inliers, cxx, cyy, czz, n_total, flags, cond
ALT_PKT = struct.Struct('<f')      # range_m -- from your altimeter radar's parser


def open_link(args):
    conn = mavutil.mavlink_connection(args.mavlink_dest, baud=args.baud,
                                       source_system=args.sysid,
                                       source_component=args.compid)
    print(f"[mavlink_bridge] waiting for heartbeat on {args.mavlink_dest} ...")
    conn.wait_heartbeat(timeout=10)
    print(f"[mavlink_bridge] heartbeat OK (target sys={conn.target_system} "
          f"comp={conn.target_component})")
    return conn


def send_odometry_px4(conn, t_usec, vx, vy, vz, cov_v_diag=(0.05, 0.05, 0.05)):
    nan = float('nan')
    vel_cov = [nan] * 21
    # MAVLink packs a 6x6 upper-triangular covariance row-major; diag(vx,vy,vz)
    # lands at indices 0, 6, 11 -- see ODOMETRY message spec.
    vel_cov[0], vel_cov[6], vel_cov[11] = cov_v_diag
    conn.mav.odometry_send(
        t_usec,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,      # frame_id (position frame, unused: pos is NaN)
        mavutil.mavlink.MAV_FRAME_BODY_FRD,       # child_frame_id (velocity frame)
        nan, nan, nan,                             # x, y, z -- unknown, RIO is velocity-only
        [nan, nan, nan, nan],                       # attitude quaternion -- unknown here
        vx, vy, vz,
        nan, nan, nan,                              # body angular rates -- unknown here
        [nan] * 21, vel_cov,
        0,                                           # reset_counter
        mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,
        0,                                           # quality: unknown = 0
    )


def send_vision_speed_ardupilot(conn, t_usec, vx, vy, vz, cov_v_diag=(0.05, 0.05, 0.05)):
    cov = [cov_v_diag[0], 0, 0, 0, cov_v_diag[1], 0, 0, 0, cov_v_diag[2]]
    conn.mav.vision_speed_estimate_send(t_usec, vx, vy, vz, cov, reset_counter=0)


def send_distance_sensor(conn, t_boot_ms, range_m, min_range_m=0.2, max_range_m=200.0):
    """Linpowave U200A: 0.2-200 m, +/-0.2 m, 20 Hz. These limits MUST match
    RNGFND1_MIN_CM / RNGFND1_MAX_CM on the autopilot or ArduPilot will
    silently discard in-range readings."""
    conn.mav.distance_sensor_send(
        t_boot_ms,
        int(min_range_m * 100), int(max_range_m * 100),
        int(max(range_m, 0.0) * 100),
        mavutil.mavlink.MAV_DISTANCE_SENSOR_RADAR,
        id=1,
        orientation=mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,
        covariance=int(20),          # 0.2 m -> 20 cm, per the message spec
    )


def run(args):
    conn = open_link(args)

    rio_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rio_sock.bind((args.rio_ip, args.rio_port))
    import select
    
    alt_sock = None
    if args.alt_port:
        alt_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        alt_sock.bind((args.alt_ip, args.alt_port))

    socks = [rio_sock] + ([alt_sock] if alt_sock else [])
    for s in socks:
        s.setblocking(False)

    t0_wall = time.monotonic()
    t_offset_us = None

    while True:
        # Opportunistically refresh SYSTEM_TIME mapping
        st = conn.recv_match(type='SYSTEM_TIME', blocking=False)
        if st is not None:
            t_offset_us = st.time_boot_ms * 1000 - int(time.monotonic() * 1e6)

        ready, _, _ = select.select(socks, [], [], 0.05)
        
        if rio_sock in ready:
            data, _ = rio_sock.recvfrom(64)
            t_frame, vx, vy, vz, n_inliers, cxx, cyy, czz, _ntot, _flags, _cond = RIO_PKT.unpack(data)
            
            # Use the RADAR FRAME time, not "now" -- the pipeline latency
            # is what EK3_VIS_DELAY must compensate, and it's measured from t_frame.
            t_usec = int(t_frame * 1e6) + (t_offset_us or 0)
            
            if args.autopilot == 'px4':
                send_odometry_px4(conn, t_usec, vx, vy, vz, cov_v_diag=(cxx, cyy, czz))
            else:
                send_vision_speed_ardupilot(conn, t_usec, vx, vy, vz, cov_v_diag=(cxx, cyy, czz))

        if alt_sock in ready:
            adata, _ = alt_sock.recvfrom(64)
            (range_m,) = ALT_PKT.unpack(adata[:4])
            t_boot_ms = int(time.monotonic() * 1000 + (t_offset_us or 0) // 1000)
            send_distance_sensor(conn, t_boot_ms, range_m)


def self_test():
    """Packs each message with the real dialect encoder -- no live link needed.
    Catches field-order/type mistakes before you're standing next to a
    spinning prop, not after."""
    from pymavlink.dialects.v20 import common as mavlink2
    mav = mavlink2.MAVLink(file=None, srcSystem=1, srcComponent=1)

    nan = float('nan')
    vel_cov = [nan] * 21
    vel_cov[0], vel_cov[6], vel_cov[11] = 0.05, 0.05, 0.05
    msg = mav.odometry_encode(
        123456, mavutil.mavlink.MAV_FRAME_LOCAL_NED, mavutil.mavlink.MAV_FRAME_BODY_FRD,
        nan, nan, nan, [nan, nan, nan, nan], 12.5, 0.8, -0.6, nan, nan, nan,
        [nan] * 21, vel_cov, 0, mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION, 0,
    )
    print(f"[self_test] ODOMETRY packed OK, {len(msg.pack(mav))} bytes, "
          f"msgid={msg.get_msgId()}")

    msg2 = mav.vision_speed_estimate_encode(
        123456, 12.5, 0.8, -0.6, [0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05], 0
    )
    print(f"[self_test] VISION_SPEED_ESTIMATE packed OK, {len(msg2.pack(mav))} bytes, "
          f"msgid={msg2.get_msgId()}")

    msg3 = mav.distance_sensor_encode(
        1000, 10, 3000, 1850, mavutil.mavlink.MAV_DISTANCE_SENSOR_RADAR, 1,
        mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, 0,
    )
    print(f"[self_test] DISTANCE_SENSOR packed OK, {len(msg3.pack(mav))} bytes, "
          f"msgid={msg3.get_msgId()}")
    print("[self_test] PASS")


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--selftest', action='store_true')
    p.add_argument('--autopilot', choices=['px4', 'ardupilot'], default='px4')
    p.add_argument('--mavlink-dest', default='udp:127.0.0.1:14540',
                    help="e.g. udp:127.0.0.1:14540 (SITL) or /dev/ttyUSB0 for a real link")
    p.add_argument('--baud', type=int, default=921600)
    p.add_argument('--sysid', type=int, default=1)
    p.add_argument('--compid', type=int, default=197)  # MAV_COMP_ID_ODOMETRY-ish range
    p.add_argument('--rio-ip', default='127.0.0.1')
    p.add_argument('--rio-port', type=int, default=5006)
    p.add_argument('--alt-ip', default='127.0.0.1')
    p.add_argument('--alt-port', type=int, default=0)
    args = p.parse_args()

    self_test() if args.selftest else run(args)
