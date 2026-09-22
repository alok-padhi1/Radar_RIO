#!/usr/bin/env python3
"""
imu_bridge.py
MAVLink → UDP bridge for Cube Orange (or any ArduPilot/PX4 flight controller).

Connects to the flight controller via USB serial (typically /dev/ttyACM0),
requests the ATTITUDE stream at 50 Hz, and broadcasts the fused angular
velocity (rollspeed, pitchspeed, yawspeed) plus attitude (roll, pitch, yaw)
over local UDP for consumption by doppler_rio.py and slam_node.py.

Based on the team's extract_imu.py, redesigned as a headless UDP broadcaster
instead of a terminal printer.

Packet format (IMU_PKT, 37 bytes):
    struct '<dffffffBf'
    t_mono:   float64  — time.monotonic() at receive
    roll:     float32  — fused roll (rad)
    pitch:    float32  — fused pitch (rad)
    yaw:      float32  — fused yaw (rad)
    omega_x:  float32  — rollspeed (rad/s), body X axis
    omega_y:  float32  — pitchspeed (rad/s), body Y axis
    omega_z:  float32  — yawspeed (rad/s), body Z axis
    airborne: uint8    — 1 if FC reports IN_AIR, 0 otherwise
    vz_ned:   float32  — FC EKF vertical velocity, NED +down (m/s)
                         FRD body frame uses the same sign convention.

Usage (standalone):
    python3 imu_bridge.py --port /dev/ttyACM0

Usage (via supervisor.py):
    Launched automatically when --imu-port is specified.

Requires: pip install pymavlink
"""

import argparse
import socket
import struct
import sys
import time
import math

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: imu_bridge.py requires 'pymavlink':", flush=True)
    print("  pip install pymavlink", flush=True)
    sys.exit(1)

# Wire format: matches the listener in doppler_rio.py and slam_node.py
IMU_PKT = struct.Struct('<dffffffBfdd')  # 53 bytes: t, r,p,y, wx,wy,wz, airborne, vz_ned, t_vz, t_airborne


def main():
    p = argparse.ArgumentParser(description="Cube Orange IMU → UDP bridge")
    p.add_argument('--port', default='/dev/ttyACM0',
                   help="Serial port for the flight controller (default: /dev/ttyACM0)")
    p.add_argument('--baud', type=int, default=115200,
                   help="Baud rate for FC serial (default: 115200)")
    p.add_argument('--rate-hz', type=int, default=50,
                   help="Requested ATTITUDE stream rate in Hz (default: 50)")
    p.add_argument('--dest-ip', default='127.0.0.1')
    p.add_argument('--dest-ports', default='5020,5021',
                   help="Comma-separated UDP ports to broadcast IMU data to "
                        "(default: 5020 for doppler_rio, 5021 for slam_node)")
    p.add_argument('--stats-interval', type=float, default=10.0,
                   help="Seconds between status prints (default: 10)")
    p.add_argument('--pitch-offset-deg', type=float, default=0.0,
                   help="Offset to subtract from the IMU pitch (for uncalibrated mounts)")
    p.add_argument('--mavlink-fwd-port', type=int, default=14540,
                   help="UDP port to forward MAVLink packets (GPS) to the logger (default: 14540)")
    args = p.parse_args()
    pitch_offset = math.radians(args.pitch_offset_deg)

    dest_ports = [int(x.strip()) for x in args.dest_ports.split(',') if x.strip()]
    if not dest_ports:
        print("[imu_bridge] ERROR: no destination ports specified.", flush=True)
        sys.exit(1)

    # ── Connect to flight controller ──
    print(f"[imu_bridge] Connecting to FC on {args.port} at {args.baud} baud...", flush=True)
    try:
        master = mavutil.mavlink_connection(args.port, baud=args.baud)
    except Exception as e:
        print(f"[imu_bridge] ERROR: Failed to connect to {args.port}: {e}", flush=True)
        sys.exit(1)

    print("[imu_bridge] Waiting for heartbeat...", flush=True)
    try:
        master.wait_heartbeat(timeout=15.0)
    except Exception:
        print("[imu_bridge] ERROR: Timeout waiting for heartbeat. "
              "Check that the Cube Orange is powered and plugged in.", flush=True)
        sys.exit(1)

    print(f"[imu_bridge] ✅ Heartbeat received — System {master.target_system}, "
          f"Component {master.target_component}", flush=True)

    # Request ALL streams to ensure we get ATTITUDE and GLOBAL_POSITION_INT
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        args.rate_hz, 1)

    print(f"[imu_bridge] Requested ALL at {args.rate_hz} Hz → "
          f"UDP {args.dest_ip}:{dest_ports} (plus GPS to {args.mavlink_fwd_port})", flush=True)

    # ── UDP output socket ──
    out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # ── Main loop ──
    msg_count = 0
    gps_count = 0
    t_last_stats = time.monotonic()
    # State for the extended IMU packet fields.
    _vz_ned: float = 0.0
    _airborne: int = 0
    _t_vz_ned: float = 0.0
    _t_airborne: float = 0.0

    try:
        while True:
            msg = master.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
                
            msg_type = msg.get_type()
            now_mono = time.monotonic()
            
            if msg_type in ('GLOBAL_POSITION_INT', 'GPS_RAW_INT'):
                try:
                    out_sock.sendto(msg.get_msgbuf(), (args.dest_ip, args.mavlink_fwd_port))
                    if msg_type == 'GLOBAL_POSITION_INT':
                        gps_count += 1
                except OSError:
                    pass

            elif msg_type == 'LOCAL_POSITION_NED':
                _vz_ned = float(msg.vz)
                _t_vz_ned = now_mono

            elif msg_type == 'EXTENDED_SYS_STATE':
                # BUG-4 Fix: landed_state 1=ON_GROUND. Treat anything else as IN_AIR
                # so we don't accidentally enable static hypothesis during TAKEOFF/LANDING.
                _airborne = 1 if getattr(msg, 'landed_state', 1) != 1 else 0
                _t_airborne = now_mono
                    
            elif msg_type == 'ATTITUDE':
                t_mono = now_mono
                msg_count += 1
    
                roll = msg.roll
                pitch = msg.pitch - pitch_offset
                yaw = msg.yaw
                omega_x = msg.rollspeed
                omega_y = msg.pitchspeed
                omega_z = msg.yawspeed
    
                # Pack and broadcast
                pkt = IMU_PKT.pack(
                    t_mono,
                    roll,
                    pitch,
                    yaw,
                    omega_x,
                    omega_y,
                    omega_z,
                    _airborne,
                    _vz_ned,
                    _t_vz_ned,
                    _t_airborne,
                )
                for port in dest_ports:
                    try:
                        out_sock.sendto(pkt, (args.dest_ip, port))
                    except OSError:
                        pass
    
                # Periodic status print
                now = time.monotonic()
                if now - t_last_stats >= args.stats_interval:
                    rate = msg_count / (now - t_last_stats) if (now - t_last_stats) > 0 else 0
                    print(f"[imu_bridge] {msg_count} ATTITUDE ({rate:.0f} Hz), {gps_count} GPS | "
                          f"RPY=[{msg.roll:+.2f}, {msg.pitch:+.2f}, {msg.yaw:+.2f}] rad | "
                          f"ω=[{msg.rollspeed:+.3f}, {msg.pitchspeed:+.3f}, {msg.yawspeed:+.3f}] rad/s", flush=True)
                    msg_count = 0
                    gps_count = 0
                    t_last_stats = now

    except KeyboardInterrupt:
        print("\n[imu_bridge] Shutting down.", flush=True)
        # Stop requesting the data stream
        try:
            master.mav.request_data_stream_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 0, 0)
        except Exception:
            pass


if __name__ == '__main__':
    main()
