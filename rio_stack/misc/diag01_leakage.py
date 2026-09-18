#!/usr/bin/env python3
"""
diag01_leakage.py
Listens to the raw UDP radar stream (from radar_fanout.py) and identifies
airframe self-returns (points with near-zero radial velocity).

Usage:
  1. Restrain drone, props spinning at 30% throttle.
  2. Run supervisor.py or radar_fanout.py normally.
  3. Run this script in another terminal:
     python3 diag01_leakage.py
  4. Let it run for 10-20 seconds, then press Ctrl+C.
  5. It will print the recommended --leakage-radius.
"""

import socket
import struct
import numpy as np
import collections
import argparse

UDP_HEADER = struct.Struct('<I')

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--ip', default='127.0.0.1')
    p.add_argument('--port', type=int, default=5005, help="Must match radar_fanout.py --rio-port")
    p.add_argument('--v-thresh', type=float, default=0.15, help="Speed threshold for static points (m/s)")
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.ip, args.port))
    print(f"Listening for raw radar frames on {args.ip}:{args.port}...")
    print("Press Ctrl+C to stop and analyze.\n")

    ranges_zero = []
    frames = 0

    try:
        while True:
            data, _ = sock.recvfrom(65535)
            if len(data) < UDP_HEADER.size:
                continue
            
            points_num = UDP_HEADER.unpack_from(data, 0)[0]
            expected = UDP_HEADER.size + (points_num * 16)
            if len(data) < expected:
                continue

            arr = np.frombuffer(data[UDP_HEADER.size:expected], dtype='<f4')
            pts = arr.reshape((points_num, 4))
            
            x, y, z, v = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
            r = np.linalg.norm(pts[:, :3], axis=1)
            
            # Find points that are "static" (self-returns)
            static_mask = np.abs(v) < args.v_thresh
            ranges_zero.extend(r[static_mask].tolist())
            frames += 1
            
            if frames % 20 == 0:
                print(f"Collected {len(ranges_zero)} near-zero points from {frames} frames...", end='\r')
                
    except KeyboardInterrupt:
        print("\n\n--- DIAG-01 RESULTS ---")
        if not ranges_zero:
            print("No near-zero points detected! Is the radar pointing at open sky?")
            return

        r_arr = np.array(ranges_zero)
        p95 = np.percentile(r_arr, 95)
        p99 = np.percentile(r_arr, 99)
        max_r = np.max(r_arr)
        
        print(f"Total static points collected: {len(r_arr)}")
        print(f"95th percentile range: {p95:.2f} m")
        print(f"99th percentile range: {p99:.2f} m")
        print(f"Absolute maximum range : {max_r:.2f} m")
        print("\nRecommendation for supervisor.py:")
        recommended = max_r + 0.15
        print(f"  --leakage-radius {recommended:.2f}")

if __name__ == '__main__':
    main()
