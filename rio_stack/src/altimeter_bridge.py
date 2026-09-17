#!/usr/bin/env python3
"""altimeter_bridge.py -- Linpowave U200A belly radar -> UDP fan-out.

The U200A is the ONLY absolute height reference in this GPS-denied stack.
SLAM Z is a free-running integrator (documented: 191 m drift in 96 s) and the
barometer drifts with weather and prop wash. Everything vertical -- the EKF
height state, the nav geofence, the failsafe descent, terrain following --
depends on this process staying alive.
"""
import argparse
import collections
import logging
import socket
import struct
import threading
import time
import numpy as np
import serial

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

ALT_PKT = struct.Struct('<f')       # matches mavlink_bridge.py

MAX_CLIMB_MPS = 5.0 # Max sane climb/descent speed for outlier rejection

class AltimeterReader(threading.Thread):
    def __init__(self, port, baud, lever_z_m, dests):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.lever_z_m = lever_z_m
        self.dests = dests
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.window = collections.deque(maxlen=5)
        self.last_accepted = None
        self.last_t = None
        
        self.n_out_of_envelope = 0
        self.n_rate_rejected = 0
        self.n_sent = 0

    def _accept(self, r_raw, t):
        if not (0.0 <= r_raw <= 200.0):
            self.n_out_of_envelope += 1
            return None
        self.window.append(r_raw)
        r = float(np.median(self.window))         # spike rejection
        if self.last_accepted is not None:
            dt = max(t - self.last_t, 1e-3)
            if abs(r - self.last_accepted) > 3.0 * MAX_CLIMB_MPS * dt:
                self.n_rate_rejected += 1
                return None                        # bird / vehicle / roof edge
        self.last_accepted, self.last_t = r, t
        return r + self.lever_z_m                  # referenced to the FC

    def _reconnect(self):
        while True:
            try:
                s = serial.Serial(self.port, self.baud, timeout=1.0)
                logging.info(f"Connected to altimeter on {self.port} at {self.baud}")
                return s
            except serial.SerialException as e:
                logging.warning(f"Failed to open {self.port}: {e}. Retrying in 2s...")
                time.sleep(2.0)

    def run(self):
        ser = self._reconnect()
        while True:
            try:
                # Find frame start: 0xEB 0x90
                b = ser.read(1)
                if not b or b[0] != 0xEB:
                    continue
                b2 = ser.read(1)
                if not b2 or b2[0] != 0x90:
                    continue
                    
                # Read remaining 4 bytes (frame_id, dist_h, dist_l, checksum)
                rem = ser.read(4)
                if len(rem) != 4:
                    continue
                    
                frame = bytearray([0xEB, 0x90]) + bytearray(rem)
                
                # Verify checksum
                chk = sum(frame[:5]) & 0xFF
                if chk != frame[5]:
                    continue
                    
                # Parse distance
                dist_raw = struct.unpack('>H', frame[3:5])[0]
                r_raw = dist_raw / 100.0  # converted to meters
                t = time.monotonic()
                
                r_fc = self._accept(r_raw, t)
                if r_fc is not None:
                    pkt = ALT_PKT.pack(r_fc)
                    for ip, port in self.dests:
                        self.sock.sendto(pkt, (ip, port))
                    
                    self.n_sent += 1
                    logging.info(f"Height: {r_fc:.2f} m")
                        
            except serial.SerialException:
                logging.error(f"Altimeter serial connection lost. Reconnecting...")
                ser.close()
                self.window.clear()
                self.last_accepted = None
                self.last_t = None
                ser = self._reconnect()

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', required=True, help="Serial port for U200A (e.g. /dev/ttyUSB1)")
    p.add_argument('--baud', type=int, default=921600, help="Baud rate for U200A")
    p.add_argument('--lever-z', type=float, required=True, help="Lever arm Z-offset (FC to Altimeter) in meters")
    p.add_argument('--dest-ports', type=str, required=True, help="Comma-separated UDP fanout ports")
    args = p.parse_args()

    dests = []
    for spec in args.dest_ports.split(','):
        spec = spec.strip()
        if not spec: continue
        if ':' in spec:
            ip, port = spec.split(':', 1)
            dests.append((ip, int(port)))
        else:
            dests.append(('127.0.0.1', int(spec)))

    reader = AltimeterReader(args.port, args.baud, args.lever_z, dests)
    reader.start()
    
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logging.info("Exiting.")

if __name__ == '__main__':
    main()
