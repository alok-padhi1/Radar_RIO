#!/usr/bin/env python3
"""
radar_fanout.py
Single owner of the U300 UART port. Parses Points_float frames (magic word
0x0201040306050807, TLV type 1) on a dedicated reader thread and fans each
frame out, non-blocking, to two independent consumers:

  - rio_q:  every frame, drop-oldest-on-full, consumed by RIOWorker at the
            radar's native ~20 Hz rate. RIO must always see the freshest
            frame -- a stale velocity is worse than a dropped one.
  - slam_q: every Nth frame (default 4 -> 5 Hz from 20 Hz), decimated at
            enqueue time (not filtered after the fact), drop-oldest-on-full,
            consumed by SLAMWorker for keyframe accumulation + GICP.

Each worker forwards its result over UDP loopback to a downstream, fully
independent process (mavlink_bridge.py for RIO; any pose/map consumer for
SLAM) -- see ARCHITECTURE.md Section 1 for the rationale.

This file replaces radar_streamer.py's role as the UART owner. It reuses the
exact same magic-word/TLV parsing logic as Points_float_PySDK_UART.py and
radar_streamer.py so behavior on the wire is unchanged; only the fan-out
after parsing is new.
"""

import logging
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
import argparse
import queue
import socket
import struct
import sys
import threading
import time

import numpy as np
import serial
import serial.tools.list_ports

MAGIC_WORD = bytearray(b'\x02\x01\x04\x03\x06\x05\x08\x07')

# Wire format handed to downstream workers / UDP consumers:
#   uint32 points_num, then points_num * float32[4] (x, y, z, v), radar frame.
UDP_HEADER = struct.Struct('<I')
# FIX R-5: the frame's parse-time epoch now rides WITH the frame.
# ARCHITECTURE.md Sec 1 says "Frames are timestamped once, at parse time, by
# the reader thread, and that same timestamp rides with the frame into both
# queues" -- but send() only ever transmitted (count, points). Both consumers
# re-stamped at recvfrom(), so:
#   * doppler_rio's t_frame was a receive time, not an epoch (measured 27 ms
#     median offset, and unbounded whenever the consumer blocked);
#   * slam_node stamped a whole drained burst within microseconds of each
#     other, collapsing the keyframe deskew dt to ~0 so motion compensation
#     did nothing at all;
#   * EKF2_EV_DELAY / EK3_VIS_DELAY could not be derived from the log.
# Wire format v2:  magic 'RF02' | float64 t_parse | uint32 n | n*(f32 x,y,z,v)
# Legacy v1 packets (uint32 n | points) are still parsed by the consumers, so
# a mixed-version deployment degrades rather than breaks.
UDP_HEADER_V2 = struct.Struct('<4sdI')
WIRE_MAGIC_V2 = b'RF02'


class Frame:
    __slots__ = ('t', 'points')

    def __init__(self, t: float, points: np.ndarray):
        self.t = t          # time.monotonic() at parse completion
        self.points = points  # (N,4) float32, columns x,y,z,v, RADAR frame


def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def select_port(explicit: str | None) -> str:
    if explicit:
        return explicit
    ports = list_ports()
    if not ports:
        print("No COM/serial ports found. Check the USB connection.")
        sys.exit(1)
    print("Available serial ports:")
    for i, p in enumerate(ports):
        print(f"  {i + 1}. {p}")
    while True:
        try:
            sel = int(input("Select port number: "))
            if 1 <= sel <= len(ports):
                return ports[sel - 1]
        except ValueError:
            pass
        print("Invalid selection, try again.")


class SerialReaderThread(threading.Thread):
    """Owns the serial port exclusively. Nothing else may call ser.read().
    Automatically reconnects if the USB device disconnects mid-session."""

    def __init__(self, port: str, baud: int, rio_q: queue.Queue,
                 slam_q: queue.Queue, slam_decimation: int = 4):
        super().__init__(daemon=True, name="SerialReader")
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(port, baud, parity=serial.PARITY_NONE,
                                  stopbits=serial.STOPBITS_ONE, timeout=0.08)
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()
        self.rio_q = rio_q
        self.slam_q = slam_q
        self.slam_decimation = slam_decimation
        self._frame_counter = 0
        self._stop = threading.Event()
        self.frames_parsed = 0
        self.frames_dropped_rio = 0
        self.frames_dropped_slam = 0
        self._reconnects = 0

    def stop(self):
        self._stop.set()

    def _reconnect(self):
        """Close current port and try to reopen it, retrying every 2s."""
        try:
            self.ser.close()
        except Exception:
            pass
        while not self._stop.is_set():
            try:
                self.ser = serial.Serial(self.port, self.baud,
                                          parity=serial.PARITY_NONE,
                                          stopbits=serial.STOPBITS_ONE,
                                          timeout=0.08)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self._reconnects += 1
                print(f"[radar_fanout] ✅ Reconnected to {self.port} "
                      f"(reconnect #{self._reconnects})")
                return True
            except (serial.SerialException, OSError) as e:
                logging.info(f"⏳ Waiting for {self.port}... ({e})")
                self._stop.wait(2.0)
        return False

    def run(self):
        while not self._stop.is_set():
            try:
                frame_bytes = self._read_one_frame()
                if frame_bytes is None:
                    continue
                pts = self._extract_points(frame_bytes)
                if pts is None:
                    continue
                t = time.monotonic()
                f = Frame(t, pts)
                self.frames_parsed += 1

                self._put_dropping_oldest(self.rio_q, f, is_rio=True)

                self._frame_counter += 1
                if self._frame_counter % self.slam_decimation == 0:
                    self._put_dropping_oldest(self.slam_q, f, is_rio=False)
            except (serial.SerialException, OSError) as e:
                logging.info(f"⚠️  Serial error: {e}")
                logging.info(f"Attempting reconnect to {self.port}...")
                if not self._reconnect():
                    break  # stop was set

    def _put_dropping_oldest(self, q: queue.Queue, item: Frame, is_rio: bool):
        try:
            q.put_nowait(item)
        except queue.Full:
            try:
                q.get_nowait()  # evict stale frame
            except queue.Empty:
                pass
            try:
                q.put_nowait(item)
            except queue.Full:
                pass
            if is_rio:
                self.frames_dropped_rio += 1
            else:
                self.frames_dropped_slam += 1

    # -- UART parsing: identical logic to Points_float_PySDK_UART.py --

    def _read_one_frame(self) -> bytearray | None:
        ser = self.ser
        index = 0
        magic_byte = ser.read(1)
        frame_data = bytearray(b'')
        if not magic_byte:
            return None
        while True:
            if self._stop.is_set():
                return None
            if not magic_byte:
                magic_byte = ser.read(1)
                continue
            if magic_byte[0] == MAGIC_WORD[index]:
                index += 1
                frame_data.append(magic_byte[0])
                if index == 8:
                    break
                magic_byte = ser.read(1)
            else:
                if index == 0:
                    magic_byte = ser.read(1)
                index = 0
                frame_data = bytearray(b'')

        version_bytes = ser.read(4)
        frame_data += bytearray(version_bytes)
        length_bytes = ser.read(4)
        frame_data += bytearray(length_bytes)
        frame_length = int.from_bytes(length_bytes, byteorder='little')
        frame_length -= 16
        if frame_length < 0 or frame_length > 1 << 20:
            return None  # corrupt length field, resync on next call
        frame_data += bytearray(ser.read(frame_length))
        return frame_data

    @staticmethod
    def _extract_points(dat: bytearray) -> np.ndarray | None:
        if len(dat) < 40:
            return None
        tlv_num = struct.unpack('I', dat[32:36])[0]
        if tlv_num == 0:
            return None
        tlv_index = 40
        tlv_type = struct.unpack('I', dat[tlv_index:tlv_index + 4])[0]
        if tlv_type != 1:
            return None
        tlv_length = struct.unpack('I', dat[tlv_index + 4:tlv_index + 8])[0]
        points_num = struct.unpack('I', dat[28:32])[0]
        payload_start = tlv_index + 8
        payload = dat[payload_start:payload_start + tlv_length]
        expected = points_num * 16
        if len(payload) < expected:
            return None
        arr = np.frombuffer(bytes(payload[:expected]), dtype='<f4', count=points_num * 4)
        return arr.reshape(points_num, 4).copy()


def make_udp_forwarder(ip: str, port_spec):
    if not port_spec:
        return None
    if isinstance(port_spec, int):
        port_list = [port_spec]
    elif isinstance(port_spec, str):
        port_list = [int(p.strip()) for p in port_spec.split(',') if p.strip()]
    else:
        port_list = list(port_spec)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(points_num: int, payload: bytes, t_parse: float = None):
        if t_parse is None:
            data = UDP_HEADER.pack(points_num) + payload          # legacy v1
        else:
            data = UDP_HEADER_V2.pack(WIRE_MAGIC_V2, t_parse, points_num) + payload
        for p in port_list:
            try:
                sock.sendto(data, (ip, p))
            except OSError:
                pass
    return send


class RIOForwardWorker(threading.Thread):
    """Drains rio_q and forwards raw frames to doppler_rio.py's UDP listener
    unchanged (keeps doppler_rio.py's interface exactly as-is)."""

    def __init__(self, rio_q: queue.Queue, forward_ip: str, forward_port: int):
        super().__init__(daemon=True, name="RIOForward")
        self.rio_q = rio_q
        self.send = make_udp_forwarder(forward_ip, forward_port)
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            try:
                f = self.rio_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if self.send:
                self.send(f.points.shape[0], f.points.astype('<f4').tobytes(), f.t)


class SLAMForwardWorker(threading.Thread):
    """Drains slam_q (already decimated to ~5 Hz) and forwards to
    slam_node.py and visualizer UDP listeners."""

    def __init__(self, slam_q: queue.Queue, forward_ip: str, forward_port):
        super().__init__(daemon=True, name="SLAMForward")
        self.slam_q = slam_q
        self.send = make_udp_forwarder(forward_ip, forward_port)
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            try:
                f = self.slam_q.get(timeout=0.5)
            except queue.Empty:
                continue
            if self.send:
                self.send(f.points.shape[0], f.points.astype('<f4').tobytes(), f.t)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', default=None, help="serial device; omit to be prompted")
    p.add_argument('--baud', type=int, default=921600)
    p.add_argument('--slam-decimation', type=int, default=4,
                    help="enqueue 1-in-N frames to the SLAM path (default 1: full 20Hz stream)")
    p.add_argument('--rio-ip', default='127.0.0.1')
    p.add_argument('--rio-port', type=int, default=5005,
                    help="matches doppler_rio.py --listen-port default")
    p.add_argument('--slam-ip', default='127.0.0.1')
    p.add_argument('--slam-port', default="5010,5012",
                    help="destination UDP port(s) for decimated stream (default: 5010,5012)")
    p.add_argument('--stats-interval', type=float, default=5.0)
    args = p.parse_args()

    port = select_port(args.port)
    rio_q: queue.Queue[Frame] = queue.Queue(maxsize=2)
    slam_q: queue.Queue[Frame] = queue.Queue(maxsize=1)

    reader = SerialReaderThread(port, args.baud, rio_q, slam_q, args.slam_decimation)
    rio_fwd = RIOForwardWorker(rio_q, args.rio_ip, args.rio_port)
    slam_fwd = SLAMForwardWorker(slam_q, args.slam_ip, args.slam_port)

    reader.start()
    rio_fwd.start()
    slam_fwd.start()

    print(f"[radar_fanout] {port} @ {args.baud} baud -> "
          f"RIO udp:{args.rio_ip}:{args.rio_port} (20Hz) | "
          f"SLAM udp:{args.slam_ip}:{args.slam_port} (~{20/args.slam_decimation:.1f}Hz)")

    try:
        while True:
            time.sleep(args.stats_interval)
            print(f"[radar_fanout] parsed={reader.frames_parsed} "
                  f"dropped_rio={reader.frames_dropped_rio} "
                  f"dropped_slam={reader.frames_dropped_slam}")
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        rio_fwd.stop()
        slam_fwd.stop()


if __name__ == '__main__':
    main()
