#!/usr/bin/env python3
"""
drivers/u300/decoder.py — U300 serial frame decoder.

Blueprint §5.1, §28: Extracts and validates U300 UART frames.
Preserves sensor timestamps, tracks sequence numbers, detects dropped
packets, reordering, and duplicates.

Extracted from radar_fanout.py to create a clean driver boundary.
"""

from __future__ import annotations

import logging
import struct
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import serial

logger = logging.getLogger(__name__)

MAGIC_WORD = bytearray(b'\x02\x01\x04\x03\x06\x05\x08\x07')


@dataclass
class FrameHealth:
    """Per-frame health telemetry — Blueprint §28.

    Log: rx_timestamp, sensor_timestamp, sequence, queue_depth,
    packet_loss, packet_reorder, point_count.
    """
    rx_timestamp: float = 0.0         # time.monotonic() at parse start
    sensor_timestamp: float = 0.0     # Sensor-provided timestamp (if available)
    parse_timestamp: float = 0.0      # time.monotonic() at parse completion
    sequence: int = 0                 # Frame sequence number
    point_count: int = 0
    packet_loss_cumulative: int = 0   # Cumulative lost frames
    packet_reorder_count: int = 0     # Cumulative reordered frames
    duplicate_count: int = 0          # Cumulative duplicate frames
    parser_error_count: int = 0       # Cumulative parser errors
    valid: bool = True


@dataclass
class DecoderStats:
    """Cumulative decoder statistics — Blueprint §28."""
    frames_parsed: int = 0
    frames_dropped: int = 0
    parser_errors: int = 0
    sequence_gaps: int = 0
    sequence_reorders: int = 0
    duplicates: int = 0
    total_points: int = 0
    last_sequence: int = -1
    last_sensor_timestamp: float = 0.0


class U300Decoder:
    """Stateful decoder for Linpowave U300 UART frames.

    Blueprint §5.1: Preserves original sensor timestamp, frame number/sequence,
    point count. Never replaces sensor timestamp with time.monotonic() if an
    original timestamp exists (§5.1).
    """

    def __init__(self):
        self.stats = DecoderStats()

    def decode_frame(self, raw_data: bytearray,
                     rx_timestamp: float = None) -> Optional[tuple[np.ndarray, FrameHealth]]:
        """Decode a raw U300 frame into points and health data.

        Args:
            raw_data: Complete frame bytes (including magic word)
            rx_timestamp: time.monotonic() at receive time

        Returns:
            (points, health) or None if decode fails.
            points: (N, 4) float32 array [x, y, z, v] in RADAR frame
        """
        if rx_timestamp is None:
            rx_timestamp = time.monotonic()

        health = FrameHealth(rx_timestamp=rx_timestamp)

        if len(raw_data) < 40:
            self.stats.parser_errors += 1
            health.valid = False
            return None

        # Extract header fields
        try:
            # Frame header: 8B magic + 4B version + 4B length + 16B sub-header
            # Sub-header at offset 28: points_num (uint32)
            # Sub-header at offset 32: TLV count (uint32)
            # Frame number at offset 20: frame_number (uint32) — if available
            frame_number = struct.unpack('I', raw_data[20:24])[0]
            points_num = struct.unpack('I', raw_data[28:32])[0]
            tlv_num = struct.unpack('I', raw_data[32:36])[0]

            # Sequence tracking
            health.sequence = frame_number
            self._track_sequence(frame_number)

        except (struct.error, IndexError):
            self.stats.parser_errors += 1
            health.valid = False
            return None

        if tlv_num == 0 or points_num == 0:
            health.point_count = 0
            health.valid = True
            health.parse_timestamp = time.monotonic()
            self.stats.frames_parsed += 1
            return np.empty((0, 4), dtype=np.float32), health

        # Extract TLV
        tlv_index = 40
        if len(raw_data) < tlv_index + 8:
            self.stats.parser_errors += 1
            health.valid = False
            return None

        try:
            tlv_type = struct.unpack('I', raw_data[tlv_index:tlv_index + 4])[0]
            if tlv_type != 1:  # Points_float TLV
                health.valid = False
                return None

            tlv_length = struct.unpack('I', raw_data[tlv_index + 4:tlv_index + 8])[0]
            payload_start = tlv_index + 8
            payload = raw_data[payload_start:payload_start + tlv_length]

            expected = points_num * 16  # 4 floats × 4 bytes each
            if len(payload) < expected:
                self.stats.parser_errors += 1
                health.valid = False
                return None

            arr = np.frombuffer(bytes(payload[:expected]),
                                dtype='<f4', count=points_num * 4)
            points = arr.reshape(points_num, 4).copy()

        except (struct.error, ValueError) as e:
            self.stats.parser_errors += 1
            health.valid = False
            logger.debug(f"Frame decode error: {e}")
            return None

        # Validate points
        if not np.all(np.isfinite(points)):
            # NaN/Inf rejection — Blueprint §11.1
            valid_mask = np.all(np.isfinite(points), axis=1)
            points = points[valid_mask]

        health.point_count = len(points)
        health.parse_timestamp = time.monotonic()
        health.valid = True
        health.packet_loss_cumulative = self.stats.sequence_gaps
        health.packet_reorder_count = self.stats.sequence_reorders
        health.duplicate_count = self.stats.duplicates

        self.stats.frames_parsed += 1
        self.stats.total_points += len(points)

        return points, health

    def _track_sequence(self, seq: int):
        """Track sequence numbers for gap/reorder/duplicate detection."""
        last = self.stats.last_sequence
        if last < 0:
            # First frame
            self.stats.last_sequence = seq
            return

        expected = last + 1
        if seq == last:
            self.stats.duplicates += 1
        elif seq < last:
            self.stats.sequence_reorders += 1
        elif seq > expected:
            gap = seq - expected
            self.stats.sequence_gaps += gap

        self.stats.last_sequence = seq

    def get_stats_string(self) -> str:
        s = self.stats
        return (f"parsed={s.frames_parsed} pts={s.total_points} "
                f"gaps={s.sequence_gaps} reorders={s.sequence_reorders} "
                f"dupes={s.duplicates} errors={s.parser_errors}")
