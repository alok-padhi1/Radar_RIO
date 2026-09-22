#!/usr/bin/env python3
"""
drivers/altimeter/decoder.py — Altimeter serial decoder.

Blueprint §8, §30: Extracts range measurements from the belly altimeter
(U200A-style interface). Performs checksum validation and basic range
envelope checks.

Extracted from src/altimeter_bridge.py to create a clean driver boundary.
"""

from __future__ import annotations

import logging
import struct
import time
from dataclasses import dataclass
from typing import Optional

try:
    import serial
except ImportError:  # pragma: no cover
    # Provide a minimal stub for serial.Serial for test environments without pyserial.
    class _SerialStub:
        """Placeholder Serial class with no functionality, used only for type hints.

        The real driver expects a Serial instance with read/write methods, but unit
        tests mock these interactions. Providing a stub prevents import errors when
        the pyserial package is unavailable.
        """
        pass

    serial = type('serial', (), {'Serial': _SerialStub})


logger = logging.getLogger(__name__)

# U200A frame: [0xEB, 0x90, frame_id, dist_h, dist_l, checksum]
FRAME_HEADER = bytes([0xEB, 0x90])
FRAME_LENGTH = 6


@dataclass
class AltimeterRawSample:
    """Raw altimeter reading before quality filtering."""
    timestamp: float           # time.monotonic() at parse
    raw_range_m: float         # Slant range along beam axis [m]
    frame_id: int = 0          # Frame identifier byte
    checksum_valid: bool = True
    parse_error: bool = False


class AltimeterDecoder:
    """Decodes serial frames from a U200A-style belly altimeter.

    Frame format: [0xEB, 0x90, frame_id, dist_high, dist_low, checksum]
    Distance: big-endian uint16, in centimetres → metres.
    Checksum: sum of first 5 bytes, masked to 8 bits.
    """

    def __init__(self, min_range_m: float = 0.0, max_range_m: float = 200.0):
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m
        self.frames_decoded = 0
        self.checksum_errors = 0
        self.range_errors = 0

    def decode_frame(self, frame_bytes: bytearray) -> Optional[AltimeterRawSample]:
        """Decode a 6-byte altimeter frame.

        Args:
            frame_bytes: 6-byte frame [0xEB, 0x90, id, dist_h, dist_l, chksum]

        Returns:
            AltimeterRawSample or None on error
        """
        if len(frame_bytes) != FRAME_LENGTH:
            return AltimeterRawSample(
                timestamp=time.monotonic(), raw_range_m=0.0,
                parse_error=True, checksum_valid=False,
            )

        # Verify header
        if frame_bytes[0] != 0xEB or frame_bytes[1] != 0x90:
            return None

        # Verify checksum
        chk = sum(frame_bytes[:5]) & 0xFF
        if chk != frame_bytes[5]:
            self.checksum_errors += 1
            return AltimeterRawSample(
                timestamp=time.monotonic(), raw_range_m=0.0,
                frame_id=frame_bytes[2], checksum_valid=False,
            )

        # Parse distance (big-endian uint16, centimetres)
        dist_raw = struct.unpack('>H', frame_bytes[3:5])[0]
        range_m = dist_raw / 100.0

        # Range envelope check
        if not (self.min_range_m <= range_m <= self.max_range_m):
            self.range_errors += 1

        self.frames_decoded += 1
        return AltimeterRawSample(
            timestamp=time.monotonic(),
            raw_range_m=range_m,
            frame_id=frame_bytes[2],
            checksum_valid=True,
        )

    def read_from_serial(self, ser: serial.Serial) -> Optional[AltimeterRawSample]:
        """Read and decode one frame from a serial port.

        Searches for the 0xEB 0x90 header, then reads remaining 4 bytes.
        """
        # Find header
        b = ser.read(1)
        if not b or b[0] != 0xEB:
            return None
        b2 = ser.read(1)
        if not b2 or b2[0] != 0x90:
            return None

        # Read remaining 4 bytes
        rem = ser.read(4)
        if len(rem) != 4:
            return None

        frame = bytearray([0xEB, 0x90]) + bytearray(rem)
        return self.decode_frame(frame)
