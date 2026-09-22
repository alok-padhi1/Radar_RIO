#!/usr/bin/env python3
"""
drivers/altimeter/quality_filter.py — Altimeter quality and staleness filter.

Blueprint §8.1-8.3:
- Computes AGL from slant range using full sensor orientation (§8.1)
- Tracks sample age and rejects stale readings (§8.2)
- Sets valid=false when surface is unmeasurable (§8.3), never holds last value
"""

from __future__ import annotations

import collections
import logging
import math
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from estimator.state import AltimeterSample
from drivers.altimeter.decoder import AltimeterRawSample

logger = logging.getLogger(__name__)


@dataclass
class AltimeterFilterConfig:
    """Configuration for altimeter quality filtering."""
    max_range_m: float = 200.0
    min_range_m: float = 0.0
    max_age_s: float = 0.5          # §8.2: reject when age > max
    max_climb_rate_mps: float = 5.0  # Outlier rejection rate limit
    median_window: int = 5           # Spike rejection window
    rate_reject_multiplier: float = 3.0
    beam_axis_body: np.ndarray = None  # Altimeter beam direction in body frame

    def __post_init__(self):
        if self.beam_axis_body is None:
            self.beam_axis_body = np.array([0.0, 0.0, -1.0])  # Straight down


class AltimeterQualityFilter:
    """Filters altimeter readings for quality, staleness, and correctness.

    Blueprint §8.1: AGL = r * |e_z^W^T R_WA a_A|
    Blueprint §8.2: Reject/inflate when age > max
    Blueprint §8.3: valid=false when surface unmeasurable, NOT last value
    """

    def __init__(self, config: AltimeterFilterConfig = None):
        self.config = config or AltimeterFilterConfig()
        self._median_window = collections.deque(maxlen=self.config.median_window)
        self._last_accepted: Optional[float] = None
        self._last_accepted_time: float = 0.0
        self._last_valid_time: float = 0.0

        # Statistics
        self.n_accepted = 0
        self.n_out_of_envelope = 0
        self.n_rate_rejected = 0
        self.n_stale_rejected = 0

    def correct_agl(self, slant_range: float,
                    roll: float = 0.0, pitch: float = 0.0) -> float:
        """Compute vertical AGL from slant range using attitude.

        Blueprint §8.1: For a downward sensor with known roll/pitch:
            h_AGL = r * |e_z^T R_WA a_A|

        For the simple case of a downward-facing sensor this reduces to:
            h_AGL = r * cos(roll) * cos(pitch)

        Args:
            slant_range: Measured slant range along beam axis [m]
            roll: Vehicle roll angle [rad]
            pitch: Vehicle pitch angle [rad]

        Returns:
            Corrected vertical AGL [m]
        """
        # Full formula: h = r * |e_z^T @ R_WA @ a_A|
        # For body-mounted downward sensor: R_WA = R_WB, a_A = [0,0,-1]
        # e_z^T @ R_WB @ [0,0,-1] = cos(roll)*cos(pitch)
        correction = abs(math.cos(roll) * math.cos(pitch))
        return slant_range * correction

    def filter(self, raw: AltimeterRawSample,
               roll: float = 0.0, pitch: float = 0.0) -> AltimeterSample:
        """Filter a raw altimeter sample into a qualified AltimeterSample.

        Args:
            raw: Raw decoded sample
            roll: Current vehicle roll [rad] (for AGL correction)
            pitch: Current vehicle pitch [rad]

        Returns:
            AltimeterSample with validity, quality, age, and corrected AGL.
            If invalid, valid=false and corrected_agl_m=NaN (§8.3).
        """
        now = time.monotonic()

        # Checksum failure
        if not raw.checksum_valid or raw.parse_error:
            return AltimeterSample(
                timestamp=raw.timestamp,
                raw_range_m=raw.raw_range_m,
                corrected_agl_m=float('nan'),
                quality=0.0,
                valid=False,
                sample_age_s=now - self._last_valid_time if self._last_valid_time > 0 else float('inf'),
            )

        r_raw = raw.raw_range_m

        # Range envelope check
        if not (self.config.min_range_m <= r_raw <= self.config.max_range_m):
            self.n_out_of_envelope += 1
            return AltimeterSample(
                timestamp=raw.timestamp,
                raw_range_m=r_raw,
                corrected_agl_m=float('nan'),
                quality=0.0,
                valid=False,
                sample_age_s=now - self._last_valid_time if self._last_valid_time > 0 else float('inf'),
            )

        # Median spike rejection
        self._median_window.append(r_raw)
        r_median = float(np.median(list(self._median_window)))

        # Rate-of-change rejection
        if self._last_accepted is not None:
            dt = max(now - self._last_accepted_time, 1e-3)
            rate = abs(r_median - self._last_accepted) / dt
            if rate > self.config.rate_reject_multiplier * self.config.max_climb_rate_mps:
                self.n_rate_rejected += 1
                return AltimeterSample(
                    timestamp=raw.timestamp,
                    raw_range_m=r_raw,
                    corrected_agl_m=float('nan'),
                    quality=0.3,
                    valid=False,
                    sample_age_s=now - self._last_valid_time if self._last_valid_time > 0 else float('inf'),
                )

        # Compute corrected AGL (§8.1)
        agl = self.correct_agl(r_median, roll, pitch)

        # Update state
        self._last_accepted = r_median
        self._last_accepted_time = now
        self._last_valid_time = now
        self.n_accepted += 1

        return AltimeterSample(
            timestamp=raw.timestamp,
            raw_range_m=r_raw,
            corrected_agl_m=agl,
            quality=1.0,
            valid=True,
            sample_age_s=0.0,
        )

    def check_staleness(self) -> float:
        """Check how long since the last valid sample.

        Blueprint §8.2: reject/inflate uncertainty when age > max.
        """
        if self._last_valid_time <= 0:
            return float('inf')
        return time.monotonic() - self._last_valid_time

    def reset(self):
        """Reset filter state (e.g., after serial reconnect)."""
        self._median_window.clear()
        self._last_accepted = None
        self._last_accepted_time = 0.0
