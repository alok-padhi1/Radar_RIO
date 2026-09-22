#!/usr/bin/env python3
"""
drivers/u300/frame_validator.py — U300 frame validation.

Blueprint §5.1: Validates decoded U300 frames for consistency,
sanity, and completeness before passing to the estimator.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

import numpy as np

from drivers.u300.decoder import FrameHealth

logger = logging.getLogger(__name__)


@dataclass
class ValidationConfig:
    """Frame validation thresholds."""
    max_point_count: int = 500      # Sanity limit for single frame
    min_point_count: int = 0        # Minimum to be valid (0 = empty frames OK)
    max_frame_gap: int = 10         # Maximum sequence gap before warning
    max_timestamp_jump_s: float = 1.0  # Maximum timestamp jump
    max_range_m: float = 350.0      # Maximum valid range
    min_range_m: float = 0.1        # Minimum valid range


class FrameValidator:
    """Validates U300 frames for consistency and sanity.

    Checks:
    - Point count within bounds
    - No NaN/Inf values
    - Range values physically plausible
    - Sequence number continuity
    - Timestamp monotonicity
    """

    def __init__(self, config: ValidationConfig = None):
        self.config = config or ValidationConfig()
        self._last_timestamp: float = 0.0
        self._last_sequence: int = -1
        self.n_validated: int = 0
        self.n_rejected: int = 0
        self.rejection_reasons: dict[str, int] = {}

    def validate(self, points: np.ndarray,
                 health: FrameHealth) -> tuple[bool, list[str]]:
        """Validate a decoded frame.

        Args:
            points: (N, 4) array [x, y, z, v]
            health: Frame health metadata from decoder

        Returns:
            (valid, reasons) — True if frame is usable, list of warning reasons
        """
        reasons = []
        valid = True

        # Check decoder health
        if not health.valid:
            reasons.append("decoder_invalid")
            valid = False

        # Point count bounds
        if len(points) > self.config.max_point_count:
            reasons.append(f"point_count_too_high_{len(points)}")
            valid = False

        # Check for NaN/Inf
        if len(points) > 0 and not np.all(np.isfinite(points)):
            n_bad = np.sum(~np.all(np.isfinite(points), axis=1))
            reasons.append(f"nan_inf_points_{n_bad}")
            # Don't invalidate — let downstream handle NaN filtering

        # Range sanity
        if len(points) > 0:
            xyz = points[:, :3]
            ranges = np.linalg.norm(xyz, axis=1)
            if np.any(ranges > self.config.max_range_m):
                reasons.append("points_beyond_max_range")
            if np.any((ranges > 0) & (ranges < self.config.min_range_m)):
                reasons.append("points_below_min_range")

        # Sequence continuity
        if self._last_sequence >= 0 and health.sequence > 0:
            gap = health.sequence - self._last_sequence - 1
            if gap > 0:
                reasons.append(f"sequence_gap_{gap}")
                if gap > self.config.max_frame_gap:
                    reasons.append("large_sequence_gap")

        # Timestamp monotonicity (§35.2)
        if health.parse_timestamp > 0 and self._last_timestamp > 0:
            dt = health.parse_timestamp - self._last_timestamp
            if dt < 0:
                reasons.append("timestamp_non_monotonic")
            elif dt > self.config.max_timestamp_jump_s:
                reasons.append(f"timestamp_jump_{dt:.3f}s")

        # Update state
        if health.sequence > 0:
            self._last_sequence = health.sequence
        if health.parse_timestamp > 0:
            self._last_timestamp = health.parse_timestamp

        if valid:
            self.n_validated += 1
        else:
            self.n_rejected += 1
            for r in reasons:
                self.rejection_reasons[r] = self.rejection_reasons.get(r, 0) + 1

        return valid, reasons
