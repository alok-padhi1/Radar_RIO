#!/usr/bin/env python3
"""
autopilot/ekf_interface.py — EKF fusion verification.

Blueprint Phase 4-5: This module provides utilities to verify that the
autopilot EKF is actually fusing the RIO data properly, rather than
just receiving it.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)


@dataclass
class EKFFusionStatus:
    """Status of EKF fusion of external vision data."""
    timestamp: float
    using_vision_velocity: bool = False
    using_vision_position: bool = False
    vision_velocity_innovation_rms: float = 0.0
    vision_position_innovation_rms: float = 0.0
    healthy: bool = False


class EKFInterface:
    """Monitors autopilot EKF status via MAVLink telemetry.
    
    This acts as a diagnostic monitor to ensure the RIO output is
    being accepted by the autopilot's EKF (e.g., PX4 EKF2 or ArduPilot EKF3).
    """

    def __init__(self):
        self._last_status = EKFFusionStatus(timestamp=0.0)

    def process_estimator_status(self, msg) -> None:
        """Process a MAVLink ESTIMATOR_STATUS message.
        
        This requires intercepting telemetry from the autopilot,
        which currently happens in a separate logging tool, but
        is structurally defined here for future integration.
        """
        pass
        
    @property
    def last_status(self) -> EKFFusionStatus:
        return self._last_status
