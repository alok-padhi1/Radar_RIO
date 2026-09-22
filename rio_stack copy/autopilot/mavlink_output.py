#!/usr/bin/env python3
"""
autopilot/mavlink_output.py — Clean estimator output layer.

Blueprint §22, §31: Separates the estimator output logic from the lower-level
MAVLink transport bridge.

Stage 1 implementation (§22.1): Output velocity only, with conservative
covariance. The MAVLink bridge will map this to ODOMETRY or VISION_SPEED_ESTIMATE
depending on the autopilot type.
"""

from __future__ import annotations

import logging
import socket
import struct
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from estimator.state import NavigationState

logger = logging.getLogger(__name__)

# Extended RIO packet (backward compatible with mavlink_bridge.py)
# struct '<dfffIfffIBf' (45 bytes)
# t, vx, vy, vz, n_inliers, cxx, cyy, czz, n_total, flags, cond
FORWARD_PKT = struct.Struct('<dfffIfffIBf')


@dataclass
class OutputConfig:
    mavlink_bridge_ip: str = "127.0.0.1"
    mavlink_bridge_port: int = 5006
    # Safety gates
    require_navigation_valid: bool = True
    # Conservative covariance overrides (Stage 1, §22.1)
    # If set, these override the estimator's covariance until Stage 2
    override_velocity_cov_mps2: Optional[float] = 0.05
    min_velocity_cov_mps2: float = 0.01


class MAVLinkOutput:
    """Manages publishing navigation states to the MAVLink bridge.

    Blueprint §32: The output layer must consume NavigationState and
    refuse to output when navigation_valid == false.
    """

    def __init__(self, config: OutputConfig = None):
        self.config = config or OutputConfig()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dest = (self.config.mavlink_bridge_ip, self.config.mavlink_bridge_port)
        
        self.n_published = 0
        self.n_rejected = 0

    def publish(self, state: NavigationState):
        """Publish the navigation state to the MAVLink bridge."""
        # Hard gate (§32)
        if self.config.require_navigation_valid and not state.navigation_valid:
            self.n_rejected += 1
            return

        # Prepare velocity covariance
        cxx = cyy = czz = self.config.min_velocity_cov_mps2
        if self.config.override_velocity_cov_mps2 is not None:
            cxx = cyy = czz = self.config.override_velocity_cov_mps2
        elif state.velocity_covariance is not None:
            # Extract diagonal elements if available
            c = state.velocity_covariance
            if c.shape == (3, 3):
                cxx, cyy, czz = c[0,0], c[1,1], c[2,2]
            elif len(c.shape) == 1 and len(c) >= 6:
                cxx, cyy, czz = c[0], c[3], c[5] # assuming standard upper-tri packing
                
            # Apply floor
            cxx = max(cxx, self.config.min_velocity_cov_mps2)
            cyy = max(cyy, self.config.min_velocity_cov_mps2)
            czz = max(czz, self.config.min_velocity_cov_mps2)

        # Build packet (using legacy struct for compatibility with mavlink_bridge.py)
        # We use state.timestamp as the frame time
        flags = 0
        cond = 0.0
        n_inliers = 0
        n_total = 0
        
        if state.rio and state.rio.valid:
            n_inliers = state.rio.n_static_points
            n_total = state.rio.n_radar_points

        vx, vy, vz = state.velocity[0], state.velocity[1], state.velocity[2]

        pkt = FORWARD_PKT.pack(
            state.timestamp,
            float(vx), float(vy), float(vz),
            int(n_inliers),
            float(cxx), float(cyy), float(czz),
            int(n_total),
            int(flags),
            float(cond)
        )

        try:
            self.sock.sendto(pkt, self.dest)
            self.n_published += 1
        except OSError as e:
            logger.error(f"Failed to publish to MAVLink bridge: {e}")
            
    def get_stats(self) -> dict:
        return {
            'published': self.n_published,
            'rejected': self.n_rejected
        }
