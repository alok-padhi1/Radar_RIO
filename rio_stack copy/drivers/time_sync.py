#!/usr/bin/env python3
"""
drivers/time_sync.py — Multi-clock synchronization and latency tracking.

Blueprint §35: Manages the clock domains across the RIO stack.
- Tracks latency across the pipeline: t_sensor -> t_rx -> t_decode -> t_estimator
- Detects burst timestamping where sensor hardware buffers packets and dumps them
  with identical host timestamps (§35.2)

Clock domains:
1. U300 clock (from firmware)
2. Cube clock (from RAW_IMU time_usec)
3. Jetson MONOTONIC (host time)
4. MAVLink target clock (synchronized via SYSTEM_TIME)
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)


@dataclass
class TimeSyncConfig:
    max_latency_ms: float = 200.0
    burst_threshold_ms: float = 2.0


class TimeSyncManager:
    """Tracks latency and synchronizes clocks across the stack."""

    def __init__(self, config: TimeSyncConfig = None):
        self.config = config or TimeSyncConfig()
        
        # State
        self.last_radar_rx_mono: float = 0.0
        self.last_imu_rx_mono: float = 0.0
        
        self.radar_burst_count: int = 0
        self.max_radar_latency: float = 0.0
        self.avg_radar_latency: float = 0.0
        self._radar_latency_samples: int = 0

    def check_radar_timestamp(self, sensor_t: float, rx_t: float) -> tuple[bool, float]:
        """Check radar timestamp for burst behavior and latency.
        
        Args:
            sensor_t: Timestamp provided by radar (if available, else rx_t)
            rx_t: Host monotonic time at UART read
            
        Returns:
            (is_valid, latency)
        """
        latency = rx_t - sensor_t
        if latency < 0:
            latency = 0.0
            
        # Update stats
        self.max_radar_latency = max(self.max_radar_latency, latency)
        self._radar_latency_samples += 1
        self.avg_radar_latency += (latency - self.avg_radar_latency) / self._radar_latency_samples
        
        # Check burst behavior (§35.2)
        # If host receives multiple packets in very short monotonic time,
        # it indicates USB/UART buffering, which destroys timing precision
        # unless sensor_t is used.
        dt_host = rx_t - self.last_radar_rx_mono
        if self.last_radar_rx_mono > 0 and dt_host < (self.config.burst_threshold_ms / 1000.0):
            self.radar_burst_count += 1
            if sensor_t == rx_t:
                # If we don't have a real sensor timestamp and we are bursting,
                # the timing is invalid.
                logger.warning(f"Radar burst detected with no sensor timestamp (dt={dt_host*1000:.1f}ms). Timing invalid.")
                self.last_radar_rx_mono = rx_t
                return False, latency
                
        self.last_radar_rx_mono = rx_t
        return True, latency

    def get_stats(self) -> dict:
        """Get time sync statistics for logging."""
        return {
            'radar_burst_count': self.radar_burst_count,
            'max_radar_latency_ms': self.max_radar_latency * 1000.0,
            'avg_radar_latency_ms': self.avg_radar_latency * 1000.0,
        }
