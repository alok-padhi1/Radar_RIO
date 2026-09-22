#!/usr/bin/env python3
"""
estimator/logger.py — Structured flight logger.

Blueprint §36: Records full flight data in a structured format (JSONL).
Replaces the legacy logging approach.

Contains ALL required fields for replay, forensics, and health analysis:
- Sensor data: radar timestamps, point counts, IMU rates, altimeter quality
- RIO state: tracks, residuals, optimizer status, pose, velocity, covariance
- SLAM state: keyframes, correspondences, fitness, eigenvalues, observability
- Health: all sensor flags, mode, quality reasons, latency
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from dataclasses import asdict, is_dataclass
from typing import Any

from estimator.state import NavigationState, RadarFrame, IMUSample, AltimeterSample

logger = logging.getLogger(__name__)


def _custom_serializer(obj: Any) -> Any:
    """Helper to serialize numpy arrays and dataclasses for JSON."""
    if hasattr(obj, 'tolist'):  # Numpy arrays
        return obj.tolist()
    if is_dataclass(obj):
        return asdict(obj)
    if hasattr(obj, 'to_dict'):
        return obj.to_dict()
    if hasattr(obj, 'value'): # Enums
        return obj.value
    raise TypeError(f"Type {type(obj)} not serializable")


class FlightLogger:
    """Structured flight logger writing JSONL."""

    def __init__(self, log_dir: str = "logs"):
        self.log_dir = log_dir
        self.is_running = False
        self._file = None
        self._lock = threading.Lock()
        
        # Stats
        self.frames_logged = 0
        self.bytes_written = 0

    def start(self, run_name: str = None):
        """Start the logger."""
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir, exist_ok=True)
            
        if run_name is None:
            time_str = time.strftime("%Y%m%d_%H%M%S")
            run_name = f"run_{time_str}"
            
        log_path = os.path.join(self.log_dir, f"{run_name}.jsonl")
        
        with self._lock:
            self._file = open(log_path, 'w')
            self.is_running = True
            
        logger.info(f"Flight logger started: {log_path}")
        
        # Write header
        self.log_event("SESSION_START", {
            "time_utc": time.time(),
            "time_mono": time.monotonic(),
            "blueprint_version": "2026-09"
        })

    def stop(self):
        """Stop the logger."""
        with self._lock:
            self.is_running = False
            if self._file:
                self.log_event("SESSION_END", {"time_mono": time.monotonic()})
                self._file.close()
                self._file = None
        logger.info(f"Flight logger stopped. Wrote {self.frames_logged} events, {self.bytes_written/1024/1024:.2f} MB")

    def _write_jsonl(self, record: dict):
        if not self.is_running or self._file is None:
            return
            
        try:
            line = json.dumps(record, default=_custom_serializer) + "\n"
            with self._lock:
                self._file.write(line)
                self._file.flush()  # Crucial for crash survivability
                self.bytes_written += len(line)
                self.frames_logged += 1
        except Exception as e:
            logger.error(f"Logger write error: {e}")

    def log_event(self, event_type: str, data: dict):
        """Log a generic event."""
        record = {
            "t": time.monotonic(),
            "type": event_type,
            "data": data
        }
        self._write_jsonl(record)

    def log_navigation_state(self, state: NavigationState):
        """Log the complete navigation state."""
        self.log_event("NAV_STATE", state.to_dict())

    def log_radar_frame(self, frame: RadarFrame, health_stats: dict = None):
        """Log radar frame metadata (not full point cloud)."""
        data = {
            "timestamp": frame.timestamp,
            "rx_timestamp": frame.rx_timestamp,
            "frame_id": frame.frame_id,
            "point_count": frame.point_count,
            "valid": frame.valid
        }
        if health_stats:
            data.update(health_stats)
        self.log_event("RADAR_FRAME", data)

    def log_imu_stats(self, stats: dict):
        """Log periodic IMU health stats."""
        self.log_event("IMU_STATS", stats)
        
    def log_health_report(self, report: dict):
        """Log the health manager's full report."""
        self.log_event("HEALTH_REPORT", report)

    def log_gps(self, gps_data: dict):
        """Log GPS data from flight controller."""
        self.log_event("GPS", gps_data)
