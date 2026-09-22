#!/usr/bin/env python3
"""
estimator/rio_interface.py — IPC Bridge to the Native C++ Estimator.

Blueprint Phase 4: Integrates the Python hardware layer with the
native HKUST RIO C++ process using ZeroMQ for lightweight IPC.
"""

from __future__ import annotations

import json
import logging
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

# We conditionally import zmq so the stack doesn't crash if it's missing,
# but it is required for actual C++ IPC.
try:
    import zmq
except ImportError:
    zmq = None

from estimator.state import IMUSample, RadarMeasurement, NavigationState, RIOState

logger = logging.getLogger(__name__)


@dataclass
class RIOInterfaceConfig:
    # ZMQ Endpoints
    # PUB from Python, SUB from C++
    sensor_endpoint: str = "ipc:///tmp/rio_sensor_in"
    # SUB in Python, PUB from C++
    state_endpoint: str = "ipc:///tmp/rio_state_out"
    
    # Timeouts
    receive_timeout_ms: int = 100


class RIOInterface:
    """ZMQ bridge connecting the Python hardware layer to the C++ core."""

    def __init__(self, config: RIOInterfaceConfig = None):
        self.config = config or RIOInterfaceConfig()
        self._context = None
        self._pub_socket = None
        self._sub_socket = None
        self._running = False
        self._listen_thread = None
        self._latest_state: Optional[NavigationState] = None
        self._state_lock = threading.Lock()
        
        # Binary structs for efficient IPC
        # IMU: [type=1, t, ax,ay,az, gx,gy,gz] -> B d f f f f f f
        self._imu_struct = struct.Struct('<B d f f f f f f')
        
    def start(self):
        """Initialize ZMQ sockets and start listening thread."""
        if zmq is None:
            logger.error("pyzmq is not installed. RIO IPC bridge cannot start.")
            return

        try:
            self._context = zmq.Context()
            
            # Publisher socket (Python -> C++)
            self._pub_socket = self._context.socket(zmq.PUB)
            self._pub_socket.bind(self.config.sensor_endpoint)
            
            # Subscriber socket (C++ -> Python)
            self._sub_socket = self._context.socket(zmq.SUB)
            self._sub_socket.connect(self.config.state_endpoint)
            self._sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
            self._sub_socket.setsockopt(zmq.RCVTIMEO, self.config.receive_timeout_ms)
            
            self._running = True
            self._listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
            self._listen_thread.start()
            
            logger.info(f"RIO Interface started on {self.config.sensor_endpoint}")
            
        except zmq.ZMQError as e:
            logger.error(f"Failed to bind ZMQ sockets: {e}")
            self._running = False

    def stop(self):
        """Cleanly shutdown the IPC bridge."""
        self._running = False
        if self._listen_thread:
            self._listen_thread.join(timeout=1.0)
            
        if self._pub_socket:
            self._pub_socket.close()
        if self._sub_socket:
            self._sub_socket.close()
        if self._context:
            self._context.term()

    def send_imu(self, sample: IMUSample):
        """Push IMU sample to C++ core."""
        if not self._running or self._pub_socket is None:
            return
            
        # Pack into binary for speed
        payload = self._imu_struct.pack(
            1, # Type ID 1 = IMU
            sample.timestamp,
            float(sample.accel_x), float(sample.accel_y), float(sample.accel_z),
            float(sample.gyro_x), float(sample.gyro_y), float(sample.gyro_z)
        )
        self._pub_socket.send(payload)

    def send_radar(self, points: list[RadarMeasurement]):
        """Push radar points to C++ core."""
        if not self._running or self._pub_socket is None or not points:
            return
            
        # For variable length arrays, JSON/MsgPack is safer than fixed struct
        # In a highly optimized pipeline, this would be flatbuffers or capnproto
        data = {
            "type": 2, # Type ID 2 = Radar
            "timestamp": points[0].timestamp,
            "points": [
                [p.range_m, p.azimuth_rad, p.elevation_rad, p.doppler_mps] 
                for p in points
            ]
        }
        self._pub_socket.send_json(data)

    def _listen_loop(self):
        """Background thread receiving state from C++."""
        while self._running:
            try:
                # We expect JSON for the complex navigation state
                msg = self._sub_socket.recv_json()
                
                # Parse the state
                state = NavigationState(
                    timestamp=msg.get("timestamp", time.monotonic()),
                    position=np.array(msg.get("position", [0,0,0])),
                    velocity=np.array(msg.get("velocity", [0,0,0])),
                    orientation_quat=np.array(msg.get("orientation", [1,0,0,0])),
                    navigation_valid=msg.get("valid", False),
                    rio=RIOState(
                        timestamp=msg.get("timestamp", time.monotonic()),
                        valid=msg.get("rio_valid", False),
                        n_static_points=msg.get("n_static", 0),
                        n_radar_points=msg.get("n_total", 0),
                        fitness_score=msg.get("fitness", 0.0),
                        condition_number=msg.get("cond", float('inf'))
                    )
                )
                
                with self._state_lock:
                    self._latest_state = state
                    
            except zmq.Again:
                # Timeout, just loop
                pass
            except Exception as e:
                logger.debug(f"Error parsing state from C++: {e}")
                
    def get_latest_state(self) -> Optional[NavigationState]:
        """Fetch the latest state received from C++."""
        with self._state_lock:
            return self._latest_state
