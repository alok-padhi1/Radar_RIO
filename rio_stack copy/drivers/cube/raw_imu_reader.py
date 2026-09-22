#!/usr/bin/env python3
"""
drivers/cube/raw_imu_reader.py — Raw Cube Orange+ IMU reader.

Blueprint §7: RIO must receive raw inertial measurements (accelerometer,
gyroscope, timestamp) at the highest valid rate. Do NOT use
LOCAL_POSITION_NED or autopilot-derived velocity/attitude as primary input.

Blueprint §7.2: Never allow RIO→Cube EKF→ATTITUDE→RIO feedback loops.
The desired direction is: raw Cube IMU → RIO → external ODOMETRY → Cube EKF.

Blueprint §7.3: Every IMU sample must retain the closest available
measurement timestamp. Avoid time.monotonic() unless the timestamp
conversion from the source clock is explicitly documented.
"""

from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from typing import Optional, Callable

import numpy as np
from pymavlink import mavutil

from estimator.state import IMUSample, IMUAttitude

logger = logging.getLogger(__name__)

# Wire formats
# Raw IMU packet: gyro + accel + timestamp
RAW_IMU_PKT = struct.Struct('<dffffffd')  # 56 bytes:
# t_sensor, ax, ay, az, gx, gy, gz, t_mono

# Fused attitude packet (REFERENCE/TELEMETRY only, §29)
# Matches existing imu_bridge.py IMU_PKT format for backward compat
ATTITUDE_PKT = struct.Struct('<dffffffBfdd')  # 53 bytes


class RawIMUReader:
    """Reads raw IMU data from the Cube Orange+ flight controller.

    Blueprint §7.1: Primary input for RIO is raw gyro + accel + timestamp.

    This reader requests RAW_IMU or SCALED_IMU from the flight controller
    and outputs IMUSample objects with sensor timestamps.

    The fused ATTITUDE stream is separately available as TELEMETRY (§29)
    for diagnostics and log comparison, but is NOT used as RIO input.
    """

    def __init__(self, port: str = '/dev/ttyACM0', baud: int = 115200,
                 rate_hz: int = 100, dest_ports: list[int] = None):
        self.port = port
        self.baud = baud
        self.rate_hz = rate_hz
        self.dest_ports = dest_ports or [5020, 5021]

        # State
        self._lock = threading.Lock()
        self._latest_raw: Optional[IMUSample] = None
        self._latest_attitude: Optional[IMUAttitude] = None
        self._latest_gps: Optional[dict] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._callbacks: list[Callable[[IMUSample], None]] = []

        # Health tracking
        self.sample_count = 0
        self.attitude_count = 0
        self.last_sample_time = 0.0
        self.last_gyro_mag = 0.0
        self.last_accel_mag = 0.0
        self._timestamps_monotonic = True
        self._last_sensor_ts = 0.0

    @property
    def latest_raw(self) -> Optional[IMUSample]:
        with self._lock:
            return self._latest_raw

    @property
    def latest_attitude(self) -> Optional[IMUAttitude]:
        """REFERENCE/TELEMETRY only — not for RIO input (§29)."""
        with self._lock:
            return self._latest_attitude

    @property
    def latest_gps(self) -> Optional[dict]:
        """Latest GPS data for logging and analysis."""
        with self._lock:
            if self._latest_gps:
                gps = self._latest_gps.copy()
                self._latest_gps = None
                return gps
            return None

    def register_callback(self, cb: Callable[[IMUSample], None]):
        self._callbacks.append(cb)

    def _process_raw_imu(self, msg) -> IMUSample:
        """Convert a MAVLink RAW_IMU / SCALED_IMU message to IMUSample.

        Blueprint §7.3: Use sensor timestamp, not time.monotonic().
        """
        # RAW_IMU provides time_usec (sensor time in microseconds)
        t_sensor = getattr(msg, 'time_usec', 0) / 1e6
        t_mono = time.monotonic()

        # RAW_IMU values are in milli-g for accel, milli-deg/s for gyro
        # SCALED_IMU provides values in mg and mrad/s
        msg_type = msg.get_type()

        if msg_type == 'RAW_IMU':
            # Convert from raw ADC-like values — depends on sensor range
            # Convert from FRD (MAVLink standard) to FLU (ROS standard)
            ax = msg.xacc / 1000.0 * 9.80665  # mg → m/s²
            ay = -(msg.yacc / 1000.0 * 9.80665)
            az = -(msg.zacc / 1000.0 * 9.80665)
            gx = np.radians(msg.xgyro / 1000.0)  # mdeg/s → rad/s
            gy = -np.radians(msg.ygyro / 1000.0)
            gz = -np.radians(msg.zgyro / 1000.0)
        elif msg_type == 'SCALED_IMU2' or msg_type == 'SCALED_IMU':
            # Convert from FRD to FLU
            ax = msg.xacc / 1000.0 * 9.80665  # mG → m/s²
            ay = -(msg.yacc / 1000.0 * 9.80665)
            az = -(msg.zacc / 1000.0 * 9.80665)
            gx = msg.xgyro / 1000.0  # mrad/s → rad/s
            gy = -(msg.ygyro / 1000.0)
            gz = -(msg.zgyro / 1000.0)
        else:
            # Fallback
            ax = ay = az = gx = gy = gz = 0.0

        # Track timestamp monotonicity
        if t_sensor > 0 and t_sensor < self._last_sensor_ts:
            self._timestamps_monotonic = False
            logger.warning(f"IMU timestamp non-monotonic: {t_sensor:.6f} < {self._last_sensor_ts:.6f}")
        self._last_sensor_ts = t_sensor

        sample = IMUSample(
            timestamp=t_sensor if t_sensor > 0 else t_mono,
            accel_x=ax, accel_y=ay, accel_z=az,
            gyro_x=gx, gyro_y=gy, gyro_z=gz,
            valid=True,
            source=msg_type,
        )

        # Update health
        self.last_gyro_mag = np.linalg.norm([gx, gy, gz])
        self.last_accel_mag = np.linalg.norm([ax, ay, az])
        self.last_sample_time = t_mono
        self.sample_count += 1

        return sample

    def _process_attitude(self, msg) -> IMUAttitude:
        """Convert ATTITUDE message to IMUAttitude — TELEMETRY only (§29)."""
        return IMUAttitude(
            timestamp=time.monotonic(),
            roll=msg.roll,
            pitch=msg.pitch,
            yaw=msg.yaw,
            rollspeed=msg.rollspeed,
            pitchspeed=msg.pitchspeed,
            yawspeed=msg.yawspeed,
            source="TELEMETRY",  # Explicitly marked per §29
        )

    def get_health(self) -> dict:
        """Get IMU health report for the health manager."""
        now = time.monotonic()
        dt = now - self.last_sample_time if self.last_sample_time > 0 else float('inf')
        return {
            'sample_count': self.sample_count,
            'sample_rate_hz': self.sample_count / max(now - self.last_sample_time, 1e-3) if self.sample_count > 0 else 0,
            'last_sample_age_s': dt,
            'gyro_mag_rads': self.last_gyro_mag,
            'accel_mag_ms2': self.last_accel_mag,
            'timestamps_monotonic': self._timestamps_monotonic,
            'stale': dt > 1.0,
        }

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logger.info(f"RawIMUReader connected to {self.port} @ {self.baud}")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run(self):
        try:
            master = mavutil.mavlink_connection(self.port, baud=self.baud)
            master.mav.request_data_stream_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, self.rate_hz, 1
            )
        except Exception as e:
            logger.error(f"Failed to connect to FC on {self.port}: {e}")
            return

        while self._running:
            msg = master.recv_match(blocking=True, timeout=0.1)
            if not msg: continue

            msg_type = msg.get_type()
            
            if msg_type in ['RAW_IMU', 'SCALED_IMU', 'SCALED_IMU2']:
                sample = self._process_raw_imu(msg)
                with self._lock:
                    self._latest_raw = sample
                for cb in self._callbacks:
                    cb(sample)
            elif msg_type == 'GLOBAL_POSITION_INT':
                with self._lock:
                    self._latest_gps = {
                        "lat": msg.lat / 1e7,
                        "lon": msg.lon / 1e7,
                        "alt_msl_m": msg.alt / 1000.0,
                        "alt_rel_m": msg.relative_alt / 1000.0,
                        "vx_mps": msg.vx / 100.0,
                        "vy_mps": msg.vy / 100.0,
                        "vz_mps": msg.vz / 100.0,
                        "hdg_deg": msg.hdg / 100.0,
                    }
