#!/usr/bin/env python3
"""
drivers/u300/adapter.py — U300→RIO measurement adapter.

Blueprint §5.2, §5.3: Converts raw U300 (x,y,z,v) points into canonical
RadarMeasurement objects with:
- Proper polar coordinates using atan2 (never asin)
- Sign-corrected Doppler velocity
- Per-point uncertainty from the calibrated noise model
- Preserved sensor timestamps

This is the adaptation boundary (§4.1):
    U300 → U300→RIO measurement adapter → HKUST RIO
"""

from __future__ import annotations

import logging
import math
from typing import Optional

import numpy as np
import yaml

from estimator.state import RadarMeasurement, RadarFrame
from estimator.uncertainty import (
    NoiseModel, polar_from_cartesian_batch, propagate_covariance_batch,
)

logger = logging.getLogger(__name__)


class U300Adapter:
    """Converts raw U300 points to canonical RadarMeasurement format.

    Blueprint §5.2: Canonical measurement includes timestamp, x_r, y_r, z_r,
    range, azimuth, elevation, doppler, quality, uncertainty.

    Blueprint §5.3: Doppler sign is applied EXACTLY ONCE here, based on
    experimentally verified convention from config/u300_doppler_convention.yaml.
    """

    def __init__(self, noise_model: NoiseModel = None,
                 doppler_sign: int = -1,
                 max_range_m: float = 200.0,
                 min_range_m: float = 0.3,
                 leakage_radius_m: float = 0.35,
                 fov_azimuth_rad: float = None,
                 fov_elevation_rad: float = None):
        """
        Args:
            noise_model: Measurement noise parameters (§10.3)
            doppler_sign: Sign correction for Doppler convention (§5.3)
            max_range_m: Maximum valid range [m]
            min_range_m: Minimum valid range [m] — below this is self-return
            leakage_radius_m: TX/RX coupling artifact zone [m]
            fov_azimuth_rad: Half-angle azimuth FOV [rad] (None = no check)
            fov_elevation_rad: Half-angle elevation FOV [rad] (None = no check)
        """
        self.noise = noise_model or NoiseModel()
        self.doppler_sign = doppler_sign
        self.max_range_m = max_range_m
        self.min_range_m = min_range_m
        self.leakage_radius_m = leakage_radius_m
        self.fov_azimuth_rad = fov_azimuth_rad or math.radians(60.0)    # ±60°
        self.fov_elevation_rad = fov_elevation_rad or math.radians(12.0)  # ±12°

        # Statistics
        self.total_input = 0
        self.total_output = 0
        self.rejected_nan = 0
        self.rejected_range = 0
        self.rejected_leakage = 0
        self.rejected_fov = 0
        self.rejected_velocity = 0

    @classmethod
    def from_config(cls, config_path: str) -> 'U300Adapter':
        """Create adapter from system.yaml configuration."""
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)

        radar_cfg = cfg.get('radar', {})
        noise_cfg = cfg.get('u300_noise', {})
        doppler_cfg = cfg.get('doppler', {})

        noise = NoiseModel(
            sigma_range_m=noise_cfg.get('sigma_range_m', {}).get('value', 0.10),
            sigma_azimuth_rad=noise_cfg.get('sigma_azimuth_rad', {}).get('value', 0.035),
            sigma_elevation_rad=noise_cfg.get('sigma_elevation_rad', {}).get('value', 0.070),
            sigma_doppler_mps=noise_cfg.get('sigma_doppler_mps', {}).get('value', 0.05),
        )

        return cls(
            noise_model=noise,
            doppler_sign=doppler_cfg.get('sign', -1),
            max_range_m=radar_cfg.get('max_range_m', 200.0),
            min_range_m=radar_cfg.get('min_range_m', 0.3),
            leakage_radius_m=radar_cfg.get('leakage_radius_m', 0.35),
            fov_azimuth_rad=math.radians(radar_cfg.get('fov_azimuth_deg', 120.0) / 2),
            fov_elevation_rad=math.radians(radar_cfg.get('fov_elevation_deg', 24.0) / 2),
        )

    def convert_frame(self, raw_points: np.ndarray, timestamp: float,
                      frame_id: int = 0,
                      rx_timestamp: float = None) -> RadarFrame:
        """Convert a raw U300 frame to a RadarFrame with canonical measurements.

        Args:
            raw_points: (N, 4) float32 [x, y, z, v] in U300 radar frame
            timestamp: Sensor timestamp (preserved, §5.1)
            frame_id: Frame sequence number
            rx_timestamp: Receive timestamp for latency tracking

        Returns:
            RadarFrame with list of RadarMeasurement objects
        """
        if rx_timestamp is None:
            rx_timestamp = timestamp

        self.total_input += len(raw_points)

        if len(raw_points) == 0:
            return RadarFrame(
                timestamp=timestamp, rx_timestamp=rx_timestamp,
                frame_id=frame_id, measurements=[], point_count=0,
                raw_points=raw_points, valid=True,
            )

        # Stage 1: NaN/Inf rejection (§11.1)
        valid_mask = np.all(np.isfinite(raw_points), axis=1)
        self.rejected_nan += int(np.sum(~valid_mask))
        pts = raw_points[valid_mask]

        xyz = pts[:, :3]
        v_raw = pts[:, 3]

        # Compute polar coordinates using atan2 (§5.2)
        r, theta, phi = polar_from_cartesian_batch(xyz)

        # Stage 2: Range gate (§11.1 items 2-3)
        range_mask = (r > max(self.leakage_radius_m, self.min_range_m)) & (r < self.max_range_m)
        self.rejected_leakage += int(np.sum(r <= self.leakage_radius_m))
        self.rejected_range += int(np.sum(~range_mask)) - int(np.sum(r <= self.leakage_radius_m))

        # Stage 3: FOV gate (§11.1 item 4)
        fov_mask = (np.abs(theta) <= self.fov_azimuth_rad) & (np.abs(phi) <= self.fov_elevation_rad)
        self.rejected_fov += int(np.sum(~fov_mask & range_mask))

        # Stage 4: Impossible velocity rejection (§11.1 item 5)
        vel_mask = np.abs(v_raw) < 50.0  # Generous limit
        self.rejected_velocity += int(np.sum(~vel_mask & range_mask & fov_mask))

        # Combined mask
        keep = range_mask & fov_mask & vel_mask

        # Apply Doppler sign correction ONCE (§5.3)
        v_corrected = v_raw * self.doppler_sign

        # Build measurements
        measurements = []
        for i in np.where(keep)[0]:
            meas = RadarMeasurement(
                timestamp=timestamp,
                x_r=float(xyz[i, 0]),
                y_r=float(xyz[i, 1]),
                z_r=float(xyz[i, 2]),
                range_m=float(r[i]),
                azimuth_rad=float(theta[i]),
                elevation_rad=float(phi[i]),
                doppler_mps=float(v_corrected[i]),
                quality=1.0,  # No quality/SNR from U300 firmware currently
                sigma_range=self.noise.sigma_range_m,
                sigma_azimuth=self.noise.sigma_azimuth_rad,
                sigma_elevation=self.noise.sigma_elevation_rad,
                sigma_doppler=self.noise.sigma_doppler_mps,
            )
            measurements.append(meas)

        self.total_output += len(measurements)

        return RadarFrame(
            timestamp=timestamp,
            rx_timestamp=rx_timestamp,
            frame_id=frame_id,
            measurements=measurements,
            point_count=len(measurements),
            raw_points=raw_points,
            valid=len(measurements) > 0,
        )

    def get_rejection_stats(self) -> dict:
        return {
            'total_input': self.total_input,
            'total_output': self.total_output,
            'rejected_nan': self.rejected_nan,
            'rejected_range': self.rejected_range,
            'rejected_leakage': self.rejected_leakage,
            'rejected_fov': self.rejected_fov,
            'rejected_velocity': self.rejected_velocity,
        }
