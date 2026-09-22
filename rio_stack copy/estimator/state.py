#!/usr/bin/env python3
"""
estimator/state.py — Canonical data contracts for the RIO navigation stack.

Blueprint §34, §71: Every measurement carries timestamp, frame, measurement,
uncertainty, validity. Every estimate carries state, covariance, quality, age.

These typed dataclasses define the internal message contracts shared by ALL
modules. No module should invent its own ad-hoc data layout.
"""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


# ============================================================================
# Enums
# ============================================================================

class NavigationMode(enum.Enum):
    """Navigation state machine states — Blueprint §20."""
    INITIALIZING = "INITIALIZING"
    RADAR_VELOCITY_GOOD = "RADAR_VELOCITY_GOOD"
    RADAR_VELOCITY_DEGRADED = "RADAR_VELOCITY_DEGRADED"
    RIO_IMU_ONLY = "RIO_IMU_ONLY"
    SLAM_GOOD = "SLAM_GOOD"
    SLAM_DEGRADED = "SLAM_DEGRADED"
    RECOVERY = "RECOVERY"
    INVALID = "INVALID"


class QualityLevel(enum.IntEnum):
    """Navigation quality levels — Blueprint §59."""
    INVALID = 0
    CRITICAL = 1
    DEGRADED = 2
    GOOD = 3
    EXCELLENT = 4


# ============================================================================
# Sensor Measurements — Blueprint §5.2, §7, §8
# ============================================================================

@dataclass
class RadarMeasurement:
    """Canonical radar measurement — Blueprint §5.2.

    One per detected target, converted from raw U300 (x,y,z,v) via the
    U300 adapter. Uses atan2 for angle computation (§5.2), never asin.
    """
    timestamp: float          # Sensor timestamp (NOT time.monotonic unless documented)
    x_r: float                # X in radar frame [m]
    y_r: float                # Y in radar frame [m]
    z_r: float                # Z in radar frame [m]
    range_m: float            # sqrt(x² + y² + z²) [m]
    azimuth_rad: float        # atan2(y, x) [rad]
    elevation_rad: float      # atan2(z, sqrt(x² + y²)) [rad]
    doppler_mps: float        # Radial velocity [m/s], sign-corrected per config
    quality: float = 1.0      # Quality/SNR if available, else 1.0
    sigma_range: float = 0.1  # Range uncertainty [m]
    sigma_azimuth: float = 0.035   # Azimuth uncertainty [rad]
    sigma_elevation: float = 0.070  # Elevation uncertainty [rad]
    sigma_doppler: float = 0.05    # Doppler uncertainty [m/s]


@dataclass
class RadarFrame:
    """A complete radar frame containing all measurements from one scan."""
    timestamp: float                # Sensor timestamp
    rx_timestamp: float             # Receive/parse timestamp (time.monotonic)
    frame_id: int                   # Sequence number
    measurements: list[RadarMeasurement] = field(default_factory=list)
    point_count: int = 0
    raw_points: Optional[np.ndarray] = None   # (N, 4) float32 [x, y, z, v]
    valid: bool = True
    drop_count: int = 0             # Cumulative dropped frame count


@dataclass
class IMUSample:
    """Raw IMU measurement — Blueprint §7.

    Primary input for RIO: raw gyro + accelerometer + timestamp.
    NOT autopilot-derived attitude or velocity (§7.1).
    """
    timestamp: float           # Sensor timestamp (closest available, §7.3)
    accel_x: float             # Accelerometer X [m/s²], body frame
    accel_y: float             # Accelerometer Y [m/s²], body frame
    accel_z: float             # Accelerometer Z [m/s²], body frame
    gyro_x: float              # Gyroscope X [rad/s], body frame
    gyro_y: float              # Gyroscope Y [rad/s], body frame
    gyro_z: float              # Gyroscope Z [rad/s], body frame
    valid: bool = True
    source: str = "RAW_IMU"    # "RAW_IMU" | "SCALED_IMU" | "ATTITUDE"

    @property
    def accel(self) -> np.ndarray:
        return np.array([self.accel_x, self.accel_y, self.accel_z])

    @property
    def gyro(self) -> np.ndarray:
        return np.array([self.gyro_x, self.gyro_y, self.gyro_z])


@dataclass
class IMUAttitude:
    """Fused attitude from the flight controller — REFERENCE/TELEMETRY only.

    Blueprint §29: Mark as REFERENCE, not PRIMARY RIO SENSOR.
    """
    timestamp: float
    roll: float                # [rad]
    pitch: float               # [rad]
    yaw: float                 # [rad]
    rollspeed: float           # [rad/s]
    pitchspeed: float          # [rad/s]
    yawspeed: float            # [rad/s]
    airborne: bool = False
    vz_ned: float = 0.0        # FC EKF vertical velocity, NED +down [m/s]
    source: str = "TELEMETRY"  # Always "TELEMETRY", never "PRIMARY"


@dataclass
class AltimeterSample:
    """Altimeter measurement — Blueprint §8.

    The altimeter outputs range_to_surface. Vertical height requires
    attitude-based correction: h_AGL = r * |e_z^T R_WA a_A| (§8.1).
    """
    timestamp: float           # Sample timestamp
    raw_range_m: float         # Slant range along beam axis [m]
    corrected_agl_m: float = float('nan')  # Corrected AGL if attitude available
    quality: float = 1.0       # 0.0 = bad, 1.0 = good
    valid: bool = True         # False when surface unmeasurable (§8.3)
    sample_age_s: float = 0.0  # Age since last valid sample (§8.2)

    def is_stale(self, max_age_s: float = 0.5) -> bool:
        """Blueprint §8.2: reject when age > configured maximum."""
        return self.sample_age_s > max_age_s


# ============================================================================
# Observability — Blueprint §17
# ============================================================================

@dataclass
class ObservabilityInfo:
    """Per-DOF observability tracking — Blueprint §17.2."""
    tx: bool = False
    ty: bool = False
    tz: bool = False
    rx: bool = False
    ry: bool = False
    rz: bool = False
    eigenvalues: Optional[np.ndarray] = None  # (6,) λ₁..λ₆
    condition_number: float = float('inf')
    n_observable_dof: int = 0

    def is_fully_observable(self) -> bool:
        return all([self.tx, self.ty, self.tz, self.rx, self.ry, self.rz])

    def observable_axes(self) -> list[str]:
        axes = []
        for name in ['tx', 'ty', 'tz', 'rx', 'ry', 'rz']:
            if getattr(self, name):
                axes.append(name)
        return axes

    def to_dict(self) -> dict:
        return {
            'tx': self.tx, 'ty': self.ty, 'tz': self.tz,
            'rx': self.rx, 'ry': self.ry, 'rz': self.rz,
        }


# ============================================================================
# Registration Result — Blueprint §16.3
# ============================================================================

@dataclass
class RegistrationResult:
    """Full registration quality output — Blueprint §16.3.

    The map layer must NEVER return only T (§16.3).
    """
    transform: np.ndarray                # (4, 4) SE(3) transformation
    fitness: float = 0.0                 # Correspondence ratio [0, 1]
    rmse: float = float('inf')           # RMS point-to-point error
    num_correspondences: int = 0
    inlier_ratio: float = 0.0
    hessian: Optional[np.ndarray] = None          # (6, 6) information matrix
    eigenvalues: Optional[np.ndarray] = None      # (6,) sorted eigenvalues
    condition_number: float = float('inf')
    translation_covariance: Optional[np.ndarray] = None  # (3, 3)
    rotation_covariance: Optional[np.ndarray] = None     # (3, 3)
    residual_distribution: Optional[np.ndarray] = None   # per-correspondence residuals
    observability: Optional[ObservabilityInfo] = None
    valid: bool = False


# ============================================================================
# RIO State — Blueprint §9.1
# ============================================================================

@dataclass
class RIOState:
    """RIO estimator output — Blueprint §9.1.

    State: [p_W, v_W, q_WB, b_a, b_g]
    """
    timestamp: float
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))      # p_W [m]
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))      # v_W [m/s]
    quaternion: np.ndarray = field(default_factory=lambda: np.array([0., 0., 0., 1.]))  # q_WB [x,y,z,w]
    bias_accel: np.ndarray = field(default_factory=lambda: np.zeros(3))    # b_a [m/s²]
    bias_gyro: np.ndarray = field(default_factory=lambda: np.zeros(3))     # b_g [rad/s]
    pose_covariance: Optional[np.ndarray] = None   # (6, 6) or (21,) upper-tri
    velocity_covariance: Optional[np.ndarray] = None  # (3, 3) or (6,) upper-tri
    valid: bool = False
    n_radar_points: int = 0
    n_static_points: int = 0
    n_tracked_points: int = 0
    doppler_residual_rms: float = float('nan')
    imu_residual_rms: float = float('nan')
    optimizer_iterations: int = 0
    
    # ESKF Fusion Metrics
    radar_velocity: Optional[tuple] = None
    radar_velocity_covariance: Optional[list] = None
    innovation: Optional[tuple] = None
    mahalanobis_distance: float = 0.0
    radar_points_used: int = 0


# ============================================================================
# SLAM State
# ============================================================================

@dataclass
class SLAMState:
    """SLAM / mapping layer output."""
    timestamp: float
    valid: bool = False
    n_correspondences: int = 0
    registration_rmse: float = float('nan')
    observable_axes: int = 0
    registration: Optional[RegistrationResult] = None
    keyframe_id: int = 0
    n_submap_points: int = 0


# ============================================================================
# Navigation State — Blueprint §34 (complete internal state message)
# ============================================================================

@dataclass
class NavigationState:
    """Complete internal state message — Blueprint §34.

    This is the canonical output consumed by the MAVLink bridge,
    the health manager, and any mission controller.
    """
    timestamp: float = 0.0
    frame_id: int = 0

    # Pose
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quaternion: np.ndarray = field(default_factory=lambda: np.array([0., 0., 0., 1.]))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # Covariances
    pose_covariance: Optional[np.ndarray] = None
    velocity_covariance: Optional[np.ndarray] = None

    # RIO status
    rio: Optional[RIOState] = None

    # SLAM status
    slam: Optional[SLAMState] = None

    # Altimeter
    altimeter: Optional[AltimeterSample] = None

    # Health — Blueprint §34, §60
    health: dict = field(default_factory=lambda: {
        'radar': False,
        'imu': False,
        'altimeter': False,
        'time_sync': False,
        'observable': False,
        'navigation_valid': False,
    })

    # Navigation mode and quality
    mode: NavigationMode = NavigationMode.INITIALIZING
    quality: QualityLevel = QualityLevel.INVALID
    quality_reasons: list[str] = field(default_factory=list)

    @property
    def navigation_valid(self) -> bool:
        """Blueprint §60: Hard navigation gate."""
        return all([
            self.health.get('radar', False),
            self.health.get('imu', False),
            self.health.get('time_sync', False),
            self.mode not in (NavigationMode.INITIALIZING, NavigationMode.INVALID),
            self.quality > QualityLevel.INVALID,
        ])

    def to_dict(self) -> dict:
        """Serialize to dict for logging/JSON output (§34)."""
        return {
            'timestamp': self.timestamp,
            'frame_id': self.frame_id,
            'pose': {
                'position': self.position.tolist() if self.position is not None else None,
                'quaternion': self.quaternion.tolist() if self.quaternion is not None else None,
            },
            'velocity': self.velocity.tolist() if self.velocity is not None else None,
            'rio': {
                'valid': self.rio.valid if self.rio else False,
                'n_radar_points': self.rio.n_radar_points if self.rio else 0,
                'n_static_points': self.rio.n_static_points if self.rio else 0,
                'n_tracked_points': self.rio.n_tracked_points if self.rio else 0,
                'doppler_residual_rms': self.rio.doppler_residual_rms if self.rio else 0,
            } if self.rio else None,
            'slam': {
                'valid': self.slam.valid if self.slam else False,
                'n_correspondences': self.slam.n_correspondences if self.slam else 0,
                'registration_rmse': self.slam.registration_rmse if self.slam else 0,
                'observable_axes': self.slam.observable_axes if self.slam else 0,
            } if self.slam else None,
            'altimeter': {
                'valid': self.altimeter.valid if self.altimeter else False,
                'agl_m': self.altimeter.corrected_agl_m if self.altimeter else 0,
                'age_s': self.altimeter.sample_age_s if self.altimeter else 0,
            } if self.altimeter else None,
            'health': self.health,
            'mode': self.mode.value,
            'quality_level': int(self.quality),
            'quality_reasons': self.quality_reasons,
        }
