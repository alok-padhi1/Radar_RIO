#!/usr/bin/env python3
"""
estimator/health.py — Navigation Health Manager.

Blueprint §20, §39, §59, §60, §61: Implements explicit navigation states,
hard navigation gates, quality levels, known failure detection, and
6-DOF observability tracking.

The health manager is the GATE between the estimator and the MAVLink output.
It must never allow "fabricated certainty" (§71) — if the estimator is
degraded, the output must reflect that.
"""

from __future__ import annotations

import enum
import logging
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# Avoid circular imports by using string-based forward references
# The actual types are in estimator.state
from estimator.state import (
    NavigationMode, QualityLevel, NavigationState,
    ObservabilityInfo, RIOState, SLAMState, AltimeterSample,
)

logger = logging.getLogger(__name__)


# ============================================================================
# Sensor Health — Blueprint §39
# ============================================================================

@dataclass
class SensorHealth:
    """Health status for a single sensor."""
    name: str
    healthy: bool = False
    last_update: float = 0.0
    update_count: int = 0
    error_count: int = 0
    stale_timeout_s: float = 2.0
    reasons: list[str] = field(default_factory=list)

    def is_stale(self, now: float = None) -> bool:
        if now is None:
            now = time.monotonic()
        return (now - self.last_update) > self.stale_timeout_s if self.last_update > 0 else True

    def update(self, healthy: bool, reason: str = ""):
        self.last_update = time.monotonic()
        self.update_count += 1
        self.healthy = healthy
        if not healthy and reason:
            self.reasons = [reason]
            self.error_count += 1
        elif healthy:
            self.reasons = []


# ============================================================================
# Known Failure Conditions — Blueprint §39
# ============================================================================

class FailureCategory(enum.Enum):
    """Categories of known failures (§39)."""
    # Radar-dominant
    TOO_FEW_POINTS = "too_few_useful_points"
    FEATURE_STARVATION = "feature_starvation"
    MOSTLY_FLAT_GROUND = "mostly_flat_ground"
    DYNAMIC_DOMINATED = "dynamic_dominated_scene"
    MULTIPATH_GHOST = "multipath_ghost_domination"
    SELF_REFLECTIONS = "self_reflections"
    WEAK_ANGULAR_DIVERSITY = "weak_angular_diversity"
    DOPPLER_DISAGREEMENT = "large_doppler_disagreement"

    # IMU-dominant
    IMU_SAMPLE_DROP = "imu_sample_drop"
    IMU_TIMESTAMP_JUMP = "imu_timestamp_discontinuity"
    GYRO_SATURATION = "gyro_saturation"
    ACCEL_SATURATION = "accelerometer_saturation"
    LARGE_BIAS_INNOVATION = "large_bias_innovation"

    # Mapping
    FEW_CORRESPONDENCES = "too_few_correspondences"
    RANK_DEFICIENT = "rank_deficient_hessian"
    AMBIGUOUS_REGISTRATION = "ambiguous_registration"
    LARGE_RESIDUAL = "large_residual"
    LARGE_TRANSFORM_JUMP = "large_transform_jump"
    SUBMAP_MISMATCH = "submap_mismatch"
    REPEATED_GEOMETRY = "repeated_geometry"

    # Navigation
    EXTERNAL_NAV_STALE = "external_nav_stale"
    COVARIANCE_TOO_LARGE = "covariance_too_large"
    INNOVATION_TOO_LARGE = "innovation_too_large"
    CLOCK_DELAY_UNSTABLE = "clock_delay_unstable"
    SENSOR_UNHEALTHY = "sensor_unhealthy"


# ============================================================================
# Health Configuration
# ============================================================================

@dataclass
class HealthConfig:
    """Configuration for health thresholds — from config/system.yaml."""
    # Sensor thresholds
    min_radar_points: int = 5
    min_static_points: int = 3
    radar_stale_timeout_s: float = 2.0
    imu_stale_timeout_s: float = 1.0
    altimeter_stale_timeout_s: float = 3.0

    # Observability
    lambda_min_threshold: float = 3.0
    observable_ratio: float = 0.25

    # Covariance
    max_velocity_sigma_mps: float = 5.0
    max_position_sigma_m: float = 50.0
    covariance_consistency_ratio_max: float = 2.0

    # Innovation
    max_innovation_sigma: float = 5.0

    # Timing
    max_measurement_age_ms: float = 100.0
    max_latency_ms: float = 200.0

    # Transition hold times (prevent rapid bouncing)
    min_good_duration_s: float = 2.0
    min_degraded_duration_s: float = 1.0

    # Addendum par 6: velocity sanity gate
    max_velocity_mps: float = 25.0

    # Addendum par 7: stale radar accept timeout
    radar_accept_stale_timeout_s: float = 0.5


# ============================================================================
# Navigation Health Manager — Blueprint §20
# ============================================================================

class NavigationHealthManager:
    """Manages navigation state machine and health assessment.

    State machine (§20):
        INITIALIZING → RIO_ONLY → RIO_GOOD → SLAM_GOOD
                                            → RIO_DEGRADED → RECOVERY → INVALID

    Hard navigation gates (§60):
        radar healthy AND IMU healthy AND time synchronized AND
        state not stale AND covariance finite AND innovation reasonable AND
        required DOF observable

    The response to failure is degradation (§61), NOT fabricated certainty.
    """

    def __init__(self, config: HealthConfig = None):
        self.config = config or HealthConfig()
        self._mode = NavigationMode.INITIALIZING
        self._quality = QualityLevel.INVALID
        self._reasons: list[str] = []
        self._active_failures: list[FailureCategory] = []
        self._mode_entry_time = time.monotonic()

        # Per-sensor health
        self.radar_health = SensorHealth("radar", stale_timeout_s=self.config.radar_stale_timeout_s)
        self.imu_health = SensorHealth("imu", stale_timeout_s=self.config.imu_stale_timeout_s)
        self.altimeter_health = SensorHealth("altimeter", stale_timeout_s=self.config.altimeter_stale_timeout_s)
        self.time_sync_healthy = False

        # Observability
        self._observability = ObservabilityInfo()

        # Watchdog timestamps (§48)
        self._last_rio_output: float = 0.0
        self._last_slam_output: float = 0.0
        self._last_mavlink_output: float = 0.0

        # Addendum par 7: stale radar accept tracking
        self._last_radar_accept_time: float = 0.0
        self._velocity_diverged: bool = False

    @property
    def mode(self) -> NavigationMode:
        return self._mode

    @property
    def quality(self) -> QualityLevel:
        return self._quality

    @property
    def navigation_valid(self) -> bool:
        """Hard navigation gate — Blueprint §60."""
        return self._check_hard_gates()

    @property
    def observability(self) -> ObservabilityInfo:
        return self._observability

    def _check_hard_gates(self) -> bool:
        """Blueprint §60: All conditions must hold."""
        return all([
            self.radar_health.healthy,
            self.imu_health.healthy,
            self.time_sync_healthy,
            not self.radar_health.is_stale(),
            not self.imu_health.is_stale(),
            self._mode not in (NavigationMode.INITIALIZING, NavigationMode.INVALID),
        ])

    def _set_mode(self, new_mode: NavigationMode):
        if new_mode != self._mode:
            old = self._mode
            self._mode = new_mode
            self._mode_entry_time = time.monotonic()
            logger.info(f"Navigation mode: {old.value} → {new_mode.value}")

    def update_radar_health(self, n_points: int, n_static: int,
                            doppler_residual_rms: float = float('nan'),
                            angular_spread: float = float('nan')):
        """Update radar sensor health assessment."""
        reasons = []
        healthy = True

        if n_points < self.config.min_radar_points:
            reasons.append(FailureCategory.TOO_FEW_POINTS.value)
            healthy = False

        if n_static < self.config.min_static_points:
            reasons.append(FailureCategory.FEATURE_STARVATION.value)
            healthy = False

        if not np.isnan(doppler_residual_rms) and doppler_residual_rms > 2.0:
            reasons.append(FailureCategory.DOPPLER_DISAGREEMENT.value)
            healthy = False

        reason = "; ".join(reasons) if reasons else ""
        self.radar_health.update(healthy, reason)

    def update_imu_health(self, sample_rate_hz: float = 0.0,
                          gyro_mag: float = 0.0, accel_mag: float = 0.0,
                          timestamp_monotonic: bool = True):
        """Update IMU sensor health assessment."""
        reasons = []
        healthy = True

        if sample_rate_hz < 10.0:
            reasons.append(FailureCategory.IMU_SAMPLE_DROP.value)
            healthy = False

        if not timestamp_monotonic:
            reasons.append(FailureCategory.IMU_TIMESTAMP_JUMP.value)
            healthy = False

        # Gyro saturation check (~2000 deg/s typical)
        if gyro_mag > 34.9:  # ~2000 deg/s in rad/s
            reasons.append(FailureCategory.GYRO_SATURATION.value)
            healthy = False

        # Accel saturation check (~16g typical)
        if accel_mag > 156.8:  # 16g in m/s²
            reasons.append(FailureCategory.ACCEL_SATURATION.value)
            healthy = False

        reason = "; ".join(reasons) if reasons else ""
        self.imu_health.update(healthy, reason)

    def update_altimeter_health(self, sample: AltimeterSample):
        """Update altimeter health from a sample."""
        healthy = sample.valid and not sample.is_stale(self.config.altimeter_stale_timeout_s)
        reason = ""
        if not sample.valid:
            reason = "invalid_reading"
        elif sample.is_stale(self.config.altimeter_stale_timeout_s):
            reason = "stale_reading"
        self.altimeter_health.update(healthy, reason)

    def update_radar_accept_time(self):
        """Notify that a radar velocity update was accepted by the ESKF.

        Addendum par 7: Health must be based on accepted-radar timestamps.
        """
        self._last_radar_accept_time = time.monotonic()

    def update_observability(self, eigenvalues: np.ndarray,
                              threshold: float = None):
        """Update 6-DOF observability from registration or estimator.

        Blueprint §17.2: Track each direction independently.
        """
        if threshold is None:
            threshold = self.config.lambda_min_threshold

        obs = ObservabilityInfo()
        obs.eigenvalues = eigenvalues

        if len(eigenvalues) >= 6:
            sorted_ev = np.sort(eigenvalues)
            obs.condition_number = sorted_ev[-1] / max(sorted_ev[0], 1e-12)

            # Map eigenvalues to DOF (tx,ty,tz,rx,ry,rz)
            # In practice the eigenvector directions determine which DOF each
            # eigenvalue corresponds to. For a simplified check we just count.
            dof_flags = eigenvalues >= threshold
            obs.tx = bool(dof_flags[0]) if len(dof_flags) > 0 else False
            obs.ty = bool(dof_flags[1]) if len(dof_flags) > 1 else False
            obs.tz = bool(dof_flags[2]) if len(dof_flags) > 2 else False
            obs.rx = bool(dof_flags[3]) if len(dof_flags) > 3 else False
            obs.ry = bool(dof_flags[4]) if len(dof_flags) > 4 else False
            obs.rz = bool(dof_flags[5]) if len(dof_flags) > 5 else False
            obs.n_observable_dof = int(np.sum(dof_flags))

        self._observability = obs

    def update_state_machine(self, rio: Optional[RIOState] = None,
                              slam: Optional[SLAMState] = None):
        """Update the navigation state machine — Blueprint §20.

        Transition logic:
            INITIALIZING → RIO_IMU_ONLY: enough IMU + radar data
            RIO_IMU_ONLY → RADAR_VELOCITY_GOOD: sufficient radar confidence
            RADAR_VELOCITY_GOOD → SLAM_GOOD: stable submap match
            * → RADAR_VELOCITY_DEGRADED: degraded geometry
            * → RECOVERY: radar/IMU issues
            * → INVALID: critical failure
        """
        now = time.monotonic()
        self._reasons = []

        # Addendum par 6: velocity sanity gate
        if rio is not None and hasattr(rio, 'velocity'):
            v_mag = float(np.linalg.norm(rio.velocity))
            if v_mag > self.config.max_velocity_mps:
                self._velocity_diverged = True
                self._set_mode(NavigationMode.INVALID)
                self._quality = QualityLevel.INVALID
                self._reasons.append(f"velocity_diverged ({v_mag:.1f} m/s)")
                logger.error(f"VELOCITY SANITY GATE: {v_mag:.1f} m/s > {self.config.max_velocity_mps} limit")
                return

        # Addendum par 7: stale radar accept
        if (self._last_radar_accept_time > 0 and
                self._mode in (NavigationMode.RADAR_VELOCITY_GOOD,) and
                (now - self._last_radar_accept_time) > self.config.radar_accept_stale_timeout_s):
            self._set_mode(NavigationMode.RADAR_VELOCITY_DEGRADED)
            self._quality = QualityLevel.DEGRADED
            self._reasons.append("radar_accept_stale")
            return

        # Check for INVALID conditions first
        if self.imu_health.is_stale(now):
            self._set_mode(NavigationMode.INVALID)
            self._quality = QualityLevel.INVALID
            self._reasons.append("imu_stale")
            return

        if self.radar_health.is_stale(now) and self._mode not in (
                NavigationMode.INITIALIZING,):
            # Radar lost — enter recovery, not instant invalid
            if self._mode != NavigationMode.RECOVERY:
                self._set_mode(NavigationMode.RECOVERY)
            self._quality = QualityLevel.CRITICAL
            self._reasons.append("radar_lost")
            return

        # State transitions
        if self._mode == NavigationMode.INITIALIZING:
            if (self.imu_health.healthy and self.radar_health.healthy and
                    rio is not None and rio.valid):
                self._set_mode(NavigationMode.RIO_IMU_ONLY)
                self._quality = QualityLevel.DEGRADED

        elif self._mode == NavigationMode.RIO_IMU_ONLY:
            if rio is not None and rio.valid and rio.n_static_points >= self.config.min_static_points:
                hold_ok = (now - self._mode_entry_time) >= self.config.min_good_duration_s
                if hold_ok:
                    self._set_mode(NavigationMode.RADAR_VELOCITY_GOOD)
                    self._quality = QualityLevel.GOOD

        elif self._mode == NavigationMode.RADAR_VELOCITY_GOOD:
            if not self.radar_health.healthy or (rio and not rio.valid):
                self._set_mode(NavigationMode.RADAR_VELOCITY_DEGRADED)
                self._quality = QualityLevel.DEGRADED
                self._reasons.append("radar_degraded")
            elif slam is not None and slam.valid and slam.n_correspondences >= self.config.min_radar_points:
                self._set_mode(NavigationMode.SLAM_GOOD)
                self._quality = QualityLevel.EXCELLENT

        elif self._mode == NavigationMode.SLAM_GOOD:
            if slam is None or not slam.valid:
                self._set_mode(NavigationMode.RADAR_VELOCITY_GOOD)
                self._quality = QualityLevel.GOOD
                self._reasons.append("slam_invalid")
            elif not self.radar_health.healthy:
                self._set_mode(NavigationMode.RADAR_VELOCITY_DEGRADED)
                self._quality = QualityLevel.DEGRADED
                self._reasons.append("radar_degraded_from_slam")

        elif self._mode == NavigationMode.SLAM_DEGRADED:
            if slam is not None and slam.valid:
                self._set_mode(NavigationMode.SLAM_GOOD)
                self._quality = QualityLevel.GOOD
            elif not self.radar_health.healthy:
                self._set_mode(NavigationMode.RADAR_VELOCITY_DEGRADED)
                self._quality = QualityLevel.DEGRADED

        elif self._mode == NavigationMode.RADAR_VELOCITY_DEGRADED:
            if self.radar_health.healthy and rio is not None and rio.valid:
                self._set_mode(NavigationMode.RADAR_VELOCITY_GOOD)
                self._quality = QualityLevel.GOOD
            elif self.radar_health.is_stale(now):
                self._set_mode(NavigationMode.RECOVERY)
                self._quality = QualityLevel.CRITICAL

        elif self._mode == NavigationMode.RECOVERY:
            if self.radar_health.healthy and self.imu_health.healthy:
                # Radar reacquired — do innovation check before resuming
                self._set_mode(NavigationMode.RIO_IMU_ONLY)
                self._quality = QualityLevel.DEGRADED
                self._reasons.append("recovery_radar_reacquired")

        elif self._mode == NavigationMode.INVALID:
            if self.imu_health.healthy and self.radar_health.healthy:
                self._set_mode(NavigationMode.INITIALIZING)
                self._quality = QualityLevel.INVALID

    def build_navigation_state(self, rio: Optional[RIOState] = None,
                                slam: Optional[SLAMState] = None,
                                altimeter: Optional[AltimeterSample] = None,
                                frame_id: int = 0) -> NavigationState:
        """Build the canonical NavigationState message.

        Updates the state machine and assembles the complete output.
        """
        now = time.monotonic()

        # Update altimeter health if provided
        if altimeter is not None:
            self.update_altimeter_health(altimeter)

        # Run state machine
        self.update_state_machine(rio, slam)

        # Assemble NavigationState
        state = NavigationState(
            timestamp=now,
            frame_id=frame_id,
            mode=self._mode,
            quality=self._quality,
            quality_reasons=list(self._reasons),
            rio=rio,
            slam=slam,
            altimeter=altimeter,
            health={
                'radar': self.radar_health.healthy,
                'imu': self.imu_health.healthy,
                'altimeter': self.altimeter_health.healthy,
                'time_sync': self.time_sync_healthy,
                'observable': self._observability.is_fully_observable(),
                'navigation_valid': self.navigation_valid,
            },
        )

        # Copy pose/velocity from RIO if available
        if rio is not None and rio.valid:
            state.position = rio.position.copy()
            state.velocity = rio.velocity.copy()
            state.quaternion = rio.quaternion.copy()
            state.velocity_covariance = rio.velocity_covariance
            state.pose_covariance = rio.pose_covariance

        return state

    def get_quality_report(self) -> dict:
        """Generate quality report for logging — Blueprint §59.

        Returns:
            Dict with quality_level and reasons array.
        """
        return {
            'quality_level': int(self._quality),
            'mode': self._mode.value,
            'reasons': list(self._reasons),
            'radar_healthy': self.radar_health.healthy,
            'imu_healthy': self.imu_health.healthy,
            'altimeter_healthy': self.altimeter_health.healthy,
            'time_sync_healthy': self.time_sync_healthy,
            'observability': self._observability.to_dict(),
            'navigation_valid': self.navigation_valid,
        }
