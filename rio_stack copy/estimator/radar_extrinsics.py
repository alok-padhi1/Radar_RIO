"""
estimator/radar_extrinsics.py — Radar and Altimeter extrinsic transforms.

Master Plan §7, §8, §18: Provides the single authoritative T_B_R and T_B_A
transforms. All radar→body conversions go through this module.

Frame contract:
    t_B_R = radar-origin position relative to body-origin, expressed in body FRD.
    R_B_R = rotation from radar frame to body FRD frame.

    For radar-origin velocity:
        v_radar_body = R_B_R @ v_radar

    For body-origin velocity (lever-arm compensated):
        v_body = v_radar_body - cross(omega_body, t_B_R)

    For point cloud transform:
        p_body = R_B_R @ p_radar + t_B_R

IMPORTANT: Current YAML values are PLACEHOLDER — not flight-validated.
"""

import logging
import numpy as np
from typing import Optional

logger = logging.getLogger(__name__)


def _rpy_to_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Convert roll/pitch/yaw (degrees) to rotation matrix.

    Convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """
    r = np.radians(roll_deg)
    p = np.radians(pitch_deg)
    y = np.radians(yaw_deg)

    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)

    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    return Rz @ Ry @ Rx


class RadarExtrinsics:
    """Manages T_B_R (radar-to-body) extrinsic transform.

    Loaded from config/system.yaml. Values are PLACEHOLDER until
    physically measured and validated with pure rotation tests (§7).

    Usage:
        ext = RadarExtrinsics.from_config(config)
        v_body = ext.velocity_radar_to_body(v_radar, omega_body)
        pts_body = ext.points_radar_to_body(pts_radar)
    """

    def __init__(self, R_B_R: np.ndarray, t_B_R: np.ndarray, calibrated: bool = False):
        """
        Args:
            R_B_R: (3,3) rotation matrix from radar frame to body FRD
            t_B_R: (3,) translation — radar origin in body coordinates [m], body FRD
            calibrated: whether these values have been experimentally validated
        """
        self.R_B_R = R_B_R.copy()
        self.t_B_R = t_B_R.copy()
        self.calibrated = calibrated

        if not calibrated:
            logger.warning(
                "Radar extrinsics are PLACEHOLDER — not experimentally calibrated. "
                "Pure rotation tests will show false translational velocity."
            )

    @classmethod
    def from_config(cls, config: dict) -> 'RadarExtrinsics':
        """Load from system.yaml config dict.

        Expected config structure:
            extrinsics:
              radar_to_body:
                translation_m: [tx, ty, tz]
                rotation_rpy_deg: [roll, pitch, yaw]
        """
        ext_cfg = config.get('extrinsics', {}).get('radar_to_body', {})
        t = np.array(ext_cfg.get('translation_m', [0.0, 0.0, 0.0]))
        rpy = ext_cfg.get('rotation_rpy_deg', [0.0, 0.0, 0.0])
        R = _rpy_to_matrix(rpy[0], rpy[1], rpy[2])

        # These are PLACEHOLDER until calibrated
        inst = cls(R_B_R=R, t_B_R=t, calibrated=False)
        logger.info(f"Loaded extrinsics: t_B_R={t.tolist()}, RPY_deg={rpy}")
        logger.info(f"R_B_R:\n{R}")
        return inst

    @classmethod
    def identity(cls) -> 'RadarExtrinsics':
        """Identity transform (radar frame = body frame)."""
        return cls(R_B_R=np.eye(3), t_B_R=np.zeros(3), calibrated=False)

    def velocity_radar_to_body(self, v_radar: np.ndarray,
                                omega_body: np.ndarray) -> np.ndarray:
        """Transform radar ego-velocity to body-origin velocity.

        Master Plan §18:
            v_radar_body = R_B_R @ v_radar          (frame rotation)
            v_body = v_radar_body - cross(ω_body, t_B_R)  (lever-arm)

        Args:
            v_radar: (3,) ego-velocity in radar frame [m/s]
            omega_body: (3,) angular velocity in body FRD [rad/s]

        Returns:
            (3,) velocity at body origin, in body FRD [m/s]
        """
        # Step 1: rotate velocity into body frame
        v_radar_body = self.R_B_R @ v_radar

        # Step 2: subtract lever-arm velocity
        # The radar is at offset t_B_R from body origin.
        # Lever-arm velocity contribution: ω × t_B_R
        v_lever = np.cross(omega_body, self.t_B_R)
        v_body = v_radar_body - v_lever

        return v_body

    def points_radar_to_body(self, pts_radar: np.ndarray) -> np.ndarray:
        """Transform radar point cloud to body FRD frame.

        Args:
            pts_radar: (N, 3) points in radar frame

        Returns:
            (N, 3) points in body FRD frame
        """
        return (self.R_B_R @ pts_radar.T).T + self.t_B_R

    def get_lever_arm(self) -> np.ndarray:
        """Return the lever arm vector t_B_R in body FRD."""
        return self.t_B_R.copy()


class AltimeterExtrinsics:
    """Manages T_B_A (altimeter-to-body) extrinsic transform.

    Master Plan §8: U200A beam direction and lever arm.
    """

    def __init__(self, t_B_A: np.ndarray, beam_axis_body: np.ndarray,
                 calibrated: bool = False):
        """
        Args:
            t_B_A: (3,) altimeter position in body FRD [m]
            beam_axis_body: (3,) unit vector of altimeter beam in body FRD
            calibrated: whether experimentally validated
        """
        self.t_B_A = t_B_A.copy()
        self.beam_axis_body = beam_axis_body.copy() / np.linalg.norm(beam_axis_body)
        self.calibrated = calibrated

    @classmethod
    def from_config(cls, config: dict) -> 'AltimeterExtrinsics':
        """Load from system.yaml."""
        ext_cfg = config.get('extrinsics', {}).get('altimeter_to_body', {})
        t = np.array(ext_cfg.get('translation_m', [0.0, 0.0, 0.0]))
        beam = np.array(ext_cfg.get('beam_axis_body', [0.0, 0.0, 1.0]))  # FRD: +Z = down
        return cls(t_B_A=t, beam_axis_body=beam, calibrated=False)
