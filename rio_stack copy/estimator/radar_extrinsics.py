"""
estimator/radar_extrinsics.py — Radar and Altimeter extrinsic transforms.

Master Plan §7, §8, §18: Provides the single authoritative T_B_R and T_B_A
transforms. All radar→body conversions go through this module.

Frame contract:
    U300 native frame: (X=lateral, Y=forward, Z=up)
    Body frame: FRD (X=forward, Y=right, Z=down)

    The transform has two parts:
      P  = axis permutation: (X_lat, Y_fwd, Z_up) → (X_fwd, Y_right, Z_down)
      R_tilt = mechanical pitch-down tilt about body +Y axis

    R_B_R = R_tilt @ P

    t_B_R = radar-origin position relative to body-origin, expressed in body FRD.

    For radar-origin velocity:
        v_radar_body = R_B_R @ v_radar

    For body-origin velocity (lever-arm compensated):
        v_body = v_radar_body - cross(omega_body, t_B_R)

    For point cloud transform:
        p_body = R_B_R @ p_radar + t_B_R

IMPORTANT: The U300 axis convention is ported from the old verified TiltMount.
Per Corrective Addendum §1, this is treated as HYPOTHESIS until verified
with a physical reflector test (boresight, right, above).
"""

import logging
import math
import numpy as np
from typing import Optional

logger = logging.getLogger(__name__)


class RadarExtrinsics:
    """Manages T_B_R (radar-to-body) extrinsic transform.

    Ported from the old verified TiltMount class. Handles:
    1. U300 native frame axis permutation (X_lat, Y_fwd, Z_up) → body-aligned
    2. Mechanical pitch-down tilt rotation
    3. Lever-arm translation

    Usage:
        ext = RadarExtrinsics.from_config(config)
        v_body = ext.velocity_radar_to_body(v_radar, omega_body)
        pts_body = ext.points_radar_to_body(pts_radar)
    """

    def __init__(self, R_B_R: np.ndarray, t_B_R: np.ndarray,
                 tilt_deg: float = 0.0, lateral_sign: float = 1.0,
                 calibrated: bool = False):
        """
        Args:
            R_B_R: (3,3) rotation matrix from radar frame to body FRD
            t_B_R: (3,) translation — radar origin in body coordinates [m], body FRD
            tilt_deg: mechanical tilt angle (for logging)
            lateral_sign: U300 lateral axis polarity (for logging)
            calibrated: whether these values have been experimentally validated
        """
        self.R_B_R = R_B_R.copy()
        self.t_B_R = t_B_R.copy()
        self.tilt_deg = tilt_deg
        self.lateral_sign = lateral_sign
        self.calibrated = calibrated

        if not calibrated:
            logger.warning(
                "Radar extrinsics are HYPOTHESIS — axis convention from old TiltMount, "
                "not independently verified with reflector test. "
                "Pure rotation tests may show false translational velocity."
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
        tilt_deg = rpy[1]  # pitch = tilt angle
        lateral_sign = ext_cfg.get('lateral_sign', 1.0)

        R_B_R = cls._build_R_B_R(tilt_deg, lateral_sign)

        inst = cls(R_B_R=R_B_R, t_B_R=t, tilt_deg=tilt_deg,
                   lateral_sign=lateral_sign, calibrated=False)
        logger.info(f"Loaded extrinsics: tilt={tilt_deg} deg, lateral_sign={lateral_sign}, t_B_R={t.tolist()}")
        logger.info(f"R_B_R:\n{R_B_R}")
        return inst

    @staticmethod
    def _build_R_B_R(tilt_deg: float, lateral_sign: float = 1.0) -> np.ndarray:
        """Build the full radar->body rotation matrix.

        Ported from old TiltMount.__post_init__:
        1. P: axis permutation (X_lat, Y_fwd, Z_up) -> (X_fwd, Y_right, Z_down)
        2. R_tilt: pitch-down rotation about body +Y axis
        3. R_B_R = R_tilt @ P

        The U300 native frame is documented as:
            X = lateral (azimuth direction)
            Y = forward (boresight / range-gate direction)
            Z = up (elevation direction)

        Body FRD:
            X = forward
            Y = right
            Z = down

        Permutation P:
            X_body = Y_radar (forward)
            Y_body = lateral_sign * X_radar (right)
            Z_body = -Z_radar (down = -up)
        """
        # Axis permutation: radar native -> body-aligned (pre-tilt)
        P = np.array([
            [0.0,          1.0,  0.0],   # X_body = Y_radar (forward)
            [lateral_sign, 0.0,  0.0],   # Y_body = +/-X_radar (right)
            [0.0,          0.0, -1.0],   # Z_body = -Z_radar (down)
        ])

        # Pitch-down tilt about body +Y axis
        th = math.radians(tilt_deg)
        c, s = math.cos(th), math.sin(th)
        R_tilt = np.array([
            [ c,  0.0, -s],
            [0.0, 1.0, 0.0],
            [ s,  0.0,  c],
        ])

        # Combined: first permute axes, then apply tilt
        return R_tilt @ P

    @classmethod
    def identity(cls) -> 'RadarExtrinsics':
        """Identity transform (radar frame = body frame)."""
        return cls(R_B_R=np.eye(3), t_B_R=np.zeros(3), calibrated=False)

    def velocity_radar_to_body(self, v_radar: np.ndarray,
                                omega_body: np.ndarray) -> np.ndarray:
        """Transform radar ego-velocity to body-origin velocity.

        Addendum par 3:
            v_radar_body = R_B_R @ v_radar          (frame rotation)
            v_body = v_radar_body - cross(omega_body, t_B_R)  (lever-arm)

        NOTE: If lever-arm Doppler compensation was already applied
        BEFORE the Doppler solve (as recommended by Addendum par 3),
        then only the R_B_R rotation should be used here, and the
        lever-arm subtraction should be skipped to avoid double-compensation.

        Args:
            v_radar: (3,) ego-velocity in radar frame [m/s]
            omega_body: (3,) angular velocity in body FRD [rad/s]

        Returns:
            (3,) velocity at body origin, in body FRD [m/s]
        """
        # Step 1: rotate velocity into body frame
        v_radar_body = self.R_B_R @ v_radar

        # Step 2: subtract lever-arm velocity
        v_lever = np.cross(omega_body, self.t_B_R)
        v_body = v_radar_body - v_lever

        return v_body

    def rotate_velocity_to_body(self, v_radar: np.ndarray) -> np.ndarray:
        """Rotate radar velocity to body frame WITHOUT lever-arm.

        Use this when lever-arm is pre-compensated in Doppler space.

        Args:
            v_radar: (3,) ego-velocity in radar frame [m/s]

        Returns:
            (3,) velocity in body FRD [m/s]
        """
        return self.R_B_R @ v_radar

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

    Master Plan par 8: U200A beam direction and lever arm.
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
