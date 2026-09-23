"""
estimator/u300_velocity/doppler.py — Robust Doppler ego-velocity estimator.

Ported from the battle-tested doppler_rio.py pipeline. Implements:
  1. RANSAC with static-hypothesis margin
  2. Weighted least-squares seed (1/R^2 weights)
  3. IRLS refinement with Huber loss
  4. Condition number gate (3D)
  5. Posterior sigma gate
  6. Acceleration gate (temporal)
  7. Deadband

Corrective Addendum par 2, 3, 13, 14, 15, 16.

Doppler measurement model:
    d_i = -u_i^T v + noise
where u_i = p_i / ||p_i|| is the LOS unit vector in body frame.
"""

import math
import logging
import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class DopplerVelocityResult:
    velocity: np.ndarray      # (3,) body-frame velocity [m/s]
    covariance: np.ndarray    # (3,3) velocity covariance
    is_valid: bool
    num_points_used: int
    condition_number: float
    rank: int
    is_static: bool = False
    reason: str = ""


def _safe_covariance(ATA: np.ndarray, huber_inflate: float = 1.0) -> np.ndarray:
    """Invert A^T W A into a covariance WITHOUT truncating weak directions.

    Ported from old doppler_rio.py (FIX R-1 / R-7):
    - Uses eigendecomposition with floored eigenvalues
    - Weak directions get LARGE variance (not zero)
    - Huber inflation for robust covariance
    """
    n = ATA.shape[0]
    try:
        M = 0.5 * (ATA + ATA.T)
        w, V = np.linalg.eigh(M)
        floor = max(float(w.max()), 1e-12) * 1e-12
        w_safe = np.clip(w, floor, None)
        cov = (V * (1.0 / w_safe)) @ V.T
        cov = 0.5 * (cov + cov.T)
        cov = np.clip(cov, -1e6, 1e6)
        return cov * float(huber_inflate)
    except np.linalg.LinAlgError:
        return np.eye(n) * 1e6


def doppler_ransac(u_body: np.ndarray, v_radial: np.ndarray,
                   eps: float = 0.40, iters: int = 60,
                   min_inlier_ratio: float = 0.35, max_speed_mps: float = 25.0,
                   static_margin_frac: float = 0.12, static_margin_min: int = 3,
                   cond_reject_threshold: float = 30.0,
                   static_min_ratio: float = 0.70,
                   static_min_points: int = 10,
                   min_points_moving: int = 8,
                   rng=None):
    """Doppler-RANSAC with static-hypothesis margin and condition gate.

    Ported from old doppler_rio.py.

    Args:
        u_body: (N,3) unit LOS vectors in body frame
        v_radial: (N,) measured radial speed per point
        eps: inlier threshold [m/s]
        iters: RANSAC iterations
        min_inlier_ratio: minimum fraction of inliers
        max_speed_mps: reject hypotheses faster than this
        static_margin_frac: margin for static vs moving
        static_margin_min: minimum margin count
        cond_reject_threshold: condition number gate
        static_min_ratio: fraction of points needed for static hypothesis
        static_min_points: minimum points for static hypothesis
        min_points_moving: minimum points for 3-DOF solve

    Returns:
        (mask, is_static, cond_number) or None if rejected
    """
    n = u_body.shape[0]
    if n < 3:
        return None
    rng = rng or np.random.default_rng()

    # Static hypothesis: v = 0
    res_zero = np.abs(v_radial)
    mask_zero = res_zero < eps
    score_zero = int(mask_zero.sum())

    # Static credibility check
    static_credible = (
        score_zero >= static_min_points and
        (score_zero / n) >= static_min_ratio
    )

    if n < min_points_moving:
        if static_credible:
            return mask_zero, True, 1.0
        return None

    best_mask, best_score = None, -1
    for _ in range(iters):
        idx = rng.choice(n, size=3, replace=False)
        A_sub = -u_body[idx]
        b_sub = v_radial[idx]

        # Degenerate-triad guard
        sing = np.linalg.svd(A_sub, compute_uv=False)
        if sing[-1] < 1e-3 or (sing[0] / max(sing[-1], 1e-9)) > cond_reject_threshold:
            continue

        try:
            v_k, _, _, _ = np.linalg.lstsq(A_sub, b_sub, rcond=1e-2)
        except (np.linalg.LinAlgError, ValueError):
            continue

        if np.linalg.norm(v_k) > max_speed_mps:
            continue

        residual = np.abs(v_radial + u_body @ v_k)
        mask = residual < eps
        score = int(mask.sum())
        if score > best_score:
            best_score, best_mask = score, mask

    margin = max(static_margin_min, int(math.ceil(static_margin_frac * n)))

    if best_mask is None or best_score <= score_zero + margin:
        if static_credible:
            return mask_zero, True, 1.0
        return None

    if best_score / n < min_inlier_ratio:
        return None

    # Condition number gate on FULL 3-column LOS matrix
    A_full = -u_body[best_mask]
    cond_3d = float(np.linalg.cond(A_full))
    if cond_3d > cond_reject_threshold:
        return None

    return best_mask, False, cond_3d


def weighted_refit(u_body: np.ndarray, v_radial: np.ndarray,
                   ranges: np.ndarray, mask: np.ndarray,
                   max_speed_mps: float = 25.0):
    """Weighted least-squares refit on RANSAC inliers with 1/R^2 weights.

    Returns:
        (v_body, cov) or (None, None) on failure
    """
    A = -u_body[mask]
    b = v_radial[mask]
    w = 1.0 / np.clip(ranges[mask], 0.5, None) ** 2
    sqrt_w = np.sqrt(w)
    A_w = A * sqrt_w[:, None]
    b_w = b * sqrt_w

    try:
        v_res, _, _, _ = np.linalg.lstsq(A_w, b_w, rcond=1e-2)
        v_body = v_res[:3]
        ATA = A_w.T @ A_w
        cov = _safe_covariance(ATA)
    except (np.linalg.LinAlgError, ValueError):
        return None, None

    if np.linalg.norm(v_body) > max_speed_mps:
        return None, None
    return v_body, cov


def irls_refit(u_body: np.ndarray, v_radial: np.ndarray,
               ranges: np.ndarray, v_seed: np.ndarray,
               huber_delta_mps: float = 0.20, max_iters: int = 4,
               tol_mps: float = 1e-3, gross_outlier_mult: float = 10.0,
               max_speed_mps: float = 25.0):
    """IRLS refinement of RANSAC-seeded velocity with Huber loss.

    Simplified from old doppler_rio.py irls_refit — uses 1/R^2 weights
    instead of full polar-uncertainty model for robustness.

    Returns:
        (v_body, cov) or (None, None) on failure
    """
    v = np.asarray(v_seed, dtype=float).copy()

    # Remove gross outliers from seed
    resid_seed = np.abs(v_radial + u_body @ v)
    keep = resid_seed <= gross_outlier_mult * huber_delta_mps
    if keep.sum() < 3:
        return None, None

    u_k = u_body[keep]
    v_k = v_radial[keep]
    r_k = ranges[keep]
    A_full = -u_k
    b_full = v_k
    cov = None

    for _ in range(max_iters):
        # Measurement weights: 1/R^2
        w_meas = 1.0 / np.clip(r_k, 0.5, None) ** 2

        # Huber weights
        resid = b_full + u_k @ v
        abs_r = np.abs(resid)
        w_huber = np.ones_like(abs_r)
        far = abs_r > huber_delta_mps
        w_huber[far] = huber_delta_mps / np.clip(abs_r[far], 1e-6, None)
        w = w_meas * w_huber

        # Huber inflation factor for covariance
        _w_sum = float(np.sum(w_meas))
        _w_eff = float(np.sum(w))
        huber_inflate = _w_sum / max(_w_eff, 1e-12)

        sqrt_w = np.sqrt(np.clip(w, 0.0, None))
        A_w = A_full * sqrt_w[:, None]
        b_w = b_full * sqrt_w

        try:
            v_new, _, _, _ = np.linalg.lstsq(A_w, b_w, rcond=1e-2)
            v_new = v_new[:3]
            ATA = A_w.T @ A_w
            cov = _safe_covariance(ATA, huber_inflate)
        except (np.linalg.LinAlgError, ValueError):
            cov = None
            break

        if np.linalg.norm(v_new) > max_speed_mps:
            break

        delta = np.linalg.norm(v_new - v)
        v = v_new
        if delta < tol_mps:
            break

    if cov is None or np.linalg.norm(v) > max_speed_mps:
        return None, None
    return v, cov


class RobustDopplerEstimator:
    """Full robust Doppler ego-velocity estimator pipeline.

    Implements the hybrid robust pipeline per Corrective Addendum par 2:
        physical filtering -> RANSAC -> WLS seed -> IRLS -> gates -> output

    All computation is done in BODY frame after transforming points.
    """

    def __init__(self,
                 min_points: int = 6,
                 max_condition_number: float = 30.0,
                 ransac_eps: float = 0.40,
                 ransac_iters: int = 60,
                 min_inlier_ratio: float = 0.35,
                 huber_delta_mps: float = 0.20,
                 max_speed_mps: float = 25.0,
                 max_sigma_v_mps: float = 0.60,
                 max_accel_mps2: float = 15.0,
                 deadband_mps: float = 0.05,
                 static_margin_frac: float = 0.12,
                 static_margin_min: int = 3,
                 static_min_ratio: float = 0.70,
                 static_min_points: int = 10,
                 min_points_moving: int = 8):
        self.min_points = min_points
        self.max_condition_number = max_condition_number
        self.ransac_eps = ransac_eps
        self.ransac_iters = ransac_iters
        self.min_inlier_ratio = min_inlier_ratio
        self.huber_delta_mps = huber_delta_mps
        self.max_speed_mps = max_speed_mps
        self.max_sigma_v_mps = max_sigma_v_mps
        self.max_accel_mps2 = max_accel_mps2
        self.deadband_mps = deadband_mps
        self.static_margin_frac = static_margin_frac
        self.static_margin_min = static_margin_min
        self.static_min_ratio = static_min_ratio
        self.static_min_points = static_min_points
        self.min_points_moving = min_points_moving

        # Acceleration gate state
        self._v_prev: Optional[np.ndarray] = None
        self._t_prev: Optional[float] = None

    def estimate(self,
                 points_body: np.ndarray,
                 dopplers: np.ndarray,
                 t_frame: float = 0.0) -> DopplerVelocityResult:
        """Estimate body-frame ego velocity from radar points already in body frame.

        Args:
            points_body: (N, 3) points in body FRD frame
            dopplers: (N,) Doppler velocities (lever-arm pre-compensated)
            t_frame: frame timestamp for acceleration gate

        Returns:
            DopplerVelocityResult with body-frame velocity
        """
        N = len(points_body)
        if N < self.min_points:
            return self._invalid_result(N, f"too_few_points ({N} < {self.min_points})")

        # Compute ranges and LOS unit vectors in body frame
        ranges = np.linalg.norm(points_body, axis=1)
        valid = ranges > 0.1
        if valid.sum() < self.min_points:
            return self._invalid_result(int(valid.sum()), "too_few_after_range_filter")

        points_body = points_body[valid]
        dopplers = dopplers[valid]
        ranges = ranges[valid]
        N = len(points_body)

        u_body = points_body / ranges[:, np.newaxis]  # (N, 3)

        # === Stage 1: RANSAC ===
        ransac_result = doppler_ransac(
            u_body, dopplers,
            eps=self.ransac_eps,
            iters=self.ransac_iters,
            min_inlier_ratio=self.min_inlier_ratio,
            max_speed_mps=self.max_speed_mps,
            static_margin_frac=self.static_margin_frac,
            static_margin_min=self.static_margin_min,
            cond_reject_threshold=self.max_condition_number,
            static_min_ratio=self.static_min_ratio,
            static_min_points=self.static_min_points,
            min_points_moving=self.min_points_moving,
        )

        if ransac_result is None:
            return self._invalid_result(N, "ransac_reject")

        mask, is_static, cond = ransac_result

        if is_static:
            v_body = np.zeros(3)
            cov = np.eye(3) * 0.25  # sigma = 0.5 m/s
            n_inliers = int(mask.sum())
            # Update accel gate state
            self._v_prev = v_body.copy()
            self._t_prev = t_frame
            return DopplerVelocityResult(
                velocity=v_body, covariance=cov, is_valid=True,
                num_points_used=n_inliers, condition_number=cond,
                rank=3, is_static=True, reason="static"
            )

        # === Stage 2: Weighted least-squares seed ===
        v_seed, _ = weighted_refit(
            u_body, dopplers, ranges, mask,
            max_speed_mps=self.max_speed_mps
        )
        if v_seed is None:
            return self._invalid_result(int(mask.sum()), "wls_seed_failed")

        # === Stage 3: IRLS refinement ===
        v_body, cov = irls_refit(
            u_body, dopplers, ranges, v_seed,
            huber_delta_mps=self.huber_delta_mps,
            max_speed_mps=self.max_speed_mps,
        )
        if v_body is None:
            return self._invalid_result(int(mask.sum()), "irls_failed")

        n_inliers = int(mask.sum())

        # === Stage 4: Posterior sigma gate ===
        sigma = np.sqrt(np.clip(np.diag(cov), 0.0, None))
        if np.any(sigma > self.max_sigma_v_mps):
            logger.info(
                f"Posterior sigma gate: sigma={sigma.round(3)} > {self.max_sigma_v_mps} "
                f"-- null direction active, rejecting"
            )
            return self._invalid_result(n_inliers, "sigma_gate")

        # === Stage 5: Acceleration gate ===
        if self._v_prev is not None and self._t_prev is not None:
            dt = t_frame - self._t_prev
            if 0.0 < dt < 0.5:
                implied_accel = float(np.linalg.norm(v_body - self._v_prev) / dt)
                if implied_accel > self.max_accel_mps2:
                    logger.warning(
                        f"Accel gate: {implied_accel:.1f} m/s^2 > "
                        f"{self.max_accel_mps2} limit "
                        f"(v_prev={self._v_prev.round(2)} -> v={v_body.round(2)}, "
                        f"dt={dt:.3f}s) -- rejecting"
                    )
                    return self._invalid_result(n_inliers, "accel_gate")

        # === Stage 6: Deadband ===
        if np.linalg.norm(v_body) < self.deadband_mps:
            v_body = np.zeros(3)

        # Update acceleration gate state
        self._v_prev = v_body.copy()
        self._t_prev = t_frame

        return DopplerVelocityResult(
            velocity=v_body, covariance=cov, is_valid=True,
            num_points_used=n_inliers, condition_number=cond,
            rank=3, is_static=False, reason="success"
        )

    def _invalid_result(self, n: int, reason: str) -> DopplerVelocityResult:
        return DopplerVelocityResult(
            velocity=np.zeros(3), covariance=np.eye(3) * 1e6,
            is_valid=False, num_points_used=n,
            condition_number=np.inf, rank=0, reason=reason
        )
