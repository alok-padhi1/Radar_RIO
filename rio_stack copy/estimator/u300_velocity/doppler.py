import numpy as np
from typing import Tuple, List, Optional
import logging
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class DopplerVelocityResult:
    velocity: np.ndarray  # (3,)
    covariance: np.ndarray # (3, 3)
    is_valid: bool
    num_points_used: int
    condition_number: float
    rank: int
    reason: str = ""

class RobustDopplerEstimator:
    """
    Robust U300 Radar Ego Velocity Estimator.
    Solves A * v = b where A is the direction vectors, b is the Doppler velocities.
    Uses iteratively reweighted least squares (IRLS) with Huber loss to reject outliers.
    """
    def __init__(self, 
                 min_points: int = 6, 
                 max_condition_number: float = 100.0,
                 huber_threshold: float = 0.2):
        self.min_points = min_points
        self.max_condition_number = max_condition_number
        self.huber_threshold = huber_threshold

    def estimate(self, 
                 points: np.ndarray, 
                 dopplers: np.ndarray, 
                 angular_velocity: Optional[np.ndarray] = None,
                 lever_arm: Optional[np.ndarray] = None) -> DopplerVelocityResult:
        """
        Estimate radar ego velocity from a set of radar points.
        
        Args:
            points: (N, 3) array of point coordinates in radar frame.
            dopplers: (N,) array of doppler velocities.
            angular_velocity: (3,) angular velocity in radar frame.
            lever_arm: (3,) translation from IMU to Radar.
            
        Returns:
            DopplerVelocityResult
        """
        if len(points) < self.min_points:
            return DopplerVelocityResult(
                velocity=np.zeros(3), covariance=np.eye(3)*1e6, is_valid=False,
                num_points_used=len(points), condition_number=np.inf, rank=0,
                reason=f"too_few_useful_points ({len(points)} < {self.min_points})"
            )

        # 1. Normalize directions: u_i = p_i / ||p_i||
        ranges = np.linalg.norm(points, axis=1)
        valid_ranges = ranges > 0.1
        if not np.any(valid_ranges):
             return DopplerVelocityResult(
                velocity=np.zeros(3), covariance=np.eye(3)*1e6, is_valid=False,
                num_points_used=0, condition_number=np.inf, rank=0,
                reason="all_points_too_close"
            )

        points = points[valid_ranges]
        dopplers = dopplers[valid_ranges]
        ranges = ranges[valid_ranges]
        
        N = len(points)
        directions = points / ranges[:, np.newaxis] # (N, 3)

        # 2. Formulate A and b
        # Measurement model: d_i = -u_i^T * v_R
        A = -directions
        b = dopplers

        # 3. Robust Iteratively Reweighted Least Squares (IRLS)
        W = np.ones(N)
        v_est = np.zeros(3)
        max_iters = 10
        
        # Initial unweighted least squares
        try:
            v_est, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        except np.linalg.LinAlgError:
            return self._invalid_result(N, "initial_lstsq_failed")

        # IRLS loop
        for _ in range(max_iters):
            residuals = np.abs(A @ v_est - b)
            
            # Huber weights
            W = np.where(residuals <= self.huber_threshold, 
                         1.0, 
                         self.huber_threshold / np.maximum(residuals, 1e-6))
            
            # Weighted least squares: (A^T * W * A) * v = A^T * W * b
            AW = A * np.sqrt(W)[:, np.newaxis]
            bw = b * np.sqrt(W)
            
            try:
                v_est, _, _, _ = np.linalg.lstsq(AW, bw, rcond=None)
            except np.linalg.LinAlgError:
                break
                
        # 4. Final Information Matrix and Covariance
        W = np.where(residuals <= self.huber_threshold, 1.0, 0.0) # hard threshold for final cov
        num_inliers = int(np.sum(W))
        if num_inliers < self.min_points:
            return self._invalid_result(num_inliers, f"too_few_inliers ({num_inliers} < {self.min_points})")
            
        AW_final = A * np.sqrt(W)[:, np.newaxis]
        H_v = AW_final.T @ AW_final
        
        # 5. Check Rank and Conditioning
        rank = np.linalg.matrix_rank(H_v)
        if rank < 3:
            return self._invalid_result(num_inliers, f"rank_deficient ({rank} < 3)")
            
        eigenvalues = np.linalg.eigvalsh(H_v)
        cond = np.abs(eigenvalues[-1] / (eigenvalues[0] + 1e-8))
        if cond > self.max_condition_number:
            return self._invalid_result(num_inliers, f"ill_conditioned ({cond:.1f} > {self.max_condition_number})")

        # Covariance
        # Assuming measurement noise std dev = 0.1 m/s
        noise_var = 0.1**2
        P_v = np.linalg.inv(H_v) * noise_var

        # 6. Lever arm compensation
        # v_R = v_B + omega_B x t_BR
        # Therefore v_B = v_R - omega_B x t_BR
        v_B = v_est
        if angular_velocity is not None and lever_arm is not None:
            v_B = v_est - np.cross(angular_velocity, lever_arm)

        return DopplerVelocityResult(
            velocity=v_B,
            covariance=P_v,
            is_valid=True,
            num_points_used=num_inliers,
            condition_number=cond,
            rank=rank,
            reason="success"
        )
        
    def _invalid_result(self, n: int, reason: str) -> DopplerVelocityResult:
        return DopplerVelocityResult(
            velocity=np.zeros(3), covariance=np.eye(3)*1e6, is_valid=False,
            num_points_used=n, condition_number=np.inf, rank=0,
            reason=reason
        )
