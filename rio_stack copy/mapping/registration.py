#!/usr/bin/env python3
"""
mapping/registration.py — Scan-to-submap point cloud registration.

Blueprint §16: GICP baseline with adaptive correspondence radius.
Returns full quality data, not just the transform.

The registration layer MUST return the Hessian and correspondences
so the observability layer (§17) can evaluate 6-DOF constraints.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# We conditionally import open3d so that modules on the RIO-only
# path do not fail if open3d is missing (e.g. Jetson dependency issues, §29)
try:
    import open3d as o3d
except ImportError:
    o3d = None

from estimator.state import RegistrationResult
from mapping.observability import compute_observability, estimate_hessian_from_correspondences

logger = logging.getLogger(__name__)


@dataclass
class RegistrationConfig:
    """Configuration for GICP registration."""
    max_correspondence_dist_m: float = 6.0
    adaptive_radius_multiplier: float = 2.0
    min_correspondences: int = 15
    max_iterations: int = 50
    voxel_size_m: float = 1.0


class PointCloudRegistrator:
    """Performs scan-to-submap registration.

    Blueprint §16.2: Adaptive correspondence radius.
    Blueprint §16.3: Returns full RegistrationResult (T, fitness, RMSE, Hessian).
    """

    def __init__(self, config: RegistrationConfig = None):
        self.config = config or RegistrationConfig()
        if o3d is None:
            logger.error("Open3D is not available. Registration will fail.")

    def register(self, source_pts: np.ndarray,
                 target_pts: np.ndarray,
                 initial_guess: np.ndarray = None) -> RegistrationResult:
        """Register source point cloud to target submap.

        Args:
            source_pts: (N, 3) current radar scan (deskewed)
            target_pts: (M, 3) target submap points
            initial_guess: (4, 4) initial SE(3) transform (from RIO prediction)

        Returns:
            RegistrationResult with full quality metadata
        """
        if o3d is None:
            return RegistrationResult(transform=np.eye(4), valid=False)

        if initial_guess is None:
            initial_guess = np.eye(4)

        result = RegistrationResult(transform=initial_guess, valid=False)

        if len(source_pts) < self.config.min_correspondences or len(target_pts) < self.config.min_correspondences:
            logger.warning(f"Registration failed: insufficient points ({len(source_pts)} -> {len(target_pts)})")
            return result

        # Convert to Open3D point clouds
        source_pcd = o3d.geometry.PointCloud()
        source_pcd.points = o3d.utility.Vector3dVector(source_pts)

        target_pcd = o3d.geometry.PointCloud()
        target_pcd.points = o3d.utility.Vector3dVector(target_pts)

        # Estimate normals for GICP
        source_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=self.config.voxel_size_m * 2.0, max_nn=30))
        target_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=self.config.voxel_size_m * 2.0, max_nn=30))

        # Perform Generalized ICP
        try:
            icp_result = o3d.pipelines.registration.registration_generalized_icp(
                source=source_pcd,
                target=target_pcd,
                max_correspondence_distance=self.config.max_correspondence_dist_m,
                init=initial_guess,
                estimation_method=o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=self.config.max_iterations
                )
            )
        except Exception as e:
            logger.error(f"GICP registration failed: {e}")
            return result

        if not icp_result.transformation.shape == (4, 4):
            return result

        # Extract quality metrics
        result.transform = icp_result.transformation
        result.fitness = icp_result.fitness
        result.rmse = icp_result.inlier_rmse
        result.num_correspondences = len(icp_result.correspondence_set)

        if result.num_correspondences < self.config.min_correspondences:
            return result

        # Compute Observability (Hessian)
        # Open3D get_information_matrix_from_point_clouds provides the Hessian
        try:
            info_matrix = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
                source_pcd, target_pcd, self.config.max_correspondence_dist_m,
                result.transform)
            
            # The Open3D info matrix is 6x6. Our observability module expects a 6x6 Hessian
            # in the order [tx, ty, tz, rx, ry, rz]. We need to verify Open3D's order.
            # Open3D's order is [rx, ry, rz, tx, ty, tz] or similar depending on version.
            # To be safe and explicit, we can estimate it manually from the correspondence set.
            
            corr = np.asarray(icp_result.correspondence_set)
            if len(corr) > 0:
                s_pts_corr = source_pts[corr[:, 0]]
                # Transform source points to target frame to evaluate Hessian at the solution
                s_pts_corr_hom = np.hstack([s_pts_corr, np.ones((len(s_pts_corr), 1))])
                s_pts_corr_target = (result.transform @ s_pts_corr_hom.T)[:3].T
                
                t_pts_corr = target_pts[corr[:, 1]]
                
                # Estimate Hessian
                hessian = estimate_hessian_from_correspondences(s_pts_corr_target, t_pts_corr)
                result.hessian = hessian
                
                # Compute observability
                obs = compute_observability(hessian)
                result.observability = obs
                result.eigenvalues = obs.eigenvalues
                result.condition_number = obs.condition_number
                
                # Registration is valid only if fitness is decent and observability is OK
                # (Actual gate is applied in health manager or observability module)
                result.valid = True
                
        except Exception as e:
            logger.warning(f"Failed to compute observability: {e}")
            
        return result
