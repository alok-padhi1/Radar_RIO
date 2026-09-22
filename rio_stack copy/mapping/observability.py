#!/usr/bin/env python3
"""
mapping/observability.py — Registration observability and degeneracy analysis.

Blueprint §17: One of the highest-priority parts of the new stack.

Key rules:
- A high correspondence count is NOT enough (§17)
- A high fitness score is NOT enough (§17)
- A low RMSE is NOT enough (§17)
- A result is acceptable ONLY when required DOF are sufficiently constrained

§17.1: Reject underconstrained pose updates
§17.2: Track 6-DOF (Tx,Ty,Tz,Rx,Ry,Rz) observability independently
§17.3: Rank-aware fusion — fuse only the observable subspace
"""

from __future__ import annotations

import logging
from typing import Optional

import numpy as np

from estimator.state import ObservabilityInfo

logger = logging.getLogger(__name__)


def compute_observability(hessian: np.ndarray,
                          lambda_min_threshold: float = 3.0) -> ObservabilityInfo:
    """Compute 6-DOF observability from the registration Hessian.

    Blueprint §17: Let H = J^T W J. Compute eigenvalues λ₁...λ₆.
    Each describes information along a different state direction.

    Args:
        hessian: (6, 6) information matrix H = J^T W J
        lambda_min_threshold: Minimum eigenvalue for a DOF to be observable

    Returns:
        ObservabilityInfo with per-DOF flags and eigenvalues
    """
    obs = ObservabilityInfo()

    if hessian is None or hessian.shape != (6, 6):
        return obs

    # Eigendecomposition
    try:
        eigenvalues, eigenvectors = np.linalg.eigh(hessian)
    except np.linalg.LinAlgError:
        logger.warning("Eigendecomposition failed on Hessian")
        return obs

    obs.eigenvalues = eigenvalues

    # Condition number
    ev_pos = eigenvalues[eigenvalues > 0]
    if len(ev_pos) >= 2:
        obs.condition_number = float(ev_pos[-1] / ev_pos[0])
    else:
        obs.condition_number = float('inf')

    # Map eigenvectors to DOF labels
    # The eigenvectors of the Hessian indicate which state directions
    # each eigenvalue constrains. We classify based on dominant components:
    # indices 0,1,2 → translation (Tx,Ty,Tz)
    # indices 3,4,5 → rotation (Rx,Ry,Rz)
    dof_labels = ['tx', 'ty', 'tz', 'rx', 'ry', 'rz']

    # For each DOF, check if there's sufficient information
    # Project each DOF basis vector onto the eigenvectors
    # and sum the weighted contributions
    for d, label in enumerate(dof_labels):
        # How much information exists in this DOF direction?
        # Project the d-th unit vector onto each eigenvector,
        # weight by eigenvalue
        info_d = 0.0
        for k in range(len(eigenvalues)):
            projection = eigenvectors[d, k] ** 2  # squared projection
            info_d += projection * max(eigenvalues[k], 0.0)

        setattr(obs, label, info_d >= lambda_min_threshold)

    obs.n_observable_dof = sum(1 for l in dof_labels if getattr(obs, l))

    return obs


def check_registration_quality(fitness: float, rmse: float,
                                n_correspondences: int,
                                observability: ObservabilityInfo,
                                min_fitness: float = 0.3,
                                max_rmse: float = 2.0,
                                min_correspondences: int = 15,
                                min_observable_dof: int = 3) -> tuple[bool, list[str]]:
    """Check if a registration result is acceptable.

    Blueprint §17: A result is acceptable ONLY when the required
    degrees of freedom are sufficiently constrained.

    Returns:
        (acceptable, reasons) — True if registration can be used
    """
    reasons = []
    acceptable = True

    if n_correspondences < min_correspondences:
        reasons.append(f"too_few_correspondences_{n_correspondences}")
        acceptable = False

    if fitness < min_fitness:
        reasons.append(f"low_fitness_{fitness:.3f}")
        acceptable = False

    if rmse > max_rmse:
        reasons.append(f"high_rmse_{rmse:.3f}")
        acceptable = False

    if observability.n_observable_dof < min_observable_dof:
        reasons.append(f"insufficient_observable_dof_{observability.n_observable_dof}")
        acceptable = False

    # §17.1: specifically check translation axes
    if not observability.tx and not observability.ty:
        reasons.append("no_horizontal_translation_observability")
        acceptable = False

    return acceptable, reasons


def select_observable_update(transform: np.ndarray,
                              observability: ObservabilityInfo,
                              hessian: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Apply rank-aware fusion: update only observable DOF.

    Blueprint §17.1, §17.3:
    - Constrained directions → update
    - Weak directions → keep prediction / increase covariance

    Args:
        transform: (4, 4) full registration transform
        observability: Per-DOF observability flags
        hessian: (6, 6) information matrix

    Returns:
        (masked_transform, covariance) — transform with weak DOF zeroed,
        and the posterior covariance from the observable subspace
    """
    # Extract the 6-DOF update [tx, ty, tz, rx, ry, rz]
    # Translation from the transform
    dt = transform[:3, 3].copy()

    # Rotation as angle-axis (small angle approx for incremental update)
    R = transform[:3, :3]
    # Extract small rotation vector
    dr = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ]) / 2.0

    update = np.concatenate([dt, dr])
    dof_flags = [observability.tx, observability.ty, observability.tz,
                 observability.rx, observability.ry, observability.rz]

    # Zero out unobservable DOF
    masked_update = update.copy()
    for i, observable in enumerate(dof_flags):
        if not observable:
            masked_update[i] = 0.0

    # Rebuild masked transform
    masked_T = np.eye(4)
    masked_T[:3, 3] = masked_update[:3]
    # Small-angle rotation reconstruction
    dr_masked = masked_update[3:]
    if np.linalg.norm(dr_masked) > 1e-10:
        from mapping.deskew import exp_so3
        masked_T[:3, :3] = exp_so3(dr_masked)

    # Compute covariance from observable subspace
    # Invert the Hessian for covariance, but only in the observable subspace
    try:
        cov = np.linalg.pinv(hessian)
    except np.linalg.LinAlgError:
        cov = np.eye(6) * 1e6  # Very large uncertainty

    # Inflate covariance for unobservable directions
    for i, observable in enumerate(dof_flags):
        if not observable:
            cov[i, i] = max(cov[i, i], 1e6)  # Very large uncertainty

    return masked_T, cov


def estimate_hessian_from_correspondences(source_points: np.ndarray,
                                           target_points: np.ndarray,
                                           weights: np.ndarray = None) -> np.ndarray:
    """Estimate the 6×6 Hessian from point correspondences.

    Computes H = J^T W J where J is the Jacobian of the point-to-point
    residual with respect to the 6-DOF pose [tx,ty,tz,rx,ry,rz].

    Args:
        source_points: (N, 3) source points
        target_points: (N, 3) target points (corresponding)
        weights: (N,) optional per-correspondence weights

    Returns:
        (6, 6) Hessian / information matrix
    """
    N = len(source_points)
    if N == 0:
        return np.zeros((6, 6))

    if weights is None:
        weights = np.ones(N)

    H = np.zeros((6, 6))

    for i in range(N):
        p = source_points[i]
        w = weights[i]

        # Jacobian of residual w.r.t. [tx, ty, tz, rx, ry, rz]
        # d(T@p)/d(tx,ty,tz) = I
        # d(T@p)/d(rx,ry,rz) = -[p]× (skew symmetric)
        J_i = np.zeros((3, 6))
        J_i[:, :3] = np.eye(3)  # Translation part
        J_i[:, 3:] = -np.array([
            [0, -p[2], p[1]],
            [p[2], 0, -p[0]],
            [-p[1], p[0], 0],
        ])  # Rotation part

        H += w * J_i.T @ J_i

    return H
