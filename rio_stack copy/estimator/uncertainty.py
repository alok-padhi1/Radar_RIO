#!/usr/bin/env python3
"""
estimator/uncertainty.py — Radar measurement uncertainty propagation.

Blueprint §9.5, §10: The radar naturally measures (r, θ, φ) rather than
an isotropic Cartesian point. Per-point covariance must be propagated from
polar coordinates via the Jacobian:

    Σ_p = J Σ_{r,θ,φ} J^T

where J = ∂p/∂(r, θ, φ).

Do NOT manufacture per-point precision that the U300 does not provide (§9.5).
Start with experimentally calibrated conservative uncertainties (§10).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class NoiseModel:
    """U300 measurement noise parameters — Blueprint §10.3.

    Load from config/system.yaml. Values must be experimentally calibrated.
    """
    sigma_range_m: float = 0.10
    sigma_azimuth_rad: float = 0.035     # ~2.0 deg
    sigma_elevation_rad: float = 0.070   # ~4.0 deg
    sigma_doppler_mps: float = 0.05

    def polar_covariance(self) -> np.ndarray:
        """Diagonal covariance in (r, θ, φ) space."""
        return np.diag([
            self.sigma_range_m ** 2,
            self.sigma_azimuth_rad ** 2,
            self.sigma_elevation_rad ** 2,
        ])


def cartesian_from_polar(r: float, theta: float, phi: float) -> np.ndarray:
    """Convert polar (range, azimuth, elevation) to Cartesian (x, y, z).

    Blueprint §9.5:
        x = r cos(θ) cos(φ)
        y = r sin(θ) cos(φ)
        z = r sin(φ)

    Args:
        r: Range [m]
        theta: Azimuth angle [rad] — atan2(y, x)
        phi: Elevation angle [rad] — atan2(z, sqrt(x²+y²))

    Returns:
        (3,) Cartesian coordinates
    """
    cos_phi = math.cos(phi)
    return np.array([
        r * math.cos(theta) * cos_phi,
        r * math.sin(theta) * cos_phi,
        r * math.sin(phi),
    ])


def polar_from_cartesian(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Convert Cartesian (x, y, z) to polar (range, azimuth, elevation).

    Blueprint §5.2: Use atan2, NOT asin(y/range/cos(elevation)).
    atan2 avoids unnecessary quadrant ambiguity.

    Returns:
        (r, theta, phi) — range [m], azimuth [rad], elevation [rad]
    """
    r = math.sqrt(x * x + y * y + z * z)
    theta = math.atan2(y, x)         # azimuth
    phi = math.atan2(z, math.sqrt(x * x + y * y))   # elevation
    return r, theta, phi


def polar_from_cartesian_batch(xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Batch convert (N, 3) Cartesian to polar.

    Blueprint §5.2: Use atan2 everywhere.

    Args:
        xyz: (N, 3) array of [x, y, z]

    Returns:
        (r, theta, phi) — each (N,) arrays
    """
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    r = np.sqrt(x ** 2 + y ** 2 + z ** 2)
    theta = np.arctan2(y, x)
    phi = np.arctan2(z, np.sqrt(x ** 2 + y ** 2))
    return r, theta, phi


def jacobian_polar_to_cartesian(r: float, theta: float, phi: float) -> np.ndarray:
    """Jacobian J = ∂p/∂(r, θ, φ) for the polar→Cartesian transformation.

    Blueprint §9.5:
        p = [r cos(θ) cos(φ),  r sin(θ) cos(φ),  r sin(φ)]^T

    Partial derivatives:
        ∂p/∂r     = [cos(θ)cos(φ),   sin(θ)cos(φ),   sin(φ)]
        ∂p/∂θ     = [-r sin(θ)cos(φ), r cos(θ)cos(φ), 0     ]
        ∂p/∂φ     = [-r cos(θ)sin(φ), -r sin(θ)sin(φ), r cos(φ)]

    Returns:
        (3, 3) Jacobian matrix
    """
    ct, st = math.cos(theta), math.sin(theta)
    cp, sp = math.cos(phi), math.sin(phi)

    return np.array([
        [ct * cp,   -r * st * cp,  -r * ct * sp],
        [st * cp,    r * ct * cp,  -r * st * sp],
        [sp,         0.0,           r * cp],
    ])


def propagate_covariance(r: float, theta: float, phi: float,
                         noise: NoiseModel) -> np.ndarray:
    """Propagate polar measurement covariance to Cartesian space.

    Blueprint §9.5:
        Σ_p = J Σ_{r,θ,φ} J^T

    Args:
        r: Range [m]
        theta: Azimuth [rad]
        phi: Elevation [rad]
        noise: Noise model parameters

    Returns:
        (3, 3) Cartesian covariance matrix
    """
    J = jacobian_polar_to_cartesian(r, theta, phi)
    Sigma_polar = noise.polar_covariance()
    return J @ Sigma_polar @ J.T


def propagate_covariance_batch(r: np.ndarray, theta: np.ndarray,
                                phi: np.ndarray,
                                noise: NoiseModel) -> np.ndarray:
    """Batch propagate covariance for N points.

    Args:
        r, theta, phi: (N,) arrays
        noise: Noise model

    Returns:
        (N, 3, 3) array of per-point Cartesian covariances
    """
    N = len(r)
    Sigma_polar = noise.polar_covariance()
    covs = np.zeros((N, 3, 3))

    for i in range(N):
        J = jacobian_polar_to_cartesian(r[i], theta[i], phi[i])
        covs[i] = J @ Sigma_polar @ J.T

    return covs


def uncertainty_ellipsoid_axes(cov: np.ndarray) -> np.ndarray:
    """Compute the semi-axes of the 1-sigma uncertainty ellipsoid.

    Args:
        cov: (3, 3) covariance matrix

    Returns:
        (3,) semi-axis lengths (eigenvalues' square roots), sorted descending
    """
    eigenvalues = np.linalg.eigvalsh(cov)
    return np.sqrt(np.maximum(eigenvalues, 0.0))[::-1]
