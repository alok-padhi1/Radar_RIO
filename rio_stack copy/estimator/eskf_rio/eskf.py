"""
estimator/eskf_rio/eskf.py — Error-State Kalman Filter.

Frame contract:
  Nominal state:
    p_NED   — position in NED world frame [North, East, Down] (metres)
    v_NED   — velocity in NED world frame (m/s)
    q_NB    — quaternion NED-to-Body (FRD) [w, x, y, z]
    b_a     — accelerometer bias in body FRD (m/s²)
    b_g     — gyroscope bias in body FRD (rad/s)

  Error state (15-dim):
    δp (3), δv (3), δθ (3), δb_a (3), δb_g (3)

  Gravity:
    g_NED = [0, 0, +9.80665] — in NED, +Z is down, gravity points down.

  IMU input:
    accel_FRD — accelerometer reading in body FRD (m/s²)
    gyro_FRD  — gyroscope reading in body FRD (rad/s)

  Stationary FRD IMU:
    accel ≈ [0, 0, -9.81]  (sensor measures upward proper acceleration)
    Prediction: v_dot = R_NB @ accel + g_NED
             = R_NB @ [0,0,-9.81] + [0,0,+9.81] = 0  ✓
"""

import numpy as np
from typing import Optional, Tuple
import logging

logger = logging.getLogger(__name__)

class ESKF:
    """Error-State Kalman Filter for Radar-Inertial Odometry.

    State: [p_NED (3), v_NED (3), q_NB (4), b_a (3), b_g (3)]
    Error State: [δp (3), δv (3), δθ (3), δb_a (3), δb_g (3)] (dim 15)

    All vectors are documented with their frame:
      p, v     — NED world frame
      q        — NED-to-Body rotation (Hamilton convention, [w,x,y,z])
      b_a, b_g — body FRD frame
      accel, gyro inputs — body FRD frame
    """
    def __init__(self,
                 initial_p: np.ndarray,
                 initial_q: np.ndarray,
                 initial_v: np.ndarray,
                 initial_ba: np.ndarray,
                 initial_bg: np.ndarray,
                 gravity: np.ndarray = None):
        # Nominal state — all in NED/FRD
        self.p = initial_p.copy()       # p_NED [N, E, D] metres
        self.v = initial_v.copy()       # v_NED [vN, vE, vD] m/s
        self.q = initial_q.copy() / np.linalg.norm(initial_q)  # q_NB [w, x, y, z]
        self.ba = initial_ba.copy()     # accel bias, body FRD
        self.bg = initial_bg.copy()     # gyro bias, body FRD

        # Gravity vector in NED world frame
        # NED: +Z is down, gravity points down → g = [0, 0, +9.80665]
        if gravity is not None:
            self.g = gravity.copy()
        else:
            self.g = np.array([0.0, 0.0, 9.80665])  # NED convention

        # Covariance matrix (15x15)
        self.P = np.eye(15) * 1e-4
        self.P[0:3, 0:3] = np.eye(3) * 0.1**2   # position uncertainty: 10cm
        self.P[3:6, 3:6] = np.eye(3) * 0.5**2   # velocity uncertainty: 0.5 m/s
        self.P[6:9, 6:9] = np.eye(3) * 0.1**2   # orientation uncertainty: ~5 deg
        self.P[9:12, 9:12] = np.eye(3) * 0.1**2 # accel bias uncertainty
        self.P[12:15, 12:15] = np.eye(3) * 0.05**2 # gyro bias uncertainty

        # Process noise continuous-time power spectral densities
        self.Q_c = np.zeros((12, 12))
        self.Q_c[0:3, 0:3] = np.eye(3) * 0.01**2   # accel noise PSD
        self.Q_c[3:6, 3:6] = np.eye(3) * 0.005**2  # gyro noise PSD
        self.Q_c[6:9, 6:9] = np.eye(3) * 0.001**2  # accel bias random walk
        self.Q_c[9:12, 9:12] = np.eye(3) * 0.0001**2 # gyro bias random walk

        self.last_time = None

    def quaternion_to_matrix(self, q: np.ndarray) -> np.ndarray:
        """Convert quaternion [w, x, y, z] to rotation matrix R_NB."""
        w, x, y, z = q
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
        ])

    def matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """Convert rotation matrix to quaternion [w, x, y, z]."""
        tr = np.trace(R)
        if tr > 0:
            S = 2.0 * np.sqrt(tr + 1.0)
            w = 0.25 * S
            x = (R[2,1] - R[1,2]) / S
            y = (R[0,2] - R[2,0]) / S
            z = (R[1,0] - R[0,1]) / S
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
            w = (R[2,1] - R[1,2]) / S
            x = 0.25 * S
            y = (R[0,1] + R[1,0]) / S
            z = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
            w = (R[0,2] - R[2,0]) / S
            x = (R[0,1] + R[1,0]) / S
            y = 0.25 * S
            z = (R[1,2] + R[2,1]) / S
        else:
            S = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
            w = (R[1,0] - R[0,1]) / S
            x = (R[0,2] + R[2,0]) / S
            y = (R[1,2] + R[2,1]) / S
            z = 0.25 * S
        q = np.array([w, x, y, z])
        return q / np.linalg.norm(q)

    def skew(self, v: np.ndarray) -> np.ndarray:
        """Skew-symmetric matrix [v]×."""
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])

    def predict(self, dt: float, accel: np.ndarray, gyro: np.ndarray):
        """Predict state using IMU measurements.

        Args:
            dt: time step (seconds)
            accel: (3,) raw accelerometer in body FRD [m/s²]
            gyro: (3,) raw gyroscope in body FRD [rad/s]

        Prediction model (NED world frame):
            p_{k+1} = p_k + v_k·dt + 0.5·(R_NB·a_corrected + g_NED)·dt²
            v_{k+1} = v_k + (R_NB·a_corrected + g_NED)·dt
            q_{k+1} = q_k ⊗ Exp(ω_corrected·dt)

        For stationary FRD: accel=[0,0,-9.81], g=[0,0,+9.81]
            R·[0,0,-9.81] + [0,0,+9.81] = 0  ✓
        """
        R = self.quaternion_to_matrix(self.q)  # R_NB
        accel_unbiased = accel - self.ba       # body FRD
        gyro_unbiased = gyro - self.bg         # body FRD

        # Euler integration in NED
        self.p = self.p + self.v * dt + 0.5 * (R @ accel_unbiased + self.g) * dt**2
        self.v = self.v + (R @ accel_unbiased + self.g) * dt

        # Quaternion integration
        delta_theta = gyro_unbiased * dt
        delta_theta_norm = np.linalg.norm(delta_theta)
        if delta_theta_norm > 1e-8:
            axis = delta_theta / delta_theta_norm
            angle = delta_theta_norm
            dq = np.array([np.cos(angle/2),
                           axis[0]*np.sin(angle/2),
                           axis[1]*np.sin(angle/2),
                           axis[2]*np.sin(angle/2)])
            # q_new = q_old ⊗ dq
            w1, x1, y1, z1 = self.q
            w2, x2, y2, z2 = dq
            self.q = np.array([
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2
            ])
            self.q /= np.linalg.norm(self.q)

        # Error-State Covariance Propagation
        F_x = np.eye(15)
        F_x[0:3, 3:6] = np.eye(3) * dt
        F_x[3:6, 6:9] = -R @ self.skew(accel_unbiased) * dt
        F_x[3:6, 9:12] = -R * dt
        F_x[6:9, 6:9] = np.eye(3) - self.skew(gyro_unbiased) * dt
        F_x[6:9, 12:15] = -np.eye(3) * dt

        F_i = np.zeros((15, 12))
        F_i[3:6, 0:3] = -R * dt
        F_i[6:9, 3:6] = -np.eye(3) * dt
        F_i[9:12, 6:9] = np.eye(3) * dt
        F_i[12:15, 9:12] = np.eye(3) * dt

        self.P = F_x @ self.P @ F_x.T + F_i @ self.Q_c @ F_i.T
        # Enforce symmetry
        self.P = 0.5 * (self.P + self.P.T)

    def update_velocity(self, v_body: np.ndarray, P_v: np.ndarray,
                        mahalanobis_thresh: float = 3.0) -> Tuple[bool, str, np.ndarray, np.ndarray, np.ndarray, float]:
        """Update state with body-frame velocity measurement.

        Args:
            v_body: (3,) velocity measured in body FRD frame [m/s]
            P_v: (3, 3) measurement covariance in body FRD
            mahalanobis_thresh: innovation gate threshold

        The observation model:
            z = v_B = R_NB^T · v_NED
        """
        R = self.quaternion_to_matrix(self.q)  # R_NB
        v_pred = R.T @ self.v                  # predicted body velocity

        innovation = v_body - v_pred

        # Jacobian H (3x15): δz = H · δx
        H = np.zeros((3, 15))
        H[:, 3:6] = R.T                        # ∂z/∂δv
        H[:, 6:9] = R.T @ self.skew(self.v)    # ∂z/∂δθ

        S = H @ self.P @ H.T + P_v

        # Mahalanobis distance
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False, "S_singular", innovation, S, np.zeros((15, 3)), np.inf

        maha_sq = innovation.T @ S_inv @ innovation
        maha = np.sqrt(maha_sq)

        if maha > mahalanobis_thresh:
            return False, f"mahalanobis_too_large ({maha:.2f} > {mahalanobis_thresh})", innovation, S, np.zeros((15, 3)), maha

        K = self.P @ H.T @ S_inv
        delta_x = K @ innovation

        self._inject_error_state(delta_x)

        # Joseph form covariance update
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ P_v @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        return True, "success", innovation, S, K, maha

    def update_altimeter(self, agl_m: float, var_h: float = 0.1**2,
                         mahalanobis_thresh: float = 3.0) -> Tuple[bool, str]:
        """Update state with altimeter AGL measurement.

        In NED: p[2] = Down = -AGL (altitude above ground is negative Z).
        Measurement model: z_agl = -p_NED[2]
        So: H = [0, 0, -1, 0...0] (row vector, 1x15)

        Args:
            agl_m: above-ground-level altitude (positive up) [metres]
            var_h: measurement variance [m²]
        """
        z_pred = -self.p[2]  # predicted AGL from NED state
        innovation = agl_m - z_pred

        H = np.zeros((1, 15))
        H[0, 2] = -1.0  # ∂z_agl / ∂p_D = -1

        S = H @ self.P @ H.T + var_h
        S_inv = 1.0 / S[0, 0]

        maha_sq = innovation**2 * S_inv
        maha = np.sqrt(maha_sq)

        if maha > mahalanobis_thresh:
            return False, f"altimeter_mahalanobis_too_large ({maha:.2f})"

        K = self.P @ H.T * S_inv
        delta_x = (K * innovation).flatten()

        self._inject_error_state(delta_x)

        # Joseph form
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K.reshape(-1, 1) @ (np.array([[var_h]])) @ K.reshape(1, -1)
        self.P = 0.5 * (self.P + self.P.T)

        return True, "success"

    def _inject_error_state(self, delta_x: np.ndarray):
        """Inject error state into nominal state."""
        self.p += delta_x[0:3]
        self.v += delta_x[3:6]

        # Orientation correction via small-angle quaternion
        delta_theta = delta_x[6:9]
        angle = np.linalg.norm(delta_theta)
        if angle > 1e-8:
            axis = delta_theta / angle
            dq = np.array([np.cos(angle/2),
                           axis[0]*np.sin(angle/2),
                           axis[1]*np.sin(angle/2),
                           axis[2]*np.sin(angle/2)])
            w1, x1, y1, z1 = self.q
            w2, x2, y2, z2 = dq
            self.q = np.array([
                w1*w2 - x1*x2 - y1*y2 - z1*z2,
                w1*x2 + x1*w2 + y1*z2 - z1*y2,
                w1*y2 - x1*z2 + y1*w2 + z1*x2,
                w1*z2 + x1*y2 - y1*x2 + z1*w2
            ])
            self.q /= np.linalg.norm(self.q)

        self.ba += delta_x[9:12]
        self.bg += delta_x[12:15]
