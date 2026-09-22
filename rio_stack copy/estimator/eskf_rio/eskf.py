import numpy as np
from typing import Optional, Tuple
import logging

logger = logging.getLogger(__name__)

class ESKF:
    """
    Error-State Kalman Filter for Radar-Inertial Odometry.
    State: [p_W (3), v_W (3), q_WB (4), b_a (3), b_g (3)]
    Error State: [delta_p (3), delta_v (3), delta_theta (3), delta_b_a (3), delta_b_g (3)] (size 15)
    """
    def __init__(self, 
                 initial_p: np.ndarray,
                 initial_q: np.ndarray,
                 initial_v: np.ndarray,
                 initial_ba: np.ndarray,
                 initial_bg: np.ndarray):
        # Nominal state
        self.p = initial_p.copy()
        self.v = initial_v.copy()
        self.q = initial_q.copy() / np.linalg.norm(initial_q) # [w, x, y, z]
        self.ba = initial_ba.copy()
        self.bg = initial_bg.copy()

        self.g = np.array([0.0, 0.0, 9.80665]) # gravity vector (assuming Z-down)

        # Covariance matrix (15x15)
        self.P = np.eye(15) * 1e-4
        self.P[0:3, 0:3] *= 1e-6 # position
        self.P[3:6, 3:6] *= 1e-6 # velocity
        self.P[6:9, 6:9] *= 1e-6 # orientation
        self.P[9:12, 9:12] *= 1e-4 # accel bias
        self.P[12:15, 12:15] *= 1e-4 # gyro bias

        # Process noise continuous-time power spectral densities
        self.Q_c = np.zeros((12, 12))
        self.Q_c[0:3, 0:3] = np.eye(3) * 0.01**2  # accel noise
        self.Q_c[3:6, 3:6] = np.eye(3) * 0.005**2 # gyro noise
        self.Q_c[6:9, 6:9] = np.eye(3) * 0.001**2 # accel bias random walk
        self.Q_c[9:12, 9:12] = np.eye(3) * 0.0001**2 # gyro bias random walk

        self.last_time = None

    def quaternion_to_matrix(self, q: np.ndarray) -> np.ndarray:
        w, x, y, z = q
        return np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
        ])

    def matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
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
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])
        
    def predict(self, dt: float, accel: np.ndarray, gyro: np.ndarray):
        """
        Predict state using IMU measurements.
        accel: (3,) raw accelerometer [m/s^2] in body frame
        gyro: (3,) raw gyroscope [rad/s] in body frame
        """
        # Nominal State Kinematics
        R = self.quaternion_to_matrix(self.q)
        accel_unbiased = accel - self.ba
        gyro_unbiased = gyro - self.bg

        # Simple Euler integration
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
            # q_new = q_old * dq
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
        
        # enforce symmetry
        self.P = 0.5 * (self.P + self.P.T)

    def update_velocity(self, v_R: np.ndarray, P_v: np.ndarray, mahalanobis_thresh: float = 3.0) -> Tuple[bool, str, np.ndarray, np.ndarray, np.ndarray, float]:
        """
        Update state with Radar Ego Velocity.
        v_R: (3,) velocity measured in body frame
        P_v: (3, 3) measurement covariance
        Returns: (accepted, reason, innovation, S, K, mahalanobis)
        """
        # Observation model: z = v_B = R^T * v_W
        R = self.quaternion_to_matrix(self.q)
        v_pred = R.T @ self.v
        
        innovation = v_R - v_pred
        
        # Jacobian H (3x15)
        # delta_z = H * delta_x
        # v_B_true = R_true^T * v_W_true
        # v_B_true = (R * (I + [delta_theta]_x))^T * (v_W + delta_v)
        # v_B_true approx R^T * (I - [delta_theta]_x) * (v_W + delta_v)
        # v_B_true approx R^T * v_W + R^T * delta_v - R^T * [delta_theta]_x * v_W
        # v_B_true approx R^T * v_W + R^T * delta_v + R^T * [v_W]_x * delta_theta
        H = np.zeros((3, 15))
        H[:, 3:6] = R.T
        H[:, 6:9] = R.T @ self.skew(self.v)
        
        S = H @ self.P @ H.T + P_v
        
        # Mahalanobis distance check
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

    def update_altimeter(self, height_m: float, var_h: float = 0.1**2, mahalanobis_thresh: float = 3.0) -> Tuple[bool, str]:
        """
        Update state with altimeter height.
        height_m: absolute height (down is positive if z-down, or negative if z-up)
        Assume world Z is Down, height is -Z. 
        Actually, let's just assume height measures -p_W[2].
        """
        z_pred = -self.p[2]
        innovation = height_m - z_pred
        
        H = np.zeros((1, 15))
        H[0, 2] = -1.0
        
        S = H @ self.P @ H.T + var_h
        S_inv = 1.0 / S[0, 0]
        
        maha_sq = innovation**2 * S_inv
        maha = np.sqrt(maha_sq)
        
        if maha > mahalanobis_thresh:
            return False, f"altimeter_mahalanobis_too_large ({maha:.2f})"
            
        K = self.P @ H.T * S_inv
        delta_x = (K * innovation).flatten()
        
        self._inject_error_state(delta_x)
        
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T
        self.P[2, 2] += K[2, 0]**2 * var_h  # adding the KRK^T term for the non-zero element
        self.P = 0.5 * (self.P + self.P.T)
        
        return True, "success"
        
    def _inject_error_state(self, delta_x: np.ndarray):
        """ Inject error state into nominal state """
        self.p += delta_x[0:3]
        self.v += delta_x[3:6]
        
        # delta_theta
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
