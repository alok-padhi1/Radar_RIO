#!/usr/bin/env python3
"""
nav_node.py
Closed-loop waypoint navigation in a GPS-denied local frame, using the
radar-SLAM pose (slam_node.py, UDP pose port) as the primary position
source, RIO velocity (doppler_rio.py, UDP forward port) for high-rate
dead-reckoning between SLAM updates, and the SLAM node's forward-cone
obstacle range for reactive braking. Commands PX4 (OFFBOARD) or ArduPilot
(GUIDED) over MAVLink via SET_POSITION_TARGET_LOCAL_NED velocity setpoints.

THIS SCRIPT ARMS, CHANGES FLIGHT MODE, AND SENDS VELOCITY SETPOINTS. That is
a fundamentally different trust level than mavlink_bridge.py (which only
ever pushes aiding data). Read NAVIGATION.md before running this against a
real airframe. Minimum non-negotiable preconditions:
  - Validated in PX4/ArduPilot SITL first, extensively, not skipped.
  - A safety pilot with an independent RC override capable of retaking
    manual control at any moment; this script's OFFBOARD/GUIDED commands
    are always preemptable by a mode-switch on the transmitter.
  - The autopilot's OWN failsafes (geofence, RC-loss, battery) configured
    and enabled independently -- this script's internal failsafes below are
    a second layer, not a replacement for the autopilot's.
  - Local regulatory compliance for the flight (this script does not know
    or enforce airspace rules).

Local navigation frame: right-handed ENU (x=East, y=North, z=Up), all
waypoints and the position estimate expressed relative to the pose the
vehicle had when nav_node started (NOT true GPS coordinates -- there is no
GPS in this loop by design). SLAM's T_world is body->world in whatever
frame slam_node.py's world origin was initialized in; this script treats
that origin as the nav frame origin directly.
"""

import argparse
import math
import socket
import struct
import sys
import threading
import time
import queue
from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np
from pymavlink import mavutil

RIO_PKT = struct.Struct('<dfffIfff')          # t, vx, vy, vz, n_inliers, cxx, cyy, czz
POSE_PKT_HDR = struct.Struct('<dId')       # t, n_map_points, fwd_range ; + 16 float64 T


# --------------------------------------------------------------- attitude math

def body_to_nav_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """SO(3) body-to-nav rotation: R_nav_body = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    All angles in radians. Convention: ZYX intrinsic = XYZ extrinsic.
    This is the same rotation the autopilot's own EKF uses internally;
    we're just reading its result via ATTITUDE rather than re-deriving it
    from raw IMU data.

    v_nav = R_nav_body @ v_body
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [  -sp,           cp*sr,            cp*cr   ],
    ])


@dataclass
class AttitudeState:
    roll: float = 0.0     # radians
    pitch: float = 0.0    # radians
    yaw: float = 0.0      # radians
    t_local: float = 0.0  # time.monotonic() at last update
    valid: bool = False


class AttitudeListener(threading.Thread):
    """Background thread that drains ATTITUDE (#30) messages from the
    MAVLink connection and stores the latest (roll, pitch, yaw). The
    autopilot sends ATTITUDE at 10-100 Hz depending on stream config;
    this thread just needs to keep up and not block the control loop.

    Shares the same mavutil connection as Autopilot -- pymavlink's
    recv_match is thread-safe for reading when only one thread reads.
    We use a separate connection here to avoid contention with the
    control loop's potential MAVLink sends."""

    STALE_TIMEOUT_S = 2.0  # if no ATTITUDE for this long, mark invalid

    def __init__(self, dest: str, baud: int, sysid: int, compid: int):
        super().__init__(daemon=True, name="AttitudeListener")
        self.state = AttitudeState()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        # Separate MAVLink connection for reading ATTITUDE, avoids
        # contention with the control loop's send connection.
        self._conn = mavutil.mavlink_connection(dest, baud=baud,
                                                source_system=sysid,
                                                source_component=compid + 1)
        self._conn.wait_heartbeat(timeout=15)
        # Request ATTITUDE stream at 50 Hz
        self._conn.mav.request_data_stream_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 50, 1)
        print(f"[nav_node] AttitudeListener: heartbeat OK, requesting ATTITUDE stream")

    def run(self):
        while not self._stop.is_set():
            msg = self._conn.recv_match(type=['ATTITUDE', 'EKF_STATUS_REPORT'],
                                        blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == 'ATTITUDE':
                with self._lock:
                    self.state.roll = msg.roll
                    self.state.pitch = msg.pitch
                    self.state.yaw = msg.yaw
                    self.state.t_local = time.monotonic()
                    self.state.valid = True
            else:  # EKF_STATUS_REPORT
                with self._lock:
                    # ArduPilot: variances are normalised; >1.0 means the EKF
                    # is rejecting/struggling with that measurement class.
                    self.ekf_vel_var = msg.velocity_variance
                    self.ekf_pos_var = msg.pos_horiz_variance
                    self.ekf_hgt_var = msg.pos_vert_variance
                    self.ekf_flags = msg.flags
                    self.ekf_t = time.monotonic()

    def ekf_healthy(self, var_limit: float = 0.9) -> bool:
        with self._lock:
            if time.monotonic() - getattr(self, 'ekf_t', 0.0) > 3.0:
                return False                      # no report = not healthy
            return (self.ekf_vel_var < var_limit and
                    self.ekf_pos_var < var_limit and
                    self.ekf_hgt_var < var_limit)

    def stop(self):
        self._stop.set()

    def get(self) -> AttitudeState:
        with self._lock:
            s = AttitudeState(
                roll=self.state.roll, pitch=self.state.pitch,
                yaw=self.state.yaw, t_local=self.state.t_local,
                valid=self.state.valid)
        # Mark stale if no update recently
        if s.valid and (time.monotonic() - s.t_local) > self.STALE_TIMEOUT_S:
            s.valid = False
        return s


# --------------------------------------------------------------- state / config

class NavState(Enum):
    INIT = auto()
    WAIT_FOR_POSE = auto()
    HOLD = auto()          # armed, offboard, commanding zero velocity -- safe default
    MISSION = auto()       # actively flying the waypoint queue
    OBSTACLE_STOP = auto()  # forward range breached hard stop, holding position
    FAILSAFE_RTL = auto()
    EXECUTING_RTH = auto() # autonomous return home via breadcrumbs
    LANDING = auto()
    DISARMED = auto()


@dataclass
class NavConfig:
    max_speed_mps: float = 2.0
    accel_limit_mps2: float = 1.0          # simple velocity-command slew limit
    waypoint_accept_radius_m: float = 1.5
    yaw_rate_max_dps: float = 30.0

    # Reactive obstacle avoidance off the SLAM node's forward-cone range.
    obstacle_brake_start_m: float = 15.0   # start scaling speed down
    obstacle_hard_stop_m: float = 5.0      # zero forward velocity, hold

    # Failsafes internal to this script (see module docstring: these are a
    # SECOND layer, the autopilot's own failsafes are the primary one).
    pose_stale_timeout_s: float = 1.0      # no SLAM pose update -> HOLD, then RTL
    pose_lost_rth_timeout_s: float = 2.0
    pose_lost_land_timeout_s: float = 6.0
    failsafe_descend_mps: float = 0.8
    alt_source_stale_timeout_s: float = 0.5   # altimeter staleness gate
    max_radius_from_origin_m: float = 150.0
    # HARD CEILING. See implementation_plan.md Section 2.6: ground returns
    # vanish above R_gnd, and R_gnd for diffuse terrain is ~120 m, not the
    # datasheet's 350 m (which is a high-RCS point-target figure).
    # 50 m is the safe value for a 40 deg tilt with R_gnd = 120 m.
    # DO NOT raise this without re-running TEST-01 at the new altitude.
    max_altitude_agl_m: float = 50.0
    min_altitude_agl_m: float = 2.0
    min_mission_start_agl_m: float = 3.0
    max_climb_mps: float = 1.0
    max_descend_mps: float = 0.7

    control_rate_hz: float = 10.0


@dataclass
class Waypoint:
    x: float   # East, m, relative to nav-frame origin
    y: float   # North, m
    z: float   # Up, m
    yaw_deg: float = float('nan')  # NaN = don't care / face direction of travel


# --------------------------------------------------------------- pose/velocity ingest

@dataclass
class PoseState:
    t_local: float = 0.0     # time.monotonic() this was last updated
    t_slam: float = 0.0      # slam_node's frame timestamp
    position_nav: np.ndarray = field(default_factory=lambda: np.zeros(3))
    fwd_obstacle_range_m: float = float('inf')
    have_pose: bool = False
    agl_m: float = float('nan')          # absolute AGL from U200A belly radar
    t_agl_local: float = 0.0


class PoseTracker:
    """Fuses SLAM's ~5Hz absolute pose with RIO's 20Hz body-frame velocity
    via simple dead-reckoning between SLAM updates -- NOT a Kalman filter.
    This is intentionally simple: the monograph's 15-state EKF (Sec. 1.6) is
    the correct production answer, but wiring a full EKF is a separate,
    larger task from 'get navigation flying.' Treat this as the placeholder
    to replace once Stage B SLAM/EKF work lands (see ARCHITECTURE.md).

    ATTITUDE ROTATION: body-frame velocity from RIO is rotated into the nav
    (ENU) frame using the autopilot's own roll/pitch/yaw estimate via
    body_to_nav_rotation() before integrating. If attitude is unavailable or
    stale, dead-reckoning is skipped entirely rather than silently integrating
    in the wrong frame — a brief pose-staleness event is far better than a
    confident, wrong position estimate that sends the vehicle off course."""

    def __init__(self, cfg: NavConfig):
        self.yaw_offset = 0.0
        self._yaw_offset_locked = False
        self.cfg = cfg
        self.pose = PoseState()
        self._last_rio_t = None
        self.start_position_nav = None
        self.breadcrumb_trail = []
        self._breadcrumb_lock = threading.Lock()

    def _update_breadcrumbs(self):
        if self.start_position_nav is None:
            self.start_position_nav = self.pose.position_nav.copy()
            with self._breadcrumb_lock:
                self.breadcrumb_trail.append(self.start_position_nav)
            return

        with self._breadcrumb_lock:
            if len(self.breadcrumb_trail) == 0:
                last_bc = self.start_position_nav
            else:
                last_bc = self.breadcrumb_trail[-1]
            
            # 2D displacement (X/Y)
            dist_2d = math.hypot(self.pose.position_nav[0] - last_bc[0], 
                                 self.pose.position_nav[1] - last_bc[1])
            if dist_2d > 2.0:
                self.breadcrumb_trail.append(self.pose.position_nav.copy())

    def get_displacement_m(self) -> float:
        if self.start_position_nav is None:
            return 0.0
        return np.linalg.norm(self.pose.position_nav - self.start_position_nav)

    def on_slam_pose(self, t_slam: float, T_world_body: np.ndarray, fwd_range: float, attitude: AttitudeState | None = None):
        p_nav = T_world_body[:3, 3].copy()
        
        # SLAM frame's yaw relative to the initial frame
        slam_yaw = math.atan2(T_world_body[1, 0], T_world_body[0, 0])
        if attitude is not None and attitude.valid and not self._yaw_offset_locked:
            self.yaw_offset = attitude.yaw - slam_yaw
            self._yaw_offset_locked = True
            print(f"[nav_node] yaw_offset LATCHED at {math.degrees(self.yaw_offset):+.1f} deg")
            
        self.pose.position_nav = p_nav
        self.pose.fwd_obstacle_range_m = fwd_range
        self.pose.t_slam = t_slam
        self.pose.t_local = time.monotonic()
        self.pose.have_pose = True
        self._last_rio_t = None
        self._update_breadcrumbs()

    def on_rio_velocity(self, t: float, v_body: np.ndarray,
                         attitude: AttitudeState | None = None):
        """Dead-reckon the position estimate forward using RIO velocity,
        rotating v_body from body frame into nav (ENU) frame using the
        autopilot's own attitude estimate.

        If attitude is None or stale, dead-reckoning is SKIPPED — we
        refuse to integrate a body-frame vector as if it were nav-frame,
        because that causes position drift proportional to sin(yaw_error)
        times speed, which is catastrophic during turns."""
        if attitude is None or not attitude.valid:
            # No usable attitude — skip dead-reckoning entirely.
            # pose.t_local is NOT updated, so stale_for() will eventually
            # trigger the HOLD/RTL cascade. That's correct: flying blind
            # on stale data is worse than briefly holding position.
            self._last_rio_t = t
            return

        nav_yaw = attitude.yaw - self.yaw_offset
        R = body_to_nav_rotation(attitude.roll, attitude.pitch, nav_yaw)
        v_nav = R @ v_body

        if self._last_rio_t is not None and self.pose.have_pose:
            dt = t - self._last_rio_t
            if 0 < dt < 0.5:
                self.pose.position_nav = self.pose.position_nav + v_nav * dt
                self.pose.t_local = time.monotonic()
                self._update_breadcrumbs()
        self._last_rio_t = t

    def on_altimeter(self, agl_m: float, attitude: AttitudeState | None):
        """Absolute AGL from the U200A belly radar, tilt-compensated.
        The belly radar measures SLANT range along body -Z; at a roll/pitch
        attitude the true vertical height is range * cos(roll) * cos(pitch).
        Over ~15 deg of tilt this is a 3.4% correction -- 1.7 m at 50 m AGL."""
        if attitude is not None and attitude.valid:
            agl_m = agl_m * math.cos(attitude.roll) * math.cos(attitude.pitch)
        self.pose.agl_m = agl_m
        self.pose.t_agl_local = time.monotonic()

    def agl(self) -> float:
        """Returns tilt-compensated AGL, or NaN if the altimeter is stale."""
        if not math.isfinite(self.pose.agl_m):
            return float('nan')
        if time.monotonic() - self.pose.t_agl_local > self.cfg.alt_source_stale_timeout_s:
            return float('nan')
        return self.pose.agl_m

    def stale_for(self) -> float:
        if not self.pose.have_pose:
            return float('inf')
        return time.monotonic() - self.pose.t_local


# --------------------------------------------------------------- MAVLink interface

class Autopilot:
    def __init__(self, dest: str, baud: int, sysid: int, compid: int,
                 platform: str):
        self.platform = platform  # 'px4' or 'ardupilot'
        self.conn = mavutil.mavlink_connection(dest, baud=baud,
                                                source_system=sysid,
                                                source_component=compid)
        print(f"[nav_node] waiting for heartbeat on {dest} ...")
        self.conn.wait_heartbeat(timeout=15)
        print(f"[nav_node] heartbeat OK sys={self.conn.target_system} "
              f"comp={self.conn.target_component}")

    def set_mode(self, mode_name: str):
        mode_id = self.conn.mode_mapping().get(mode_name)
        if mode_id is None:
            raise RuntimeError(f"Unknown mode '{mode_name}' for this autopilot/dialect")
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

    def arm(self, arm: bool = True):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1 if arm else 0, 0, 0, 0, 0, 0, 0)

    def is_armed(self) -> bool:
        return (self.conn.flightmode != '') and (self.conn.motors_armed())

    def send_velocity_setpoint(self, vx_ned: float, vy_ned: float, vz_ned: float,
                                yaw_rate_rad_s: float = 0.0):
        """SET_POSITION_TARGET_LOCAL_NED, velocity-only (position bits ignored
        via type_mask), yaw-rate control."""
        # POSITION_TARGET_TYPEMASK: 1 = IGNORE that field.
        #   bits 0-2  position   -> 1,1,1 ignore
        #   bits 3-5  velocity   -> 0,0,0 USE
        #   bits 6-8  accel      -> 1,1,1 ignore
        #   bit  9    force flag -> 0
        #   bit 10    yaw        -> 1 ignore   (we do NOT command absolute heading)
        #   bit 11    yaw_rate   -> 0 USE
        # = 0b0000_0101_1100_0111 = 0x05C7 = 1479
        type_mask = 0b0000_0101_1100_0111   # 1479: velocity + yaw_rate
        self.conn.mav.set_position_target_local_ned_send(
            0, self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask,
            0, 0, 0,
            vx_ned, vy_ned, vz_ned,
            0, 0, 0,
            0, yaw_rate_rad_s,
        )

    def request_rtl(self):
        mode = 'RTL' if self.platform == 'ardupilot' else 'AUTO.RTL'
        try:
            self.set_mode(mode)
        except RuntimeError:
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)

    def request_land(self):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

    def offboard_or_guided_mode(self):
        self.set_mode('OFFBOARD' if self.platform == 'px4' else 'GUIDED')


# --------------------------------------------------------------- controller

class VelocitySlewLimiter:
    def __init__(self, accel_limit: float):
        self.accel_limit = accel_limit
        self.v_cmd = np.zeros(3)
        self._last_t = None

    def step(self, v_target: np.ndarray, t: float) -> np.ndarray:
        if self._last_t is None:
            self._last_t = t
            self.v_cmd = v_target.copy()
            return self.v_cmd
        dt = max(t - self._last_t, 1e-3)
        self._last_t = t
        max_delta = self.accel_limit * dt
        delta = v_target - self.v_cmd
        norm = np.linalg.norm(delta)
        if norm > max_delta:
            delta = delta * (max_delta / norm)
        self.v_cmd = self.v_cmd + delta
        return self.v_cmd


def velocity_toward(current: np.ndarray, target: np.ndarray, max_speed: float,
                    max_climb: float = 1.0, max_descend: float = 0.7) -> np.ndarray:
    """Proportional guidance with SEPARATE horizontal and vertical speed caps.
    Vertical is capped much harder than horizontal because (a) at a 40 deg
    tilt the Vx/Vz correlation is -0.964, so fast vertical motion degrades the
    horizontal solution, and (b) climbing pushes the ground out of the radar's
    usable range (Sec 2.5). Descend is capped tighter still -- vortex ring
    state plus the altimeter's 0.2 m min range at touchdown."""
    err = target - current
    err_h = err[:2]
    dist_h = float(np.linalg.norm(err_h))
    v = np.zeros(3)
    if dist_h > 1e-6:
        speed_h = min(max_speed, max_speed * min(1.0, dist_h / 3.0))
        v[:2] = (err_h / dist_h) * speed_h
    # Z is DOWN-positive in the map frame; err[2] > 0 means the target is below us.
    dz = err[2]
    vz_cap = max_descend if dz > 0 else max_climb
    v[2] = float(np.clip(dz * 0.5, -vz_cap, vz_cap))   # P-controller, capped
    return v


def apply_obstacle_scaling(v_cmd_nav: np.ndarray, heading_nav: np.ndarray,
                            fwd_range_m: float, cfg: NavConfig) -> tuple[np.ndarray, bool]:
    """Scales down (or zeros) the forward component of the commanded
    velocity based on the SLAM node's forward-cone obstacle range. Returns
    (adjusted_velocity, hard_stop_triggered)."""
    # slam_node.py sends -1.0 as the "nothing detected in the forward cone"
    # sentinel (it cannot put +inf on the wire). Treat any non-positive value
    # as "clear", NOT as a 1-metre obstacle.
    if (not math.isfinite(fwd_range_m)) or fwd_range_m <= 0.0:
        return v_cmd_nav, False
    if fwd_range_m <= cfg.obstacle_hard_stop_m:
        # Zero the along-heading component; allow lateral/vertical motion to
        # continue so the vehicle can still be commanded to sidestep.
        h_norm = np.linalg.norm(heading_nav)
        if h_norm < 1e-6:
            return np.zeros(3), True
        h = heading_nav / h_norm
        along = np.dot(v_cmd_nav, h)
        if along > 0:
            v_cmd_nav = v_cmd_nav - along * h
        return v_cmd_nav, True
    if fwd_range_m <= cfg.obstacle_brake_start_m:
        scale = (fwd_range_m - cfg.obstacle_hard_stop_m) / (
            cfg.obstacle_brake_start_m - cfg.obstacle_hard_stop_m)
        h_norm = np.linalg.norm(heading_nav)
        if h_norm > 1e-6:
            h = heading_nav / h_norm
            along = np.dot(v_cmd_nav, h)
            if along > 0:
                v_cmd_nav = v_cmd_nav - along * (1.0 - scale) * h
    return v_cmd_nav, False


# --------------------------------------------------------------- RTH Planner

class RTHPlanner(threading.Thread):
    def __init__(self, tracker: PoseTracker):
        super().__init__(daemon=True, name="RTHPlanner")
        self.tracker = tracker
        self.return_waypoints = queue.Queue()
        self._trigger = threading.Event()

    def trigger_rth(self):
        self._trigger.set()

    def run(self):
        while True:
            self._trigger.wait()
            self._trigger.clear()
            
            with self.tracker._breadcrumb_lock:
                trail = list(self.tracker.breadcrumb_trail)
                start_pos = self.tracker.start_position_nav
            
            if start_pos is None:
                # Empty queue to signal failure
                continue
                
            current_z = self.tracker.pose.position_nav[2]

            # Use raw breadcrumbs in reverse to prevent cutting corners
            reversed_trail = list(reversed(trail))
            
            # Apply Z-axis safety: fly flat at current altitude
            safe_trail = []
            for wp in reversed_trail:
                safe_wp = wp.copy()
                safe_wp[2] = current_z
                safe_trail.append(safe_wp)
                
            # Append the start position with safe Z
            safe_start = start_pos.copy()
            safe_start[2] = current_z
            safe_trail.append(safe_start)
            
            # Empty existing queue
            while not self.return_waypoints.empty():
                self.return_waypoints.get()
                
            for wp in safe_trail:
                # Convert numpy array to Waypoint object. Waypoint expects Z=Up (so -NED_Z)
                self.return_waypoints.put(Waypoint(x=wp[0], y=wp[1], z=-wp[2]))


# --------------------------------------------------------------- main loop

class NavNode:
    def __init__(self, ap: Autopilot, cfg: NavConfig, waypoints: list[Waypoint],
                 pose_ip: str, pose_port: int, rio_ip: str, rio_port: int,
                 alt_ip: str = '127.0.0.1', alt_port: int = 5031,
                 attitude_listener: AttitudeListener | None = None):
        self.ap = ap
        self.cfg = cfg
        self.waypoints = waypoints
        self.wp_index = 0
        self.tracker = PoseTracker(cfg)
        self.state = NavState.INIT
        self.slew = VelocitySlewLimiter(cfg.accel_limit_mps2)
        self.origin_locked_z = None
        self.attitude_listener = attitude_listener

        self.pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pose_sock.bind((pose_ip, pose_port))
        self.pose_sock.setblocking(False)

        self.rio_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rio_sock.bind((rio_ip, rio_port))
        self.rio_sock.setblocking(False)

        self.alt_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.alt_sock.bind((alt_ip, alt_port))       # default 127.0.0.1:5031
        self.alt_sock.setblocking(False)

        self._t_state_enter = time.monotonic()
        self._rtl_sent = False

    # -- ingest --

    def _drain_sockets(self):
        # Fetch attitude FIRST so both the pose and the RIO handlers see the
        # same sample. This is the source of roll/pitch/yaw used to (a) resolve
        # the map->NED yaw offset and (b) rotate RIO body velocity into nav.
        att = self.attitude_listener.get() if self.attitude_listener else None

        try:
            while True:
                data, _ = self.pose_sock.recvfrom(2048)
                if len(data) < POSE_PKT_HDR.size + 128:
                    continue                      # short/truncated packet, drop it
                t, n_map, fwd_range = POSE_PKT_HDR.unpack_from(data, 0)
                T = np.frombuffer(data, dtype='<f8', count=16,
                                   offset=POSE_PKT_HDR.size).reshape(4, 4)
                self.tracker.on_slam_pose(t, T, fwd_range, att)   # <-- att was missing
        except BlockingIOError:
            pass
        except struct.error:
            pass                                   # malformed packet, drop it

        try:
            while True:
                data, _ = self.rio_sock.recvfrom(64)
                if len(data) < RIO_PKT.size:
                    continue
                t, vx, vy, vz, _n, _cxx, _cyy, _czz = RIO_PKT.unpack(data[:RIO_PKT.size])
                self.tracker.on_rio_velocity(t, np.array([vx, vy, vz]), att)
        except BlockingIOError:
            pass
        except struct.error:
            pass

        try:
            while True:
                data, _ = self.alt_sock.recvfrom(32)
                if len(data) >= 4:
                    (agl,) = struct.unpack('<f', data[:4])
                    self.tracker.on_altimeter(agl, att)
        except BlockingIOError:
            pass

    # -- safety checks, evaluated every control tick regardless of state --

    def _check_geofence(self) -> bool:
        p = self.tracker.pose.position_nav
        r_xy = math.hypot(p[0], p[1])
        if r_xy > self.cfg.max_radius_from_origin_m:
            print(f"[nav_node] GEOFENCE breach: r={r_xy:.1f}m > "
                  f"{self.cfg.max_radius_from_origin_m}m")
            return False
        alt_agl = self.tracker.agl()
        if not math.isfinite(alt_agl):
            # No trustworthy absolute height. SLAM Z is NOT an acceptable
            # fallback for a safety limit (documented 191 m drift in 96 s).
            print("[nav_node] ALTIMETER STALE -- no trustworthy AGL, failing geofence")
            return False
        if alt_agl > self.cfg.max_altitude_agl_m:
            print(f"[nav_node] ALTITUDE limit breach: agl={alt_agl:.1f}m > "
                  f"{self.cfg.max_altitude_agl_m}m")
            return False
        if self.state == NavState.MISSION and alt_agl < self.cfg.min_altitude_agl_m:
            print(f"[nav_node] ALTITUDE limit breach: agl={alt_agl:.1f}m < "
                  f"{self.cfg.min_altitude_agl_m}m")
            return False
        return True

    def _enter(self, state: NavState):
        if state != self.state:
            print(f"[nav_node] state {self.state.name} -> {state.name}")
            self.state = state
            self._t_state_enter = time.monotonic()

    def _begin_rth(self):
        print("[nav_node] Triggering autonomous breadcrumb RTH.")
        self.rth_planner.trigger_rth()
        self.rth_obstacle_block_start = None
        self._enter(NavState.RTH)

    def tick(self):
        self._drain_sockets()
        stale = self.tracker.stale_for()

        if self.state == NavState.INIT:
            if self.tracker.pose.have_pose:
                self._enter(NavState.HOLD)
            else:
                self._enter(NavState.WAIT_FOR_POSE)
            return

        if self.state == NavState.WAIT_FOR_POSE:
            if self.tracker.pose.have_pose:
                self._enter(NavState.HOLD)
            return

        # ---- Failsafe ladder (see implementation_plan.md Sec 8.3) ----
        # Tier 1: pose briefly stale -> stop moving, hold, keep trying.
        # Tier 2: pose lost longer than pose_lost_rth_timeout_s but the
        #         estimator is still healthy -> fly the breadcrumb home.
        # Tier 3: EKF unhealthy OR pose unrecoverable -> descend vertically
        #         on the altimeter. This is the ONLY action that does not
        #         require a trustworthy horizontal position.
        ekf_ok = True
        if self.attitude_listener:
            ekf_ok = self.attitude_listener.ekf_healthy()
        flying = self.state in (NavState.HOLD, NavState.MISSION,
                                NavState.OBSTACLE_STOP, NavState.RTH)

        if flying and not ekf_ok:
            print("[nav_node] EKF UNHEALTHY -- descending in place on altimeter")
            self._enter(NavState.FAILSAFE_LAND)

        if flying and stale > self.cfg.pose_lost_land_timeout_s:
            print(f"[nav_node] POSE LOST {stale:.1f}s -- descending in place")
            self._enter(NavState.FAILSAFE_LAND)
        elif flying and stale > self.cfg.pose_lost_rth_timeout_s \
                and self.state != NavState.RTH:
            print(f"[nav_node] POSE STALE {stale:.1f}s -- starting breadcrumb RTH")
            self._begin_rth()

        if self.state in (NavState.HOLD, NavState.MISSION, NavState.OBSTACLE_STOP):
            if not self._check_geofence():
                print("[nav_node] GEOFENCE BREACH -- descending in place")
                self._enter(NavState.FAILSAFE_LAND)

        if self.state == NavState.FAILSAFE_LAND:
            # Pure vertical descent using the belly altimeter. No horizontal
            # command at all -- we have explicitly decided we do not trust our
            # horizontal estimate, so we must not act on it.
            agl = self.tracker.agl()
            if math.isfinite(agl) and agl < 0.4:
                self.ap.send_velocity_setpoint(0, 0, 0)
                self.ap.arm(False)
                self._enter(NavState.DISARMED)
                return
            vz = self.cfg.failsafe_descend_mps     # +down in NED
            if math.isfinite(agl) and agl < 5.0:
                vz = 0.3                            # slow final
            self.ap.send_velocity_setpoint(0.0, 0.0, vz)
            return

        if self.state == NavState.RTH:
            if self.active_rth_waypoint is None:
                if self.rth_planner.return_waypoints.empty():
                    print("[nav_node] RTH complete. Holding over origin.")
                    self._enter(NavState.HOLD)
                    self.ap.send_velocity_setpoint(0, 0, 0)
                    return
                self.active_rth_waypoint = self.rth_planner.return_waypoints.get()
                if self.active_rth_waypoint is None:
                    # RTH failed
                    print("[nav_node] RTH failed to generate path. Descending in place.")
                    self._enter(NavState.FAILSAFE_LAND)
                    return

            wp = self.active_rth_waypoint
            target = np.array([wp.x, wp.y, -wp.z])  # Waypoint z is Up, target expects NED Down
            pos = self.tracker.pose.position_nav
            fwd_range = self.tracker.pose.fwd_obstacle_range_m
            
            dist = np.linalg.norm(target - pos)
            if dist < self.cfg.waypoint_accept_radius_m:
                print(f"[nav_node] RTH waypoint reached (dist={dist:.2f}m)")
                self.active_rth_waypoint = None
                return

            v_target = velocity_toward(pos, target, self.cfg.max_speed_mps,
                                       self.cfg.max_climb_mps, self.cfg.max_descend_mps)
            v_target, hard_stop = apply_obstacle_scaling(v_target, target - pos, fwd_range, self.cfg)
            v_cmd = self.slew.step(v_target, time.monotonic())
            
            if hard_stop:
                if self.rth_obstacle_block_start is None:
                    self.rth_obstacle_block_start = time.monotonic()
                elif (time.monotonic() - self.rth_obstacle_block_start) > 15.0:
                    print("[nav_node] RTH blocked for 15s. Executing FAILSAFE_LAND.")
                    self._enter(NavState.FAILSAFE_LAND)
                    return
                # Brake and hold
                self.ap.send_velocity_setpoint(0, 0, 0)
                return
            else:
                self.rth_obstacle_block_start = None

            yaw_off = self.tracker.yaw_offset
            cy, sy = math.cos(yaw_off), math.sin(yaw_off)
            v_cmd_ned = np.array([
                v_cmd[0] * cy - v_cmd[1] * sy,
                v_cmd[0] * sy + v_cmd[1] * cy,
                v_cmd[2]
            ])
            self.ap.send_velocity_setpoint(*v_cmd_ned)
            return

        if self.state == NavState.LANDING or self.state == NavState.DISARMED:
            return

        if stale > self.cfg.pose_stale_timeout_s:
            # Soft version: don't RTL yet, just freeze in place on the last
            # known-good position rather than continuing to command motion
            # off a stale/dead-reckoned estimate.
            self.ap.send_velocity_setpoint(0, 0, 0)
            return

        pos = self.tracker.pose.position_nav
        fwd_range = self.tracker.pose.fwd_obstacle_range_m

        if self.state == NavState.HOLD:
            self.ap.send_velocity_setpoint(0, 0, 0)
            return

        if self.state == NavState.MISSION:
            if self.wp_index >= len(self.waypoints):
                self._enter(NavState.HOLD)
                self.ap.send_velocity_setpoint(0, 0, 0)
                return
            wp = self.waypoints[self.wp_index]
            target = np.array([wp.x, wp.y, wp.z])
            dist = np.linalg.norm(target - pos)
            if dist < self.cfg.waypoint_accept_radius_m:
                print(f"[nav_node] waypoint {self.wp_index} reached (dist={dist:.2f}m)")
                self.wp_index += 1
                return

            v_target = velocity_toward(pos, target, self.cfg.max_speed_mps,
                                       self.cfg.max_climb_mps, self.cfg.max_descend_mps)
            v_target, hard_stop = apply_obstacle_scaling(v_target, target - pos, fwd_range, self.cfg)
            v_cmd = self.slew.step(v_target, time.monotonic())

            if hard_stop:
                self._enter(NavState.OBSTACLE_STOP)
            yaw_off = self.tracker.yaw_offset
            cy, sy = math.cos(yaw_off), math.sin(yaw_off)
            v_cmd_ned = np.array([
                v_cmd[0] * cy - v_cmd[1] * sy,
                v_cmd[0] * sy + v_cmd[1] * cy,
                v_cmd[2]
            ])
            self.ap.send_velocity_setpoint(*v_cmd_ned)
            return

        if self.state == NavState.OBSTACLE_STOP:
            self.ap.send_velocity_setpoint(0, 0, 0)
            if fwd_range > self.cfg.obstacle_brake_start_m:
                print("[nav_node] obstacle cleared, resuming mission")
                self._enter(NavState.MISSION)
            return

    def start_mission(self):
        if self.state != NavState.HOLD:
            print(f"[nav_node] refusing to start mission from state {self.state.name}")
            return
        if not self.ap.is_armed():
            print("[nav_node] REFUSING: vehicle is not armed. Take off manually in "
                  "LOITER/ALT_HOLD first, then issue 'go'. This script does not arm.")
            return
        agl = self.tracker.agl()
        if not math.isfinite(agl) or agl < self.cfg.min_mission_start_agl_m:
            print(f"[nav_node] REFUSING: AGL={agl} is not a valid airborne altitude "
                  f"(need > {self.cfg.min_mission_start_agl_m} m from the belly radar).")
            return
        if self.attitude_listener and not self.attitude_listener.ekf_healthy():
            print("[nav_node] REFUSING: EKF not healthy.")
            return
        if self.tracker.stale_for() > self.cfg.pose_stale_timeout_s:
            print("[nav_node] REFUSING: SLAM pose is stale.")
            return
        self.ap.offboard_or_guided_mode()      # mode change only. NO arm().
        self.home_map = self.tracker.pose.position_nav.copy()
        self._enter(NavState.MISSION)

    def run(self):
        period = 1.0 / self.cfg.control_rate_hz
        print("[nav_node] entering control loop. Type 'go' + Enter (on stdin, "
              "non-blocking check) to arm and start the mission once HOLD is reached.")
        import select
        while True:
            t0 = time.monotonic()
            self.tick()

            r, _, _ = select.select([sys.stdin], [], [], 0)
            if r:
                cmd = sys.stdin.readline().strip().lower()
                if cmd == 'go':
                    self.start_mission()
                elif cmd == 'land':
                    self.ap.request_land()
                    self._enter(NavState.LANDING)
                elif cmd == 'rtl':
                    self._enter(NavState.FAILSAFE_RTL)

            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, period - elapsed))


def parse_waypoints(spec: str) -> list[Waypoint]:
    """--waypoints 'x,y,z;x,y,z;...' in the SLAM nav frame, meters, relative
    to wherever slam_node.py's world origin was (i.e. vehicle start pose)."""
    wps = []
    for chunk in spec.split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        x, y, z = (float(v) for v in chunk.split(','))
        wps.append(Waypoint(x, y, -z))  # Z is UP in user input, but internal map frame is NED (Z-Down)
    return wps


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--mavlink-dest', default='udp:127.0.0.1:14540',
                    help="SITL default; use the real serial/UDP link for hardware")
    p.add_argument('--baud', type=int, default=921600)
    p.add_argument('--platform', choices=['px4', 'ardupilot'], default='px4')
    p.add_argument('--sysid', type=int, default=1)
    p.add_argument('--compid', type=int, default=191)  # MAV_COMP_ID_AUTOPILOT1-adjacent

    p.add_argument('--pose-ip', default='127.0.0.1')
    p.add_argument('--pose-port', type=int, default=5011)
    p.add_argument('--rio-ip', default='127.0.0.1')
    p.add_argument('--rio-port', type=int, default=5006)

    p.add_argument('--waypoints', required=True,
                    help="'x,y,z;x,y,z;...' SLAM frame meters relative to start pose, "
                         "e.g. '0,10,5;10,10,5;10,0,5;0,0,5'")
    p.add_argument('--max-speed', type=float, default=2.0)
    p.add_argument('--max-radius', type=float, default=150.0)
    p.add_argument('--max-altitude', type=float, default=50.0)

    args = p.parse_args()

    cfg = NavConfig(max_speed_mps=args.max_speed,
                     max_radius_from_origin_m=args.max_radius,
                     max_altitude_agl_m=args.max_altitude)
    waypoints = parse_waypoints(args.waypoints)
    if not waypoints:
        print("No waypoints parsed, exiting.")
        sys.exit(1)

    ap = Autopilot(args.mavlink_dest, args.baud, args.sysid, args.compid, args.platform)

    # Start the attitude listener — reads ATTITUDE messages from the
    # autopilot on a dedicated MAVLink connection so it doesn't contend
    # with the control loop's sends. This is the source of roll/pitch/yaw
    # used to rotate RIO body-frame velocity into the nav frame.
    att_listener = AttitudeListener(args.mavlink_dest, args.baud,
                                    args.sysid, args.compid)
    att_listener.start()

    node = NavNode(ap, cfg, waypoints, args.pose_ip, args.pose_port,
                    args.rio_ip, args.rio_port, attitude_listener=att_listener)
    node.run()


if __name__ == '__main__':
    main()
