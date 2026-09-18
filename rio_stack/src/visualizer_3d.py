#!/usr/bin/env python3
"""
visualizer_3d.py
Real-time 3D Trajectory & Point Cloud Visualizer for 4D mmWave Radar SLAM & RIO.

Subscribes to:
  - Port 5011: SLAM 3D Pose (T_world 4x4 matrix), map point count, forward obstacle range
  - Port 5012: Decimated radar point cloud (x, y, z, v) from radar_fanout
  - Port 5009: RIO 3D body velocity (vx, vy, vz, inliers) from doppler_rio

Features:
  1. Interactive 3D Open3D window displaying:
     - Real-time accumulated 3D world point cloud (voxel downsampled).
     - Live vehicle 3D walking trajectory path (colored line set).
     - Vehicle coordinate frame / position indicator at current SE(3) pose.
  2. Real-time Terminal Heads-Up Display (HUD) showing:
     - Velocity (vx, vy, vz, total speed in m/s)
     - Distance traveled (integrated RIO distance and SLAM path distance)
     - Current position [X, Y, Z] in meters
     - Forward obstacle distance
     - Keyframe count and accumulated map points
  3. Supports both GUI (interactive Open3D) and headless/CLI-only mode (--no-gui).

Usage:
  python3 visualizer_3d.py
  python3 visualizer_3d.py --no-gui
"""

import argparse
import math
import os
import socket
import struct
import sys
import time
import numpy as np
import open3d as o3d

UDP_HEADER = struct.Struct('<I')
# Extended RIO packet — must match doppler_rio.py FORWARD_PKT exactly (46 bytes)
RIO_PKT = struct.Struct('<dfffIfffIBf')
POSE_PKT_HDR = struct.Struct('<dId')  # t, n_map_points, fwd_obstacle_range ; + 16xfloat64


class TiltMount:
    def __init__(self, theta_tilt_deg: float = 40.0, lateral_sign: float = 1.0):
        th = math.radians(theta_tilt_deg)
        c, s = math.cos(th), math.sin(th)
        R_tilt = np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])
        P = np.array([
            [0.0,          1.0,  0.0],
            [lateral_sign, 0.0,  0.0],
            [0.0,          0.0, -1.0],
        ])
        self.R = R_tilt @ P

    def to_body(self, xyz_radar: np.ndarray) -> np.ndarray:
        return xyz_radar @ self.R.T


def parse_points_packet(data: bytes):
    if len(data) < 4:
        return None
    (n,) = UDP_HEADER.unpack_from(data, 0)
    expected = 4 + n * 16
    if n == 0 or len(data) < expected:
        return None
    pts = np.frombuffer(data, dtype='<f4', count=n * 4, offset=4)
    return pts.reshape(n, 4)


def parse_pose_packet(data: bytes):
    hdr_size = POSE_PKT_HDR.size
    if len(data) < hdr_size + 128:
        return None
    t, n_map, fwd_range = POSE_PKT_HDR.unpack_from(data, 0)
    T = np.frombuffer(data[hdr_size:hdr_size + 128], dtype='<f8').reshape(4, 4)
    return t, n_map, fwd_range, T


def parse_rio_packet(data: bytes):
    if len(data) < RIO_PKT.size:
        return None
    t, vx, vy, vz, inliers, _cxx, _cyy, _czz, _ntot, _flags, _cond = RIO_PKT.unpack_from(data, 0)
    return t, np.array([vx, vy, vz]), inliers


def main():
    p = argparse.ArgumentParser(description="4D Radar 3D Trajectory & Point Cloud Visualizer")
    p.add_argument('--pose-port', type=int, default=5011, help="SLAM pose UDP port")
    p.add_argument('--points-port', type=int, default=5012, help="Radar points UDP port (streamed by radar_fanout)")
    p.add_argument('--rio-port', type=int, default=5009, help="RIO velocity UDP port (streamed by doppler_rio)")
    p.add_argument('--tilt-deg', type=float, default=40.0, help="Radar pitch tilt angle in degrees")
    p.add_argument('--lateral-sign', type=float, default=1.0, help="Lateral axis sign")
    p.add_argument('--voxel-size', type=float, default=0.08, help="Map accumulation voxel size in meters")
    p.add_argument('--max-map-points', type=int, default=50000, help="Maximum accumulated map points")
    p.add_argument('--no-gui', action='store_true', help="Run in terminal HUD mode without 3D GUI window")
    args = p.parse_args()

    mount = TiltMount(theta_tilt_deg=args.tilt_deg, lateral_sign=args.lateral_sign)

    # Sockets with SO_REUSEPORT
    pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pose_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    pose_sock.bind(('127.0.0.1', args.pose_port))
    pose_sock.setblocking(False)

    points_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    points_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    points_sock.bind(('127.0.0.1', args.points_port))
    points_sock.setblocking(False)

    rio_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rio_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    rio_sock.bind(('127.0.0.1', args.rio_port))
    rio_sock.setblocking(False)

    print("=" * 70)
    print("4D mmWave Radar Real-Time Visualizer Initialized")
    print(f"  Listening: Pose (Port {args.pose_port}), Points (Port {args.points_port}), RIO (Port {args.rio_port})")
    print(f"  Tilt: {args.tilt_deg}° downward | GUI: {'OFF (Terminal HUD only)' if args.no_gui else 'ON (Open3D 3D Window)'}")
    print("=" * 70)

    # State tracking
    T_current = np.eye(4)
    trajectory = [T_current[:3, 3].copy()]
    map_pcd = o3d.geometry.PointCloud()
    live_pcd = o3d.geometry.PointCloud()
    traj_lines = o3d.geometry.LineSet()
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.4)

    v_body = np.zeros(3)
    rio_inliers = 0
    fwd_range_m = float('inf')
    n_map_reported = 0
    rio_pos = np.zeros(3)
    total_dist_rio = 0.0
    last_rio_t = None
    start_time = time.time()
    last_hud_print = time.time()

    # Setup GUI if enabled
    vis = None
    if not args.no_gui:
        has_display = bool(os.environ.get("DISPLAY"))
        if not has_display:
            print("[visualizer] No DISPLAY detected. Falling back to --no-gui terminal mode.")
            args.no_gui = True
        else:
            try:
                vis = o3d.visualization.Visualizer()
                vis.create_window(window_name="4D Radar SLAM Real-Time 3D Trajectory & Map", width=1280, height=800)
                vis.add_geometry(map_pcd)
                vis.add_geometry(live_pcd)
                vis.add_geometry(traj_lines)
                vis.add_geometry(axes)
                opt = vis.get_render_option()
                opt.background_color = np.array([0.08, 0.08, 0.10])
                opt.point_size = 4.0
                opt.line_width = 3.0
            except Exception as e:
                print(f"[visualizer] Failed to initialize GUI ({e}). Falling back to --no-gui mode.")
                args.no_gui = True
                vis = None

    first_view = True
    running = True

    try:
        while running:
            time.sleep(0.015)  # ~60 Hz loop

            # 1. Drain RIO velocity
            try:
                while True:
                    data, _ = rio_sock.recvfrom(64)
                    res = parse_rio_packet(data)
                    if res is not None:
                        t_rio, v_b, inliers = res
                        v_body = v_b
                        rio_inliers = inliers
                        if last_rio_t is not None:
                            dt = max(0.0, min(0.2, t_rio - last_rio_t))
                            rio_pos += v_body * dt
                            total_dist_rio = float(np.linalg.norm(rio_pos))
                        last_rio_t = t_rio
            except BlockingIOError:
                pass

            # 2. Drain SLAM poses
            pose_updated = False
            try:
                while True:
                    data, _ = pose_sock.recvfrom(1024)
                    res = parse_pose_packet(data)
                    if res is not None:
                        _t, n_map, fwd, T = res
                        T_current = T
                        n_map_reported = n_map
                        fwd_range_m = fwd
                        pos = T_current[:3, 3].copy()
                        if len(trajectory) == 0 or np.linalg.norm(pos - trajectory[-1]) > 0.02:
                            trajectory.append(pos)
                        pose_updated = True
            except BlockingIOError:
                pass

            # 3. Drain radar points
            pts_world_batch = []
            pts_colors_batch = []
            try:
                while True:
                    data, _ = points_sock.recvfrom(65535)
                    pts = parse_points_packet(data)
                    if pts is not None and len(pts) > 0:
                        xyz_b = mount.to_body(pts[:, :3])
                        v_rad = pts[:, 3]
                        # Transform to world frame
                        ones = np.ones((len(xyz_b), 1))
                        homo = np.hstack([xyz_b, ones])
                        world_pts = (homo @ T_current.T)[:, :3]
                        pts_world_batch.append(world_pts)

                        # Color by Doppler velocity
                        # Static (v ~ 0) = Green, Approaching (v < 0) = Blue, Receding (v > 0) = Red
                        colors = np.zeros((len(v_rad), 3))
                        for i, vi in enumerate(v_rad):
                            if abs(vi) < 0.20:
                                colors[i] = [0.2, 0.85, 0.4]  # Static ground
                            elif vi > 0:
                                colors[i] = [min(1.0, 0.4 + vi), 0.2, 0.2]  # Moving away (red)
                            else:
                                colors[i] = [0.2, 0.4, min(1.0, 0.4 - vi)]  # Moving closer (blue)
                        pts_colors_batch.append(colors)
            except BlockingIOError:
                pass

            # 4. Update 3D Geometry
            if pts_world_batch:
                new_pts = np.vstack(pts_world_batch)
                new_colors = np.vstack(pts_colors_batch)
                live_pcd.points = o3d.utility.Vector3dVector(new_pts)
                live_pcd.colors = o3d.utility.Vector3dVector(new_colors)

                # Merge into map cloud
                map_pcd.points.extend(live_pcd.points)
                map_pcd.colors.extend(live_pcd.colors)
                if len(map_pcd.points) > 1000:
                    map_pcd = map_pcd.voxel_down_sample(args.voxel_size)
                    if len(map_pcd.points) > args.max_map_points:
                        idx = np.random.choice(len(map_pcd.points), args.max_map_points, replace=False)
                        map_pcd = map_pcd.select_by_index(idx)

            if pose_updated and vis is not None:
                # Update trajectory line set
                if len(trajectory) >= 2:
                    points_arr = np.array(trajectory)
                    lines = [[i, i + 1] for i in range(len(points_arr) - 1)]
                    colors = [[1.0, 0.84, 0.0] for _ in lines]  # Gold trajectory line
                    traj_lines.points = o3d.utility.Vector3dVector(points_arr)
                    traj_lines.lines = o3d.utility.Vector2iVector(lines)
                    traj_lines.colors = o3d.utility.Vector3dVector(colors)

                # Update coordinate axes position
                axes.transform(np.linalg.inv(axes.get_rotation_matrix_from_xyz((0, 0, 0))))
                axes_curr = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.35)
                axes_curr.transform(T_current)
                axes.vertices = axes_curr.vertices

            if vis is not None:
                vis.update_geometry(map_pcd)
                vis.update_geometry(live_pcd)
                vis.update_geometry(traj_lines)
                vis.update_geometry(axes)
                if first_view and len(map_pcd.points) > 10:
                    vis.reset_view_point(True)
                    first_view = False
                vis.poll_events()
                vis.update_renderer()

            # 5. Terminal HUD (updates every 0.5 seconds)
            now = time.time()
            if now - last_hud_print >= 0.5:
                elapsed = now - start_time
                spd = float(np.linalg.norm(v_body))
                pos = T_current[:3, 3]
                slam_dist = float(np.linalg.norm(pos - trajectory[0])) if len(trajectory) > 0 else 0.0
                obs_str = f"{fwd_range_m:.1f}m" if math.isfinite(fwd_range_m) and fwd_range_m > 0 else "CLEAR"
                
                status_icon = "🟢 STATIC" if spd < 0.10 else f"🔵 MOVING ({spd:.2f} m/s)"
                sys.stdout.write(
                    f"\r\033[K[HUD {elapsed:5.1f}s] {status_icon:18s} | "
                    f"v_body=[{v_body[0]:+5.2f}, {v_body[1]:+5.2f}, {v_body[2]:+5.2f}] m/s | "
                    f"RIO Dist: {total_dist_rio:5.1f}m | SLAM Pos: [{pos[0]:+5.1f}, {pos[1]:+5.1f}, {pos[2]:+5.1f}]m (dist={slam_dist:4.1f}m) | "
                    f"Map: {len(map_pcd.points):4d} pts | Obstacle: {obs_str}"
                )
                sys.stdout.flush()
                last_hud_print = now

    except KeyboardInterrupt:
        print("\n[visualizer] Interrupted by user.")
    finally:
        if vis is not None:
            vis.destroy_window()
        pose_sock.close()
        points_sock.close()
        rio_sock.close()

        print("\n" + "=" * 70)
        print("RUN SUMMARY:")
        print(f"  Total Duration:         {time.time() - start_time:.1f} seconds")
        print(f"  Integrated RIO Dist:    {total_dist_rio:.2f} meters")
        final_pos = T_current[:3, 3]
        print(f"  Final SLAM Position:    [{final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f}] meters")
        print(f"  Net SLAM Displacement:  {np.linalg.norm(final_pos - trajectory[0]):.2f} meters")
        print(f"  Total Keyframe Steps:   {len(trajectory)}")
        print(f"  Accumulated Map Points: {len(map_pcd.points)}")
        print("=" * 70)


if __name__ == '__main__':
    main()
