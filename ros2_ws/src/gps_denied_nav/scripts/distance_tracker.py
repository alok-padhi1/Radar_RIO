#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math
import numpy as np
import os
import struct

class DistanceTracker(Node):
    def __init__(self):
        super().__init__('distance_tracker')
        # Subscribe to the odometry topic published by FAST-LIO2
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10)
            
        # Subscribe to the point cloud topic to save PCD
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.cloud_callback,
            10)
            
        self.start_pos = None
        self.last_pos = None
        self.total_distance = 0.0
        
        self.total_points = 0
        self.temp_bin_path = '/tmp/fastlio_map_tmp.bin'
        self.final_pcd_path = 'FAST_LIO2_Map.pcd'
        
        # Open a temporary binary file to stream points so we don't run out of memory
        self.bin_file = open(self.temp_bin_path, 'wb')
        
        self.get_logger().info('Distance tracker & PCD saver started!')
        self.get_logger().info('Listening to /Odometry and /cloud_registered...')
        self.get_logger().info('Press Ctrl+C to stop, print summary, and compile the final PCD.')

    def odom_callback(self, msg):
        current_pos = msg.pose.pose.position
        
        if self.start_pos is None:
            self.start_pos = current_pos
            self.last_pos = current_pos
            return
            
        # Calculate distance from last position to current position
        dx = current_pos.x - self.last_pos.x
        dy = current_pos.y - self.last_pos.y
        dz = current_pos.z - self.last_pos.z
        
        step_distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Filter out high-frequency IMU jitter by requiring at least 2cm of movement
        if step_distance > 0.02:
            self.total_distance += step_distance
            self.last_pos = current_pos

    def cloud_callback(self, msg):
        # Extract X, Y, Z from the PointCloud2 message
        # FAST-LIO2 pointclouds are already registered to the global world frame
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        if hasattr(points, 'dtype') and points.dtype.names is not None:
            points_arr = np.column_stack((points['x'], points['y'], points['z'])).astype(np.float32)
        else:
            points_arr = np.array(list(points), dtype=np.float32)
        
        # Write binary floats directly to our temp file
        if points_arr.shape[0] > 0:
            self.bin_file.write(points_arr.tobytes())
            self.total_points += points_arr.shape[0]
            
        # Every 10,000 points, give a small log
        if self.total_points % 100000 < 5000:
            self.get_logger().info(f'Accumulated {self.total_points} points so far...')

    def save_pcd_and_print_summary(self):
        self.bin_file.close()
        
        # Write the final PCD file with the correct header
        print("\nSaving PCD file... please wait.")
        try:
            with open(self.final_pcd_path, 'wb') as f_pcd:
                # Standard PCD ASCII Header for binary data
                header = f"# .PCD v0.7 - Point Cloud Data file format\n"
                header += f"VERSION 0.7\n"
                header += f"FIELDS x y z\n"
                header += f"SIZE 4 4 4\n"
                header += f"TYPE F F F\n"
                header += f"COUNT 1 1 1\n"
                header += f"WIDTH {self.total_points}\n"
                header += f"HEIGHT 1\n"
                header += f"VIEWPOINT 0 0 0 1 0 0 0\n"
                header += f"POINTS {self.total_points}\n"
                header += f"DATA binary\n"
                
                f_pcd.write(header.encode('ascii'))
                
                # Append the binary data
                with open(self.temp_bin_path, 'rb') as f_bin:
                    f_pcd.write(f_bin.read())
            
            print(f"[Success] Saved {self.total_points} points to {os.path.abspath(self.final_pcd_path)}")
        except Exception as e:
            print(f"[Error] Failed to save PCD: {e}")
            
        # Clean up temp file
        if os.path.exists(self.temp_bin_path):
            os.remove(self.temp_bin_path)
            
        if self.start_pos is None or self.last_pos is None:
            print("\n[Summary] No odometry data received. Did you start FAST-LIO2?")
            return
            
        # Calculate total displacement from start to end
        disp_x = self.last_pos.x - self.start_pos.x
        disp_y = self.last_pos.y - self.start_pos.y
        disp_z = self.last_pos.z - self.start_pos.z
        displacement = math.sqrt(disp_x**2 + disp_y**2 + disp_z**2)
        
        print("\n" + "="*50)
        print("          MISSION SUMMARY REPORT")
        print("="*50)
        print(f"Total Distance Traveled : {self.total_distance:.2f} meters")
        print(f"Total Final Displacement: {displacement:.2f} meters")
        print(f"Start Position          : (X: {self.start_pos.x:.2f}, Y: {self.start_pos.y:.2f}, Z: {self.start_pos.z:.2f})")
        print(f"End Position            : (X: {self.last_pos.x:.2f}, Y: {self.last_pos.y:.2f}, Z: {self.last_pos.z:.2f})")
        print("="*50 + "\n")

def main(args=None):
    rclpy.init(args=args)
    tracker = DistanceTracker()
    
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.save_pcd_and_print_summary()
        tracker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
