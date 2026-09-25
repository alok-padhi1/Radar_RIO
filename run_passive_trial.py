#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import subprocess
import signal
import sys
import os
import time
import datetime
import math

class SupervisorNode(Node):
    def __init__(self):
        super().__init__('passive_trial_supervisor')
        self.subscription = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10
        )
        self.get_logger().info('Subscribed to /Odometry. Monitoring position...')
        self.start_x = None
        self.start_y = None
        self.start_z = None
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_z = 0.0
        self.total_distance = 0.0

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.start_z = z
            self.last_x = x
            self.last_y = y
            self.last_z = z
            
        # Accumulate distance
        dx = x - self.last_x
        dy = y - self.last_y
        dz = z - self.last_z
        self.total_distance += math.sqrt(dx*dx + dy*dy + dz*dz)
        
        self.last_x = x
        self.last_y = y
        self.last_z = z

        sys.stdout.write(f"\r🚀 Current Position: X={x:7.3f} m | Y={y:7.3f} m | Z={z:7.3f} m | Dist={self.total_distance:7.3f} m   ")
        sys.stdout.flush()

bag_proc = None
launch_proc = None
bag_dir = ""
supervisor_node = None

def cleanup(sig, frame):
    print("\n\n" + "="*60)
    print("   🛑 CTRL+C DETECTED - SHUTTING DOWN SAFELY")
    print("="*60)
    
    if bag_proc:
        print("Stopping rosbag...")
        try:
            os.killpg(os.getpgid(bag_proc.pid), signal.SIGINT)
        except:
            pass
    if launch_proc:
        print("Stopping LIO nodes...")
        try:
            os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
        except:
            pass
        
    print("Waiting 4 seconds for safe file closure...")
    time.sleep(4)
    
    print("\n" + "="*60)
    print("                    FINAL RESULTS")
    print("="*60)
    if supervisor_node and supervisor_node.start_x is not None:
        dx = supervisor_node.last_x - supervisor_node.start_x
        dy = supervisor_node.last_y - supervisor_node.start_y
        dz = supervisor_node.last_z - supervisor_node.start_z
        displacement = math.sqrt(dx*dx + dy*dy + dz*dz)
        print(f"📍 Total Traveled Distance : {supervisor_node.total_distance:.3f} meters")
        print(f"📍 Final Displacement      : {displacement:.3f} meters (from start point)")
        print(f"   (If you returned to start, displacement should be close to 0.0)")
    else:
        print("No odometry data was received.")
    print("\n✅ Rosbag securely saved to:")
    print(f"   {bag_dir}")
    print("✅ FAST-LIO Submaps/Keyframes saved to:")
    print("   /mnt/nvme/maps/keyframes/  <-- You can open these .pcd files in CloudCompare!")
    print("✅ Map Manifest saved to:")
    print("   /mnt/nvme/maps/manifest.yaml")
    print("=========================================================\n")
    sys.exit(0)

def main():
    global bag_proc, launch_proc, bag_dir, supervisor_node
    signal.signal(signal.SIGINT, cleanup)

    print("=========================================================")
    print("   🚀 LIO PASSIVE TRIAL SUPERVISOR")
    print("=========================================================\n")
    
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_dir = os.path.expanduser(f"~/LIO/bags/passive_trial_{timestamp}")
    os.makedirs(os.path.expanduser("~/LIO/bags"), exist_ok=True)
    
    env = os.environ.copy()
    
    print("[1] Starting Livox Driver, FAST-LIO2, and Adapters in background...")
    launch_cmd = 'ros2 launch gps_denied_nav lio_passive.launch.py'
    launch_proc = subprocess.Popen(launch_cmd, shell=True, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    
    print("    Waiting 5 seconds for nodes to initialize...")
    time.sleep(5)
    
    print("[2] Starting Rosbag recording...")
    print(f"    Bag will be saved to: {bag_dir}")
    bag_cmd = f'ros2 bag record -o {bag_dir} /livox/lidar /livox/imu /Odometry /lio/health /keyframe /tf /tf_static'
    bag_proc = subprocess.Popen(bag_cmd, shell=True, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    
    time.sleep(2)
    
    print("\n[3] System is running! Monitoring Odometry...")
    print("    (Walk your 0.5m, 1m, 5m loops now)")
    print("    Press [CTRL+C] at any time to safely stop and save.")
    print("---------------------------------------------------------")
    
    rclpy.init()
    supervisor_node = SupervisorNode()
    try:
        rclpy.spin(supervisor_node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
