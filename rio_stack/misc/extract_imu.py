#!/usr/bin/env python3
"""
extract_imu.py

Connects to a Pixhawk (e.g. Cube+) via USB (ACM) or serial using MAVLink
and extracts the fused Attitude (Roll/Pitch/Yaw) as well as the raw HighRes IMU data
(Accelerometer, Gyroscope, and Compass/Magnetometer).

Usage:
  python3 extract_imu.py
"""

import time
import argparse
try:
    from pymavlink import mavutil
except ImportError:
    print("Error: pymavlink is not installed. Please install it using:")
    print("pip install pymavlink")
    exit(1)

def main():
    # Hardcoded values for Pixhawk over USB
    PORT = '/dev/ttyACM0'
    BAUD = 115200

    print(f"Connecting to Pixhawk on {PORT} at {BAUD} baud...")
    
    # Create the connection
    try:
        master = mavutil.mavlink_connection(PORT, baud=BAUD)
    except Exception as e:
        print(f"Failed to connect to {PORT}: {e}")
        exit(1)
    
    # Wait for the first heartbeat to confirm connection
    print("Waiting for heartbeat (ensure Pixhawk is powered and plugged in)...")
    try:
        master.wait_heartbeat(timeout=10.0)
    except Exception as e:
        print("Timeout waiting for heartbeat. Check port and baud rate.")
        exit(1)
        
    print(f"Heartbeat received! Connected to System {master.target_system}, Component {master.target_component}")
    
    # Request data streams from the Pixhawk at 50Hz
    # EXTRA1 stream includes ATTITUDE
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 50, 1)
        
    # RAW_SENSORS stream includes HIGHRES_IMU (Accel, Gyro, Compass)
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 50, 1)

    print("\nReading IMU, Attitude, and Compass data. Press Ctrl+C to stop.\n")
    print("-" * 70)
    
    try:
        while True:
            # Block until we receive Attitude or some form of IMU message
            msg = master.recv_match(type=['ATTITUDE', 'HIGHRES_IMU', 'RAW_IMU', 'SCALED_IMU'], blocking=True)
            if not msg:
                continue
                
            msg_type = msg.get_type()
            
            if msg_type == 'ATTITUDE':
                import math
                r_deg = math.degrees(msg.roll)
                p_deg = math.degrees(msg.pitch)
                y_deg = math.degrees(msg.yaw)
                print("\n=== ATTITUDE (Fused EKF) ===")
                print(f"Roll:  {r_deg:>+7.2f}°")
                print(f"Pitch: {p_deg:>+7.2f}°")
                print(f"Yaw:   {y_deg:>+7.2f}°")
                
            elif msg_type == 'HIGHRES_IMU':
                print(f"\n=== {msg_type} ===")
                print(f"Accel   (m/s²) :  X: {msg.xacc:>+6.2f} | Y: {msg.yacc:>+6.2f} | Z: {msg.zacc:>+6.2f}")
                print(f"Gyro    (rad/s):  X: {msg.xgyro:>+6.2f} | Y: {msg.ygyro:>+6.2f} | Z: {msg.zgyro:>+6.2f}")
                print(f"Compass (Gauss):  X: {msg.xmag:>+6.2f} | Y: {msg.ymag:>+6.2f} | Z: {msg.zmag:>+6.2f}")
                
            elif msg_type in ['RAW_IMU', 'SCALED_IMU']:
                print(f"\n=== {msg_type} ===")
                # Note: RAW_IMU might be unscaled integers, SCALED_IMU is in milli-units (mG, mrad/s, mgauss)
                print(f"Accel :  X: {msg.xacc:>+6} | Y: {msg.yacc:>+6} | Z: {msg.zacc:>+6}")
                print(f"Gyro  :  X: {msg.xgyro:>+6} | Y: {msg.ygyro:>+6} | Z: {msg.zgyro:>+6}")
                print(f"Compass:  X: {msg.xmag:>+6} | Y: {msg.ymag:>+6} | Z: {msg.zmag:>+6}")
                
    except KeyboardInterrupt:
        print("\nStopping data stream.")
        
        # Stop requesting the data streams before exiting
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 0, 0)
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 0, 0)

if __name__ == '__main__':
    main()
