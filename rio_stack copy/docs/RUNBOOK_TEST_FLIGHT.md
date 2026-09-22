# RIO Stack: Bench Testing & Flight Runbook

This document is the authoritative guide for launching the RIO stack for a **manual test flight**. This flight is meant to collect data safely, allowing us to compare the RIO (Radar) estimated path against the FC (Cube) GPS path, *without* the code attempting to control the drone.

---

## 1. Prerequisites
Ensure your hardware (Cube, Altimeter, U300 Radar) is connected and powered on.

---

## 2. Launch Sequence

You only need **one terminal** open on your Jetson companion computer.

### The Python Supervisor (Safe Mode)
This terminal runs the entire RIO stack, including the new ESKF fusion engine, hardware drivers, and data logger. We use `--no-mavlink` so it **cannot** send control commands back to the drone during this test.

**Command:**
```bash
cd "/home/alok/radar/rio_stack copy"
python3 src/supervisor.py --no-mavlink
```
**Expected Terminal Output:**
```text
[INFO] [Supervisor] [1/7] Loading configuration from config/system.yaml
[INFO] [Supervisor] [3/7] Initializing hardware drivers
[INFO] [RawIMUReader] RawIMUReader connected to /dev/ttyACM0 @ 115200
[INFO] [Supervisor] [7/7] MAVLink Output DISABLED (Safe Mode)
[INFO] [FlightLogger] Flight logger started: logs/run_20260922_143000.jsonl
```

---

## 3. The Flight Procedure

1.  **Verify Setup**: Ensure both terminals are running without crashing.
2.  **Arm**: Pick up your RC transmitter and arm the drone.
3.  **Takeoff**: Take off in **Stabilize, AltHold, or Loiter** (Standard manual modes). Do NOT attempt to engage an external-vision or offboard mode.
4.  **Fly Aggressively**: Fly forward, backward, left, right, and perform some yaw spins. The radar needs diverse motion to calculate velocity accurately.
5.  **Land**: Land the drone safely and disarm.
6.  **Stop**: Go to Terminal 2 (Python) and press `Ctrl+C` to stop the logger and save the file.

---

## 4. Understanding the Logs

Because you stopped the script cleanly, all data from the flight is now saved in a `.jsonl` file located in the `logs/` directory (e.g., `logs/run_20260922_143000.jsonl`).

### What is in the log?
Every single line in this file is a complete JSON object, perfectly timestamped. We record:
1.  **GPS**: The ground truth from the flight controller (Latitude, Longitude, Altitude, GPS Velocity).
2.  **NAV_STATE**: The 3D position, velocity, and orientation calculated by the RIO Radar.
3.  **HEALTH_REPORT**: The Ceres condition number and whether the algorithm considers the environment visually degraded.
4.  **RADAR_FRAME**: How many radar points were detected in that split second.

### Example Log Output (What it actually looks like):

```json
{"t": 12345.10, "type": "SESSION_START", "data": {"time_utc": 1695420000, "blueprint_version": "2026-09"}}

{"t": 12345.15, "type": "GPS", "data": {"lat": 28.5355, "lon": 77.3910, "alt_msl_m": 210.5, "vx_mps": 1.2, "vy_mps": 0.0, "vz_mps": -0.1, "hdg_deg": 45.0}}

{"t": 12345.16, "type": "RADAR_FRAME", "data": {"timestamp": 12345.158, "frame_id": 42, "point_count": 124, "valid": true}}

{"t": 12345.18, "type": "NAV_STATE", "data": {"timestamp": 12345.18, "frame_id": 42, "pose": {"position": [1.15, 0.02, -0.10], "quaternion": [0,0,0,1]}, "velocity": [1.25, -0.01, -0.05], "health": {"radar": true, "imu": true, "observable": true}, "mode": "RIO_GOOD"}}

{"t": 12346.10, "type": "HEALTH_REPORT", "data": {"quality_level": 3, "condition_number": 45.2, "reasons": []}}
```

### Next Steps for Analysis
Because both the `GPS` line and the `NAV_STATE` line exist side-by-side on the same timeline `t`, we can write a simple Python plotting script later to draw two lines on a graph: 
- Line 1: GPS Velocity (`vx_mps`, `vy_mps`)
- Line 2: Radar Velocity (`velocity[0]`, `velocity[1]`)

If the two lines perfectly overlap, the math is proven, the offsets are correct, and the drone is ready for full autonomy!
