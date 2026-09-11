# RIO Radar Stack — Command Cookbook

This document consolidates all the common run commands for the `rio_stack` workspace. Since the project has been restructured, always pay attention to the working directory for each command.

---

## 1. Running the Main Stack (`supervisor.py`)

The supervisor manages all the radar nodes (`radar_fanout`, `doppler_rio`, `slam_node`, etc.) automatically. 

**Run from:** `Radar_RIO/rio_stack/src/`

### Stage 1A: Hand-Held Walk Test (with GPS ground-truth)
Use this specific command for flat-ground walking tests. It lowers the voxel size for human-height scanning and uses `--tilt-deg 0.0` assuming you are holding the radar flat.
```bash
cd ~/radar/rio_stack/src/
python3 supervisor.py \
    --port /dev/ttyUSB0 --tilt-deg 0.0 --no-mavlink \
    --voxel-size 0.10 --max-corr-dist 1.0 \
    --eps 0.10 --save-pcd ../../maps/ --visualizer-no-gui \
    --gps-port /dev/ttyUSB1 --log-dir ../logs/
```
*(The JSONL logs are saved to `../logs/`, and the 3D map is saved to `../../maps/` when you press Ctrl+C)*

### Stage 3: IMU Rotation Compensation Testing
Use this for testing the Z-axis drift fix using the Cube Orange IMU. The IMU must be plugged in via USB before running this.
```bash
cd ~/radar/rio_stack/src/
python3 supervisor.py \
    --port /dev/ttyUSB0 --tilt-deg 0.0 --no-mavlink \
    --voxel-size 0.10 --max-corr-dist 1.0 \
    --eps 0.10 --save-pcd ../../maps/ --visualizer-no-gui \
    --gps-port /dev/ttyUSB1 --log-dir ../logs/ \
    --imu-port /dev/ttyACM0
```

### Standard Bench / Handheld Testing (with GPS logging)
Use this for general handheld tests with standard parameters.
```bash
cd ~/radar/rio_stack/src/
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 --no-mavlink --gps-port /dev/ttyUSB1
```
*(Logs are saved to `../logs/run_YYYYMMDD_HHMMSS.jsonl`)*

### Basic Bench Testing (No GPS, No Autopilot)
Use this to simply verify the radar is outputting data and SLAM is running.
```bash
cd Radar_RIO/rio_stack/src/
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 --no-mavlink
```

### Full Flight / SITL Testing (with MAVLink)
Use this when connected to an autopilot (PX4/ArduPilot).
```bash
cd Radar_RIO/rio_stack/src/
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 --platform px4 --mavlink-dest udp:127.0.0.1:14540
```

### Saving a Map (.pcd)
You can append `--save-pcd ../../maps/` to any supervisor command to save the accumulated 3D SLAM map when you press `Ctrl+C`.
```bash
cd Radar_RIO/rio_stack/src/
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 --no-mavlink --save-pcd ../../maps/
```

---

## 2. Post-Run Analysis (`tools/`)

Run these scripts to analyze the data collected during your tests.

**Run from:** `Radar_RIO/rio_stack/`

### GPS Drift & Performance Analysis
Analyzes the JSONL log from a handheld GPS test and outputs a pass/fail report on SLAM drift.
```bash
cd Radar_RIO/rio_stack/
python3 tools/analyze_run.py logs/run_YYYYMMDD_HHMMSS.jsonl
```

### Exporting Analysis Data for Plotting
You can export the raw time-aligned arrays to `.npz` format for custom Python plotting.
```bash
cd Radar_RIO/rio_stack/
python3 tools/analyze_run.py logs/run_YYYYMMDD_HHMMSS.jsonl --save-npz logs/results.npz
```

---

## 3. Automated Testing (`tests/`)

Run these to verify the stack's mathematical logic and port bindings aren't broken.

**Run from:** `Radar_RIO/rio_stack/tests/`

### Plumbing-Only Test (No Radar Required)
Verifies that all internal UDP ports can bind, math rotations are correct, and internal self-tests pass.
```bash
cd Radar_RIO/rio_stack/tests/
python3 sitl_test.py --plumbing-only
```

### Full SITL Integration Test
Requires PX4/ArduPilot SITL to be running, and the radar to be plugged in. Tests the actual supervisor startup and stabilization.
```bash
cd Radar_RIO/rio_stack/tests/
python3 sitl_test.py --port /dev/ttyUSB0 --platform px4
```

---

## 4. Standalone Node Self-Tests (`src/`)

Some nodes have built-in unit tests you can run directly.

**Run from:** `Radar_RIO/rio_stack/src/`

### RIO Math Self-Test
Validates the weighted least-squares and RANSAC matrix math.
```bash
cd Radar_RIO/rio_stack/src/
python3 doppler_rio.py --selftest
```

### MAVLink Bridge Self-Test
Validates the coordinate frame rotations from Body frame to NED frame.
```bash
cd Radar_RIO/rio_stack/src/
python3 mavlink_bridge.py --selftest
```
