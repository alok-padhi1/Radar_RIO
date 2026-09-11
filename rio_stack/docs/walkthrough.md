# RIO Radar — Implementation Walkthrough

## Summary of Changes

### New Files Created

#### 1. [gps_logger.py](file:///home/aman/Work_Repository/ShunyAnantam/RIO-radar/Radar_RIO/testing%20RIO+3d/gps_logger.py)
GPS ground-truth + radar output simultaneous logger. Runs alongside the radar stack and records:
- **GPS fixes** (NEO-M8N NMEA via serial → converted to ENU meters)
- **RIO velocity** (from `doppler_rio.py` on UDP 5013)
- **SLAM pose** (from `slam_node.py` on UDP 5014)

All entries share a common `t_mono` timestamp for post-run time-alignment. Output is a timestamped JSONL file in `logs/`.

#### 2. [analyze_run.py](file:///home/aman/Work_Repository/ShunyAnantam/RIO-radar/Radar_RIO/testing%20RIO+3d/analyze_run.py)
Post-run analysis script. Reads the JSONL log and computes:
- **XY vs Z error decomposition** (answers the audit's #1 question: is the 17% Stage 2 error Z-dominant?)
- **RIO distance vs GPS path length** (odometry accuracy)
- **Velocity comparison** (RIO speed vs GPS-derived speed)
- **Drift rate** (linear fit of position error over time)
- **Pass/fail gates** aligned to the Field Runbook

---

### Modified Files

#### 3. [slam_node.py](file:///home/aman/Work_Repository/ShunyAnantam/RIO-radar/Radar_RIO/testing%20RIO+3d/slam_node.py) — 3 changes
- **Multi-port pose forwarding**: `--pose-port` now accepts comma-separated ports (e.g., `5011,5014`). Each consumer gets its own dedicated port — no `SO_REUSEPORT` conflicts.
- **Configurable plane threshold** (audit S-3): New `--plane-threshold` arg (default 0.6m). Increase for flight altitude.
- **GICP timing guard** (audit R-8): Logs a warning if GICP takes >150ms. Critical for RPi4 deployment.

#### 4. [supervisor.py](file:///home/aman/Work_Repository/ShunyAnantam/RIO-radar/Radar_RIO/testing%20RIO+3d/supervisor.py) — GPS integration
- New flags: `--gps-port`, `--gps-baud`, `--log-dir`
- When `--gps-port` is set:
  - Adds UDP 5013 to RIO forward ports (for `gps_logger.py`)
  - Adds UDP 5014 to SLAM pose ports (for `gps_logger.py`)
  - Launches `gps_logger.py` as a non-critical child process

---

## How To Use

### Handheld Test with GPS Ground Truth

```bash
# 1. Plug in radar FIRST (→ /dev/ttyUSB0), then GPS (→ /dev/ttyUSB1)

# 2. Run the stack with GPS logging
cd Radar_RIO/testing\ RIO+3d/
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 \
    --no-mavlink \
    --gps-port /dev/ttyUSB1

# 3. Walk your test course (e.g., 20m straight, 50m square)
# 4. Ctrl+C to stop. gps_logger prints the log file path.

# 5. Analyze the run
python3 analyze_run.py logs/run_YYYYMMDD_HHMMSS.jsonl
```

### What the Report Tells You

The report decomposes SLAM error into XY (horizontal) and Z (vertical). This directly answers the audit's key question about the Stage 2 17% error:

- **If Z-error dominates**: Confirms audit risk R-4 — gravity alignment / IMU integration is the priority
- **If XY-error dominates**: GICP registration quality is the limiter — focus on filter tuning and off-plane structure

### Port Allocation (Updated)

| Port | Consumer | Data |
|------|----------|------|
| 5005 | `doppler_rio.py` | Raw radar frames from `radar_fanout.py` |
| 5006 | `slam_node.py` | RIO velocity (deskew + Doppler prior) |
| 5007 | `mavlink_bridge.py` | RIO velocity (autopilot aiding) |
| 5008 | `nav_node.py` | RIO velocity (dead-reckoning) |
| 5009 | `visualizer_3d.py` | RIO velocity (display) |
| 5010 | `slam_node.py` | Decimated radar points from `radar_fanout.py` |
| 5011 | `nav_node.py` / `visualizer_3d.py` | SLAM pose |
| 5012 | `visualizer_3d.py` | Decimated radar points (display) |
| **5013** | **`gps_logger.py`** | **RIO velocity (logging)** |
| **5014** | **`gps_logger.py`** | **SLAM pose (logging)** |

## Testing

- All 4 files pass Python syntax validation
- `analyze_run.py` tested end-to-end with synthetic data — all metrics compute correctly, report prints cleanly, pass/fail gates evaluate correctly
- `slam_node.py` and `supervisor.py` retain backward compatibility — existing commands without `--gps-port` work identically
