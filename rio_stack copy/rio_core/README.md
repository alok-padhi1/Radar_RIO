# RIO Core — HKUST Radar-Inertial Odometry

This directory is the integration point for the **HKUST-Aerial-Robotics/RIO** C++ estimator.

## Architecture (Blueprint §4)

The HKUST RIO is a C++/Ceres-based optimization estimator. On the Jetson it
should run as a **dedicated native C++ process**, not a Python reimplementation.

```
U300 adapter (Python)
      ↓ IPC (socket / shared memory / ROS topic)
RIO C++ process
      ↓ IPC
State message → mapping / health / MAVLink (Python)
```

## Integration Steps (Blueprint §4, Phase 4-5)

1. Clone upstream: https://github.com/HKUST-Aerial-Robotics/RIO
2. Build isolated with Ceres, Eigen, PCL on Jetson
3. Validate with official sample dataset
4. Connect U300 adapter
5. Document build in `docs/BUILD_ENVIRONMENT.md`

## What This Directory Will Contain

- `CMakeLists.txt` or build scripts
- Any C++ adapter/wrapper code
- RIO configuration files
- IPC bridge (e.g., Unix socket server)

## Current Status

**PLACEHOLDER** — awaiting HKUST RIO C++ repository integration.
The Python-side adapter interfaces are defined in `estimator/rio_interface.py`.
