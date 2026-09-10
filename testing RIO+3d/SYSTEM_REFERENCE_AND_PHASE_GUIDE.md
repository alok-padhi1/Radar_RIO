# 4D mmWave Radar Dual-Thread RIO & SLAM Navigation System
## Complete System Reference, Architecture, Visualization & Multi-Stage Field Guide

---

## 1. System Overview & Architecture

This software stack provides **GPS-denied 3D navigation and obstacle avoidance** for unmanned aerial vehicles (multirotors and fixed-wing drones) using a forward-facing, pitch-tilted **Linpowave U300 4D FMCW mmWave Radar**.

### 1.1 Dual-Thread Architecture
Raw 4D FMCW radar data contains sparse target returns $(x, y, z, v_{radial})$. Direct registration of raw point clouds fails due to multipath ghosts, near-field leakage, and low point density. To solve this, the pipeline splits processing into two decoupled asynchronous threads:

```
                  ┌─────────────────────────────────────────────────────────┐
                  │                 Linpowave U300 4D Radar                 │
                  │              (UART @ 921600 Baud, 20 Hz)                │
                  └────────────────────────────┬────────────────────────────┘
                                               │
                                               ▼
                                 ┌───────────────────────────┐
                                 │      radar_fanout.py      │
                                 │   (Dedicated UART Owner)  │
                                 └───────┬───────────┬───────┘
                                         │           │
                     [Port 5005: 20 Hz]  │           │  [Port 5010: ~5 Hz Decimated]
                                         ▼           ▼
   ┌──────────────────────────────────────────┐    ┌──────────────────────────────────────────┐
   │              THREAD 1: RIO               │    │              THREAD 2: SLAM              │
   │             (doppler_rio.py)             │    │         (slam_node.py + filters.py)      │
   │                                          │    │                                          │
   │  • Tilt Transform (R_tilt @ P_R)         │    │  • Keyframe Accumulator (~300ms window)  │
   │  • 3D Doppler-RANSAC Outlier Rejection   │    │  • RIO Motion Deskewing                  │
   │  • Weighted Least-Squares (1/R²) Refit   │───▶│  • 6-Stage Rejection Filter Pipeline     │
   │  • Body-Frame 3D Velocity [vx,vy,vz]     │    │  • Ground-Plane Extraction (RANSAC)      │
   │                                          │    │  • k-NN Adaptive Covariance Estimation   │
   │  Rate: 20 Hz (Low Latency < 15ms)        │    │  • Keyframe-to-Submap GICP Registration  │
   └───────────────┬──────────────────────────┘    │  • Forward Safety Cone Obstacle Range    │
                   │                               │                                          │
                   │ [Ports 5007, 5008]            │  Rate: ~5 Hz (Computationally Heavy)     │
                   │                               └────────────────────┬─────────────────────┘
                   │                                                    │ [Port 5011]
                   ▼                                                    ▼
   ┌───────────────────────────────┐               ┌──────────────────────────────────────────┐
   │       mavlink_bridge.py       │               │                nav_node.py               │
   │   (Autopilot EKF Velocity)    │               │    (Autonomous Mission State Machine)    │
   │                               │               │                                          │
   │  • VISION_SPEED_ESTIMATE      │               │  • Dead-Reckons Pos between SLAM Updates │
   │  • Directly feeds PX4/Ardu EKF│               │  • Rotates Body Velocity to ENU Nav Frame│
   │  • Prevents high-speed drift  │               │  • Reactive Braking on Forward Obstacle  │
   └───────────────┬───────────────┘               │  • Sends SET_POSITION_TARGET_LOCAL_NED   │
                   │                               └────────────────────┬─────────────────────┘
                   ▼                                                    ▼
            [MAVLink @ 14540]                                    [MAVLink @ 14540]
   ┌──────────────────────────────────────────────────────────────────────────────────────────┐
   │                                PX4 / ArduPilot Autopilot                                 │
   │                      (GPS-Denied Position Hold, Mission Execution)                       │
   └──────────────────────────────────────────────────────────────────────────────────────────┘
```

---

### 1.2 UDP Loopback Network & Packet Specifications

To prevent process blocking and ensure zero coupling between nodes, inter-process communication occurs entirely over UDP localhost (`127.0.0.1`):

| Port | Source Node | Destination Node | Rate | Payload Description | Struct Format |
|---|---|---|---|---|---|
| **5005** | `radar_fanout.py` | `doppler_rio.py` | 20 Hz | Raw radar point cloud $(x, y, z, v)$ | `uint32 n` + $n \times$ `4×float32` |
| **5006** | `doppler_rio.py` | `slam_node.py` | 20 Hz | Body velocity for point deskewing & Doppler prior | `<dfffI` $(t, v_x, v_y, v_z, n_{inliers})$ |
| **5007** | `doppler_rio.py` | `mavlink_bridge.py` | 20 Hz | Body velocity for autopilot EKF aiding | `<dfffI` $(t, v_x, v_y, v_z, n_{inliers})$ |
| **5008** | `doppler_rio.py` | `nav_node.py` | 20 Hz | Body velocity for high-rate dead-reckoning | `<dfffI` $(t, v_x, v_y, v_z, n_{inliers})$ |
| **5010** | `radar_fanout.py` | `slam_node.py` | ~5 Hz | Decimated radar point cloud $(x, y, z, v)$ | `uint32 n` + $n \times$ `4×float32` |
| **5011** | `slam_node.py` | `nav_node.py` | ~5 Hz | 3D Pose $(4\times 4\ T_{world})$ + map point count + obstacle range | `<dId` $(t, n_{pts}, fwd\_range)$ + $16\times$`float64` |
| **14540**| `mavlink_bridge.py` / `nav_node.py` | Autopilot (PX4/ArduPilot) | 20–50 Hz | MAVLink telemetry, speed estimates, position setpoints | Standard MAVLink v2 |

---

## 2. Complete Code Inventory & Breakdown

Every Python file in `/home/alok/radar/testing RIO+3d/` has a distinct, non-overlapping responsibility:

```
testing RIO+3d/
├── radar_fanout.py    # Exclusive serial owner; demuxes UART stream to RIO and SLAM UDP ports
├── doppler_rio.py     # 20 Hz Doppler-RANSAC 3D ego-velocity estimator
├── filters.py         # 6-stage point rejection & cleaning pipeline (used by SLAM)
├── slam_node.py       # 5 Hz keyframe GICP SLAM + forward obstacle cone detector
├── mavlink_bridge.py  # Bridges RIO velocity into autopilot EKF (VISION_SPEED_ESTIMATE)
├── nav_node.py        # Waypoint state machine, attitude rotation & velocity setpoint generator
├── supervisor.py      # Stack process manager (launches, tags logs, monitors, and stops)
└── sitl_test.py       # Automated plumbing, port conflict, and unit test suite
```

---

### 2.1 `radar_fanout.py` (UART Serial Owner & Fan-Out Demux)
* **Purpose**: Sole owner of `/dev/ttyUSB0` (or specified serial port). It reads binary TLV frames from the radar hardware, unpacks the $(x, y, z, v_{radial})$ float array, and broadcasts UDP packets.
* **Why It Is Needed**: Serial ports cannot be opened by multiple processes simultaneously. `radar_fanout.py` parses once and splits the stream into a 20 Hz queue for RIO and a 5 Hz decimated queue for SLAM without blocking either consumer.
* **Key CLI Options**:
  * `--port /dev/ttyUSB0`: Serial device path.
  * `--baud 921600`: UART baud rate.
  * `--rio-port 5005`: Destination UDP port for full 20 Hz stream.
  * `--slam-port 5010`: Destination UDP port for decimated 5 Hz stream.
  * `--slam-decimation 4`: Enqueues every 4th frame for SLAM (20 Hz / 4 = 5 Hz).
* **Terminal Output**:
  ```text
  [fanout] [fanout] parsed=100 (20.0 Hz)  dropped_rio=0  dropped_slam=0
  ```

---

### 2.2 `doppler_rio.py` (Doppler-RANSAC Radar-Inertial Odometry)
* **Purpose**: Calculates true 3D vehicle ego-velocity $\mathbf{v}_{body} = [v_x, v_y, v_z]^T$ at 20 Hz from the radial Doppler measurements of stationary targets.
* **Mathematical Algorithm**:
  1. **Tilt Rotation**: Transforms radar points to body frame:
     $$\mathbf{P}_B = \mathbf{R}_{tilt} \mathbf{P}_R + \mathbf{t}_{lever}$$
     $$\mathbf{R}_{tilt} = \begin{bmatrix} \cos\theta & 0 & -\sin\theta \\ 0 & 1 & 0 \\ \sin\theta & 0 & \cos\theta \end{bmatrix}$$
  2. **Unit Line-of-Sight (LOS) Vectors**: $\mathbf{u}_i = \frac{\mathbf{P}_{B,i}}{\|\mathbf{P}_{B,i}\|}$.
  3. **Static Doppler Model**: For static ground points, measured radial velocity equals negative LOS projected onto body velocity:
     $$V_{radial, i} = -\mathbf{u}_i^T \mathbf{v}_{body}$$
  4. **Doppler-RANSAC (3-Point Solver)**: Randomly samples 3 non-coplanar points, solves for candidate velocity $\mathbf{v}_k = -\mathbf{A}_{3\times 3}^{-1} \mathbf{b}$, evaluates residual $|V_i + \mathbf{u}_i^T \mathbf{v}_k| < \epsilon$ across all points. Rejects dynamic movers and multipath clutter.
  5. **Weighted Least-Squares Refit**: Refits $\mathbf{v}_{body}$ on inliers using inverse-square distance weights $w_i = \frac{1}{R_i^2}$:
     $$\mathbf{v}_{body} = (\mathbf{A}^T \mathbf{W} \mathbf{A})^{-1} \mathbf{A}^T \mathbf{W} \mathbf{b}$$
* **Why It Is Needed**: Provides instantaneous velocity with zero integration drift, allowing the drone to hold position and estimate distance traveled even when SLAM keyframes drop or are rejected.
* **Key CLI Options**:
  * `--listen-port 5005`: Port where raw points arrive.
  * `--forward-ports "5006,5007,5008"`: Fan-out RIO velocity to SLAM, MAVLink bridge, and Navigation node.
  * `--theta-tilt-deg 40.0`: Sensor pitch-down angle in degrees.
  * `--eps 0.15`: RANSAC velocity inlier threshold in m/s.
  * `--selftest`: Runs synthetic mathematical validation.
* **Terminal Output**:
  ```text
  [rio] t=1725792001.234  v_body=[+0.852 -0.012 -0.045] m/s  inliers=38/44
  ```

---

### 2.3 `filters.py` (6-Stage Radar Point Cloud Cleaning Pipeline)
* **Purpose**: Pre-cleans raw radar detections before they reach the SLAM registration engine.
* **The 6 Rejection Stages**:
  1. **Near-Field Leakage Gate** ($r < 0.35\text{ m}$): Drops antenna coupling noise directly in front of the radar.
  2. **Valid Range Envelope Gate** ($0.35\text{ m} \le r \le 350\text{ m}$): Discards extreme range aliases and multi-bounce artifacts.
  3. **Doppler Static-Consistency Filter**: Drops points where $|V_i - (-\mathbf{u}_i^T \mathbf{v}_{body\_hint})| > 0.20\text{ m/s}$. Removes moving objects (cars, pedestrians, birds, foliage) and dynamic ghosts.
  4. **Temporal Persistence Tracker**: Requires a point to have a spatial neighbor within $1.0\text{ m}$ in at least 2 of the last 4 frames. Eliminates transient multipath returns that only flash for a single frame.
  5. **Statistical Outlier Removal (SOR)**: Drops isolated points with low neighbor density ($k=8$, $\text{std\_ratio}=1.5$).
  6. **Voxel Downsampling** ($\text{voxel\_size}=1.0\text{ m}$): Unifies density and bounds computational load.
* **Why It Is Needed**: Raw FMCW radar contains angular smearing and multipath ghosts. Feeding raw radar clouds into GICP causes catastrophic convergence failure.

---

### 2.4 `slam_node.py` (Keyframe Accumulator, GICP SLAM & Obstacle Detector)
* **Purpose**: Produces the absolute $SE(3)$ transformation matrix ($\mathbf{T}_{world} \in \mathbb{R}^{4\times 4}$) representing vehicle position $[X, Y, Z]$ and orientation, and calculates real-time obstacle distance.
* **Internal Architecture**:
  1. **Keyframe Accumulator & Deskewing**: Gathers 3 to 6 consecutive radar frames over a $0.3\text{ s}$ window. Translates points back using RIO velocity:
     $$\mathbf{P}_{deskewed} = \mathbf{P}(t) - \mathbf{v}_{body} \cdot (t_{ref} - t)$$
     This turns 30 sparse points into 200–500 dense, motion-corrected points.
  2. **Ground Plane Separation**: Uses RANSAC plane fitting to separate dominant ground returns from off-plane vertical structures (walls, trees, poles).
  3. **Adaptive Covariances**: Computes local $k$-NN geometric covariance ellipsoids per point.
  4. **Generalized ICP (GICP)**: Registers the keyframe against the global accumulated map cloud using the RIO constant-velocity prediction as the initial guess ($T_{init}$).
  5. **Forward Safety Cone Obstacle Detection**: Evaluates points inside a $\pm 20^\circ$ cone around the vehicle $+x$ axis. Returns `fwd_range` in meters (or $\infty$ / $-1.0$ if clear).
* **Key CLI Options**:
  * `--listen-port 5010`: Ingests decimated radar frames.
  * `--rio-port 5006`: Ingests RIO velocity for deskewing & $T_{init}$.
  * `--pose-port 5011`: Broadcasts pose packet to `nav_node.py`.
  * `--voxel-size 1.5`: SLAM map voxel downsampling in meters.
* **Terminal Output**:
  ```text
  [slam] keyframe OK  raw=184 -> final=112  map_pts=1850  corr=74  fitness=0.892  ground/off_plane=82/30  fwd_range=14.2m
  ```

---

### 2.5 `mavlink_bridge.py` (Autopilot EKF Velocity Aiding)
* **Purpose**: Transmits RIO body velocity into PX4 or ArduPilot using standard MAVLink messages (`VISION_SPEED_ESTIMATE` or `ODOMETRY`).
* **Why It Is Needed**: Autopilot flight controllers require continuous, low-latency velocity feedback to stop IMU accelerometer integration drift in GPS-denied environments.
* **Key CLI Options**:
  * `--autopilot px4` (or `ardupilot`).
  * `--mavlink-dest udp:127.0.0.1:14540`: Connects to SITL or companion computer MAVLink router.
  * `--rio-port 5007`: Ingests RIO velocity updates.
  * `--selftest`: Verifies MAVLink message serialization.
* **Terminal Output**:
  ```text
  [mavlink_bridge] pushing VISION_SPEED_ESTIMATE: vx=+0.85 vy=-0.01 vz=-0.05 m/s (inliers=38)
  ```

---

### 2.6 `nav_node.py` (Autonomous Mission State Machine & Nav Controller)
* **Purpose**: Coordinates high-level mission navigation, waypoints, obstacle braking, and position setpoints.
* **Key Features**:
  1. **Attitude Rotation**: Background `AttitudeListener` thread drains `ATTITUDE` messages from the autopilot and computes $\mathbf{R}_{nav\_body}(\text{roll}, \text{pitch}, \text{yaw})$.
  2. **Dual-Rate Dead-Reckoning**: Integrates high-rate (20 Hz) RIO velocity in the ENU navigation frame:
     $$\mathbf{P}_{nav}(t) = \mathbf{P}_{nav}(t-\Delta t) + \mathbf{R}_{nav\_body} \mathbf{v}_{body} \Delta t$$
     When a 5 Hz SLAM keyframe arrives, it smoothly blends/resets the position to the absolute SLAM coordinate.
  3. **Reactive Obstacle Braking**: If `fwd_range < 3.0m`, state machine overrides waypoint navigation and enters `OBSTACLE_STOP`, commanding zero velocity.
  4. **State Machine Modes**:
     * `INIT` $\rightarrow$ `WAIT_FOR_POSE` $\rightarrow$ `HOLD` $\rightarrow$ `MISSION` $\rightarrow$ `WAYPOINT_ARRIVED` $\rightarrow$ `RTL` (Return to Launch) $\rightarrow$ `OBSTACLE_STOP` $\rightarrow$ `FAILSAFE_RTL`.
* **Key CLI Options**:
  * `--platform px4` (or `ardupilot`).
  * `--waypoints "0,10,5;10,10,5;10,0,5;0,0,5"`: List of ENU $(X, Y, Z)$ targets in meters.
  * `--pose-port 5011`: Ingests SLAM pose and forward obstacle distance.
  * `--rio-port 5008`: Ingests 20 Hz RIO velocity for dead-reckoning.
  * `--max-speed 1.5`: Maximum cruising speed in m/s.
* **Terminal Output**:
  ```text
  [nav] state=MISSION target_wp=1 (10.0, 10.0, 5.0)  current_pos=(4.2, 5.1, 5.0)  dist_to_wp=6.3m  fwd_obs=14.2m
  ```

---

### 2.7 `supervisor.py` (Process Orchestrator & Watchdog)
* **Purpose**: Single launch command that brings up the entire pipeline in the exact required sequence with tagged logging, port management, and safe shutdown.
* **Why It Is Needed**: Manually running 5 terminal tabs with multiple UDP ports risks improper startup order, port conflicts, and orphaned processes.
* **Key CLI Options**:
  * `--port /dev/ttyUSB0`: Hardware serial port.
  * `--tilt-deg 40.0`: Mount tilt angle.
  * `--no-mavlink`: Disables MAVLink and navigation for Bench / Handheld testing (Stages 1–3).
  * `--enable-nav --waypoints "..."`: Launches the full autonomous flight navigation loop (Stage 4+).
* **Terminal Output**:
  ```text
  [supervisor] starting fanout: python3 radar_fanout.py --port /dev/ttyUSB0
  [supervisor] starting rio: python3 doppler_rio.py --listen-port 5005 --forward-ports 5006,5007
  [supervisor] starting slam: python3 slam_node.py --listen-port 5010 --rio-port 5006 --pose-port 5011
  ```

---

### 2.8 `sitl_test.py` (Automated System Validation Suite)
* **Purpose**: Automated test harness to verify math, socket communication, and prevent regressions.
* **Tests Performed**:
  * Test 1: RIO & MAVLink Bridge self-tests (synthetic point cloud Doppler-RANSAC validation).
  * Test 2: SO(3) Attitude rotation math verification.
  * Test 3: UDP multi-port fan-out conflict check (ensures no port collision on 5006, 5007, 5008).

---

## 3. How Distance, Velocity, and Obstacles are Calculated & Tracked

### 3.1 Velocity Measurement ($\mathbf{v}_{body}$)
* **Source**: `doppler_rio.py` at 20 Hz.
* **Calculation**: Doppler-RANSAC solves $-\mathbf{u}_i^T \mathbf{v}_{body} = V_{radial, i}$ on static radar points.
* **Where You See It**:
  * In the `[rio]` terminal log: `v_body=[+0.852 -0.012 -0.045] m/s`.
  * In QGroundControl: under MAVLink `VISION_SPEED_ESTIMATE`.

### 3.2 Distance Traveled & Position Tracking
Distance traveled is measured and cross-verified via **two independent mechanisms**:

```
                       ┌─────────────────────────────────────────────────────────┐
                       │                   DISTANCE COMPUTATION                  │
                       └────────────────────────────┬────────────────────────────┘
                                                    │
                      ┌─────────────────────────────┴─────────────────────────────┐
                      ▼                                                           ▼
       METHOD A: RIO Dead-Reckoning (20 Hz)                        METHOD B: SLAM Odometry (5 Hz)
     ──────────────────────────────────────                      ─────────────────────────────────
     • Integrates rotated body velocity:                         • Reads translation vector from T_world:
       P_nav(t) = ∫ (R_nav_body @ v_body) dt                       [X, Y, Z] = T_world[0:3, 3]
     • Distance traveled:                                        • Distance from start origin:
       Dist_RIO = ∑ ||v_body|| * Δt                                Dist_origin = √(X² + Y² + Z²)
     • High rate, smooth, zero lag                               • Total path length:
     • Small cumulative drift over time (1–3%)                     Dist_SLAM = ∑ ||T_{k} - T_{k-1}||
                                                                 • Drift-free absolute local map frame
```

* **Where to Inspect Distance**:
  * In `nav_node.py` logs: `current_pos=(X, Y, Z)` and `dist_to_wp=D`.
  * In log files: Summing `||v_body|| * dt` gives integrated distance traveled. During handheld walking (Stage 2), compare this value against your measured 50-meter walking track.

### 3.3 Forward Obstacle Distance (`fwd_range`)
* **Source**: `slam_node.py` calculated once per keyframe.
* **Calculation**: Finds the minimum Euclidean distance among all filtered points within a $\pm 20^\circ$ forward cone along the body $+x$ axis:
  $$\text{in\_cone} = (x > 0.35\text{ m}) \land \left(\frac{x}{\sqrt{x^2+y^2+z^2}} > \cos(20^\circ)\right)$$
  $$\text{fwd\_range} = \min_{\text{in\_cone}} \sqrt{x^2 + y^2 + z^2}$$
* **Where You See It**:
  * In `[slam]` logs: `fwd_range=14.2m` (or `inf` / `-1.0` if clear).
  * In `[nav]` logs: `fwd_obs=14.2m`. If this value drops below `3.0m`, the navigation node halts forward motion.

---

## 4. Where and How to Visualize Every Stage

### 4.1 Terminal Live Telemetry Visualization
Every process output is multiplexed and tagged by `supervisor.py`:

```text
[fanout] parsed=520 (20.0 Hz)  dropped_rio=0  dropped_slam=0
[rio]    t=1725792015.42  v_body=[+0.920 +0.010 -0.020] m/s  inliers=42/48
[slam]   keyframe OK  raw=195 -> final=120  map_pts=2410  corr=88  fitness=0.912  ground/off_plane=95/25  fwd_range=12.5m
[nav]    state=MISSION  wp=2/4  pos=[+12.4, +8.1, +5.0]  target=[+20.0, +10.0, +5.0]  dist=7.8m  fwd_obs=12.5m
```

### 4.2 3D Point Cloud & SLAM Map Real-Time Visualization
To visualize the 3D filtered point cloud, ground plane, and accumulated map in an interactive Open3D window:

1. Create a lightweight visualization script `visualizer_3d.py` in your testing folder:
```python
#!/usr/bin/env python3
import socket, struct, numpy as np, open3d as o3d

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 5010))  # Listens to decimated filtered radar stream
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="4D Radar Real-Time Point Cloud", width=1024, height=768)
pcd = o3d.geometry.PointCloud()
vis.add_geometry(pcd)

print("Visualizer listening on UDP port 5010...")
first = True
while True:
    data, _ = sock.recvfrom(65535)
    (n,) = struct.unpack_from('<I', data, 0)
    pts = np.frombuffer(data, dtype='<f4', count=n*4, offset=4).reshape(n, 4)
    pcd.points = o3d.utility.Vector3dVector(pts[:, :3])
    
    # Color points by Doppler velocity (Blue = approaching, Red = receding)
    v = pts[:, 3]
    colors = np.zeros((n, 3))
    colors[:, 0] = np.clip(v / 2.0, 0, 1)      # Red
    colors[:, 2] = np.clip(-v / 2.0, 0, 1)     # Blue
    colors[:, 1] = 1.0 - (colors[:, 0] + colors[:, 2]) # Green for zero velocity
    pcd.colors = o3d.utility.Vector3dVector(colors)
    
    vis.update_geometry(pcd)
    if first:
        vis.reset_view_point(True)
        first = False
    vis.poll_events()
    vis.update_renderer()
```

2. Run alongside `supervisor.py` to see real-time 3D point clusters colored by Doppler velocity.

### 4.3 Ground Control Station (QGroundControl) Visualization
When `mavlink_bridge.py` is running:
* **Attitude & Heading Indicator**: Reflects live pitch, roll, and yaw.
* **Local Position (NED/ENU)**: The vehicle icon moves across the local grid in real time without GPS.
* **Velocity Vectors**: Green velocity needle tracks RIO `v_body` output.
* **Waypoints**: QGC displays active target waypoints and flight path.

---

## 5. Step-by-Step Multi-Stage Field Guide & Exact Commands

Follow these stages sequentially. **Do not skip stages**; each stage isolates and validates a specific physical and algorithmic variable.

---

### Stage 0: Pre-Flight Safety & Plumbing Self-Tests

* **Goal**: Validate that mathematical libraries, SO(3) rotations, multi-socket fan-out, and serial dependencies function without error.
* **Components Running**: `sitl_test.py`, `doppler_rio.py --selftest`, `mavlink_bridge.py --selftest`.
* **Execution Command**:
  ```bash
  cd "/home/alok/radar/testing RIO+3d"
  python3 sitl_test.py --plumbing-only
  ```
* **Expected Output**:
  ```text
  [sitl_test] Running Test 1: Module self-tests...
  [self_test] PASS (velocity error = 0.0221 m/s < 0.15 m/s)
  [mavlink_bridge] self_test PASS
  [sitl_test] Running Test 2: SO(3) body-to-nav rotation math...
  [sitl_test] PASS: Rotation math validated.
  [sitl_test] Running Test 3: Multi-port UDP fan-out conflict check...
  [sitl_test] PASS: All 3 consumers received RIO velocity without conflict.
  [sitl_test] ALL PRE-FLIGHT CHECKS PASSED.
  ```
* **Pass Gate**: All 3 tests pass with 0 errors.

---

### Stage 1: Bench Static Testing (Radar + RIO + SLAM)

* **Goal**: Confirm the physical radar sensor, 6-stage filter pipeline, and GICP registration are stable on a stationary bench before motion is introduced.
* **Components Running**: `radar_fanout.py`, `doppler_rio.py`, `slam_node.py` (orchestrated by `supervisor.py`).
* **Execution Command**:
  ```bash
  cd "/home/alok/radar/testing RIO+3d"
  python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40.0 --no-mavlink
  ```
* **Expected Output**:
  ```text
  [fanout] parsed=100 (20.0 Hz)  dropped_rio=0  dropped_slam=0
  [rio]    t=1725792010.12  v_body=[+0.002 -0.001 +0.000] m/s  inliers=35/38
  [slam]   keyframe OK  raw=140 -> final=85  map_pts=420  corr=62  fitness=0.940  ground/off_plane=75/10  fwd_range=inf
  ```
* **Verification Checks**:
  1. `v_body` on stationary bench must be within $\pm 0.05\text{ m/s}$ of zero.
  2. `n_ground` should represent the large majority of `n_final` points.
  3. Let it run for **5+ minutes continuous**. Ensure zero exceptions or crashes.
* **Pass Gate**: 5 minutes of continuous run, steady point counts, GICP convergence fitness $> 0.80$.

---

### Stage 2: Handheld Walking Test (Odometry & Distance Validation)

* **Goal**: Validate that RIO velocity integration and SLAM pose chains accurately measure true distance traveled during walking motion.
* **Equipment**: Laptop/companion computer + radar mounted on chest rig or handheld pole (tilted $40^\circ$ downward).
* **Execution Command**:
  ```bash
  cd "/home/alok/radar/testing RIO+3d"
  python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40.0 --no-mavlink
  ```
* **Test Protocol**:
  1. Mark a **50.0-meter straight line** on the ground (measured with a tape measure or paced).
  2. Start at the 0m mark, start `supervisor.py`.
  3. Walk in a straight line at normal pace (~$1.0\text{ m/s}$) to the 50m mark and stop.
  4. Note the forward velocity $v_x$ during walking and verify total integrated distance.
  5. Walk a figure-eight pattern to verify velocity vector rotation during turns.
* **Expected Output**:
  ```text
  [rio]  t=1725792040.50  v_body=[+1.120 +0.040 -0.010] m/s  inliers=42/46
  [slam] keyframe OK  raw=210 -> final=145  map_pts=3200  corr=98  fitness=0.880  ground/off_plane=110/35  fwd_range=8.5m
  ```
* **Verification Checks**:
  1. Total integrated distance ($\int v_x dt$) must match $50\text{ m} \pm 5\%$ ($47.5\text{ m}$ to $52.5\text{ m}$).
  2. RIO inlier ratio $> 85\%$ of all frames while walking.
  3. SLAM keyframe convergence $> 80\%$.
* **Pass Gate**: Measured distance error $< 5\%$, no velocity discontinuities or sign inversions.

---

### Stage 3: Vehicle-Mounted Drive Test (High-Speed & Varied Terrain)

* **Goal**: Validate RIO and SLAM behavior at flight-realistic speeds ($5\text{ to }15\text{ m/s}$) and over long distances ($1\text{ to }3\text{ km}$).
* **Equipment**: Radar mounted to vehicle roof rack or rigid window mount, pointing forward-down at $40^\circ$.
* **Execution Command**:
  ```bash
  cd "/home/alok/radar/testing RIO+3d"
  python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40.0 --no-mavlink
  ```
* **Test Protocol**:
  1. Drive along a road at $20\text{ to }50\text{ km/h}$ ($5.5\text{ to }14\text{ m/s}$) for 30 minutes.
  2. Compare radar velocity against the car's speedometer / GPS reference.
  3. **Gap Behavior Check**: Drive across a smooth bridge or overpass where radar ground reflection drops.
* **Expected Output**:
  ```text
  [rio]  t=1725792100.20  v_body=[+10.420 -0.150 +0.080] m/s  inliers=55/62
  [slam] keyframe OK  raw=310 -> final=190  map_pts=15200  corr=130  fitness=0.865  ground/off_plane=140/50  fwd_range=32.0m
  ```
* **Verification Checks**:
  1. During radar illumination loss (e.g. over water/bridge), RIO must report `valid=False` and SLAM must reject the keyframe. It **must NOT produce smooth, false trajectories**.
* **Pass Gate**: 30+ minutes stable drive, speed tracks vehicle speedometer within $\pm 0.3\text{ m/s}$, clean rejection on surface dropouts.

---

### Stage 4: SITL Full Autonomous Mission Simulation

* **Goal**: Validate closed-loop waypoint navigation, MAVLink integration, arming, state machine transitions, and reactive obstacle stops in software-in-the-loop simulation with zero hardware risk.

#### Step 4.1: Launch Autopilot SITL
In Terminal 1, launch PX4 or ArduPilot SITL:
```bash
# Example for PX4 SITL
cd ~/PX4-Autopilot && make px4_sitl jmavsim
```

#### Step 4.2: Launch Full Navigation Stack
In Terminal 2, launch `supervisor.py` with navigation and square mission waypoints ($10\times 10\text{ m}$ square at $5\text{ m}$ altitude):
```bash
cd "/home/alok/radar/testing RIO+3d"
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40.0 \
    --platform px4 --mavlink-dest udp:127.0.0.1:14540 \
    --enable-nav --waypoints "0,10,5;10,10,5;10,0,5;0,0,5"
```

#### Step 4.3: Execute Mission
1. Watch terminal output until state reaches `[nav] state=HOLD`.
2. In the `nav_node` prompt, type:
   ```text
   go
   ```
3. The vehicle switches to `OFFBOARD` / `GUIDED`, arms, climbs to $5\text{ m}$, navigates through all 4 waypoints, and returns to start (`RTL`).

#### Step 4.4: Deliberate Obstacle Failsafe Injection Test
1. While in `state=MISSION`, inject an obstacle return by walking in front of the radar ($r < 3.0\text{ m}$).
2. Confirm state immediately switches:
   ```text
   [nav] Obstacle detected at 2.1m (< 3.0m safety threshold)! -> Entering OBSTACLE_STOP
   ```
3. Velocity setpoints immediately command $[0, 0, 0]\text{ m/s}$.

* **Pass Gate**: Clean square mission execution in SITL, smooth waypoint transitions, immediate braking on forward obstacle detection.

---

### Stage 5: Real Airframe Flight Progression

> [!CAUTION]
> Always maintain an independent safety pilot on manual RC override at all times during flight testing. Ensure the autopilot's own hardware geofence and battery failsafes are active.

```
                      ┌─────────────────────────────────────────────────────────┐
                      │             FLIGHT TEST PROGRESSION PHASES              │
                      └────────────────────────────┬────────────────────────────┘
                                                   │
          ┌─────────────────┬──────────────────────┴───────────┬─────────────────┐
          ▼                 ▼                                  ▼                 ▼
     Stage 5a          Stage 5b                           Stage 5c          Stage 5d
   Tethered Hover   Low Manual Flight                  Full Autonomous   Fixed-Wing Cruise
   (1-2m Altitude)  (5-10m Altitude)                   Waypoint Mission  (100-200m AGL)
   Verify EKF aiding Verify SLAM consistency           10x10m square     Long-range GPS-denied
```

* **Stage 5a (Tethered Low Hover, 1–2m)**:
  * Arm in manual mode (`POSCTL` / `LOITER`).
  * Verify autopilot EKF accepts `VISION_SPEED_ESTIMATE` without altitude or position runaway.
* **Stage 5b (Manual Flight, 5–10m)**:
  * Fly manual translation boxes.
  * Verify in QGC that SLAM pose and RIO velocity track the airframe motion cleanly.
* **Stage 5c (Full Autonomous GPS-Denied Mission)**:
  * Launch `supervisor.py` with `--enable-nav`.
  * Command `go` and supervise automated waypoint navigation in GPS-denied environment.
* **Stage 5d (High-Altitude Fixed-Wing / Long Range, 100–200m AGL)**:
  * Fixed-wing cruise flight over varied ground terrain. Radar pitch tilt ensures constant forward-ground illumination.

---

## 6. Troubleshooting & Common Failure Modes

| Symptom / Error | Root Cause | Immediate Fix |
|---|---|---|
| `serial.serialutil.SerialException: [Errno 13] Permission denied` | User not in `dialout` group | Run `sudo usermod -a -G dialout $USER` and log back in. |
| `[slam_node] keyframe REJECTED: too_sparse_after_filtering` | Filter thresholds too tight or radar pointed at empty sky | Verify radar tilt is $40^\circ$ downward pointing at ground structure. Check `--leakage-radius 0.35`. |
| `[rio] RIO frame rejected (n=2)` | Radar sees fewer than 3 returns | Normal when pointed at unobstructed open sky. Ensure sensor has line-of-sight to ground clutter. |
| `OSError: [Errno 98] Address already in use` | Previous node process did not terminate cleanly | Kill stale processes: `pkill -9 -f "doppler_rio\|slam_node\|radar_fanout\|nav_node"`. |
| `[nav_node] Timeout waiting for initial pose` | `slam_node.py` hasn't converged on first keyframe | Check that `radar_fanout.py` is receiving UART frames and `slam_node.py` is running on port 5010. |
| Autopilot EKF rejects vision velocity | Time synchronization offset or high latency | Set autopilot parameter `EKF2_EV_DELAY` to match measured system latency (~$30\text{ ms}$). |

---

## 7. Quick Reference Command Cheat Sheet

```bash
# 1. Run Automated Pre-Flight Self-Tests
python3 sitl_test.py --plumbing-only

# 2. Bench Testing (Stationary Sensor Check)
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40.0 --no-mavlink

# 3. Handheld Walking Test (50m Distance Check)
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40.0 --no-mavlink

# 4. Vehicle Drive Test (High-Speed Road Test)
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40.0 --no-mavlink

# 5. SITL Autonomous Waypoint Navigation
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40.0 \
    --platform px4 --mavlink-dest udp:127.0.0.1:14540 \
    --enable-nav --waypoints "0,10,5;10,10,5;10,0,5;0,0,5"

# 6. Kill Stale Background Processes (Emergency Reset)
pkill -9 -f "doppler_rio|slam_node|radar_fanout|nav_node|mavlink_bridge|supervisor"
```
