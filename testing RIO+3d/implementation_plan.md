# Radar Navigation System — Master Implementation Plan

## Current Status
- **Hardware:** Linpowave U300 4D FMCW radar, connected via USB, hand-held.
- **Software:** `radar_fanout.py` → `doppler_rio.py` (RANSAC + WLS velocity) → `slam_node.py` (Frame-to-Map GICP with RIO prior).
- **Bugs Fixed:** Coordinate frame transform, absolute vs. relative GICP pose, planar degeneracy guard, planar fallback variable bug.
- **Not Yet Done:** Walk test validation, IMU integration, drone mounting.
- **End Goal:** ≤ 0.5% relative trajectory error on the UAV.

---

## Stage 1: Hand-Held Walk Test Validation (No IMU, No Drone)

**Purpose:** Prove that the coordinate frame fix in `slam_node.py` actually works before touching anything else. This is our baseline.

### Stage 1A: Straight-Line Walk (20 meters)
- Walk in a perfectly straight line for 20 meters on flat ground.
- Use a GPS app or measuring tape for ground truth.
- **Pass Criteria:**
  - SLAM distance: 20m ± 2m (within 10%)
  - SLAM displacement: 20m ± 2m (within 10%)
  - RIO distance: 20m ± 0.5m (within 2.5%)
  - Map Z-axis span: < 2 meters (no vertical explosion)

### Stage 1B: Return-to-Origin Walk (20m out, 20m back)
- Walk 20 meters straight, turn around, walk 20 meters back to the starting point.
- **Pass Criteria:**
  - Total distance: ~40m ± 4m
  - Final displacement: < 3m (you returned to the origin, so displacement should be near zero)
  - This tests whether GICP can re-lock to old map points (loop closure behavior)

### Stage 1C: Random Walk with Turns (GPS verified)
- Walk freely with turns, curves, and direction changes for ~50 meters of total distance.
- Use a GPS app to record the actual path and total distance.
- Displacement will be whatever the GPS says.
- **Pass Criteria:**
  - SLAM distance within 15% of GPS distance
  - SLAM displacement within 20% of GPS displacement
  - No NaN or crash in the logs
  - Map looks reasonable (no vertical spike, no spiral)

### Stage 1 Command
```bash
cd "/home/alok/radar/testing RIO+3d" && python3 supervisor.py \
    --port /dev/ttyUSB0 --tilt-deg 15.0 --no-mavlink \
    --voxel-size 0.10 --max-corr-dist 1.0 --min-correspondences 6 \
    --persistence-radius 0.50 --persistence-min-hits 1 \
    --window-s 0.4 --deadband 0.05 --eps 0.10 \
    --save-pcd /home/alok/radar/maps/ --visualizer-no-gui
```

### Stage 1 Decision Gate
- If Stage 1A and 1B pass → proceed to Stage 2.
- If Stage 1A fails with large Z-drift → the GICP absolute pose fix has a secondary bug; debug before proceeding.
- If Stage 1A fails with distance wildly wrong but displacement correct → RIO velocity has a scale error; investigate `doppler_rio.py` weighting.
- If Stage 1B displacement is large (>5m) → GICP is not re-locking to old map points; investigate `max_corr_dist` and map density.

---

## Stage 2: Software Hardening (Still Hand-Held, No IMU)

**Purpose:** Fix all remaining software issues found during Stage 1 before adding hardware complexity.

### Stage 2A: Add Diagnostic Logging
- Log per-keyframe: timestamp, `off_plane_ratio`, GICP fitness, GICP RMSE, number of correspondences, whether GICP or RIO-fallback was used, RIO velocity `[vx, vy, vz]`, pose `[x, y, z, roll, pitch, yaw]`.
- Save this to a CSV file alongside the PCD map.
- **Why:** Without per-frame diagnostics, we cannot debug drift. We are currently blind to what happens between the start and end of a walk.

### Stage 2B: Fix USB Port Stability
- During previous tests, the USB port disconnected during movement (the device disappeared from `/dev/ttyUSB*`).
- **Action:** Use a powered USB hub, secure the cable with tape or a clip, and verify the connection survives 5 minutes of vigorous walking.
- **Pass Criteria:** Zero port disconnections during a 5-minute walk.

### Stage 2C: Adaptive Voxel Size (For Future High-Altitude)
- Currently fixed at `--voxel-size 0.10`. This is correct for ground-level but will consume too much RAM at 50-100m altitude.
- **Action:** Implement range-adaptive voxel sizing: `voxel_size = clip(k * range * sigma_angle, 0.05, 2.0)`.
- **Why now:** This is a code change, not a hardware change. Better to do it before we mount on the drone.

### Stage 2 Decision Gate
- If diagnostics show GICP is consistently used (not just RIO fallback) → the 3D structure is rich enough for SLAM.
- If diagnostics show GICP is almost never used → the radar tilt angle or environment lacks vertical features; consider increasing tilt or testing near walls.

---

## Stage 3: IMU Integration (Hand-Held, IMU Added)

**Purpose:** Add the IMU to the system and connect it to `doppler_rio.py` to fix the lever-arm / rotational velocity bug.

### Stage 3A: IMU Selection & Purchase

> [!IMPORTANT]
> **Recommended IMU:** Any 6-axis or 9-axis IMU that connects via I2C or SPI to a Raspberry Pi / Jetson. Examples:
> - **ICM-42688-P** (best performance, ~$15, SPI preferred)
> - **BMI270** (good performance, ~$10, I2C/SPI)
> - **MPU-6050** (cheapest, ~$3, I2C, lower quality but sufficient for prototyping)
>
> For initial testing, an **MPU-6050 breakout board** is fine. For the final drone, upgrade to ICM-42688-P.

### Stage 3B: IMU Mounting Position

> [!IMPORTANT]
> **Where to mount the IMU:**
>
> **Option A (Best for hand-held testing): Bolt the IMU directly to the back of the radar PCB.**
> - The lever arm (`t_IR`) becomes nearly zero (< 1 cm).
> - This means the `omega × t_IR` correction term is negligible, and we can skip it entirely during hand-held testing.
> - The IMU and radar move as a single rigid unit — no flex, no vibration difference.
> - **This is what we should do for Stage 3.**
>
> **Option B (Required for drone integration — Stage 5): Mount the IMU at the drone's center of gravity.**
> - Standard practice in aerospace: the IMU measures the drone's true body motion.
> - The lever arm (`t_IR`) must then be carefully measured (e.g., 15 cm forward, 5 cm up).
> - The `omega × t_IR` correction becomes critical and must be coded.
> - **This is what we will do in Stage 5.**

**For Stage 3, use Option A.** Bolt the IMU to the radar.

### Stage 3C: IMU Software Driver
- Write a small Python module (`imu_reader.py`) that reads the gyroscope (`omega_x, omega_y, omega_z` in rad/s) and accelerometer (`a_x, a_y, a_z` in m/s²) at 200-400 Hz.
- Timestamp each reading with `time.monotonic()`.
- Broadcast IMU data over a local UDP port (e.g., `127.0.0.1:5558`).

### Stage 3D: Connect IMU to `doppler_rio.py`
- `doppler_rio.py` listens on the IMU UDP port.
- For each radar frame, it looks up the most recent gyroscope reading (`omega`).
- The Doppler velocity model changes from:
  ```
  v_d,i = -u_i^T * v_body          (CURRENT — WRONG)
  ```
  to:
  ```
  v_d,i = -u_i^T * (v_body + omega × p_i)   (CORRECT)
  ```
- This single equation change cancels out the rotational velocity of each radar point caused by the drone spinning.

### Stage 3E: Transmit Velocity Covariance
- `doppler_rio.py` currently sends `[timestamp, vx, vy, vz, n_inliers]` over UDP.
- **Upgrade:** Send `[timestamp, vx, vy, vz, cov_xx, cov_yy, cov_zz, cov_xy, cov_xz, cov_yz, n_inliers, observability_score]`.
- `slam_node.py` can then use the covariance to decide how much to trust RIO vs. GICP.

### Stage 3F: Walk Test with IMU
- Repeat Stage 1A, 1B, and 1C with the IMU attached.
- **Pass Criteria:**
  - RIO velocity error should decrease (especially during turns).
  - If you spin in place (pure rotation, no translation), RIO should report ~0 m/s instead of the spurious velocity it would have reported without the gyro correction.

### Stage 3 Decision Gate
- If spin-in-place test shows < 0.05 m/s false velocity → gyro correction is working.
- If spin-in-place test still shows > 0.15 m/s → check IMU axis alignment / sign convention.
- If walk tests improve over Stage 1 → proceed to Stage 4.

---

## Stage 4: Advanced Filtering (Hand-Held, IMU Connected)

**Purpose:** Replace the brittle hard-threshold filters with continuous probabilistic math.

### Stage 4A: Replace RANSAC with Adaptive Weighting (in `doppler_rio.py`)
- Keep RANSAC as the **initializer** to get a rough velocity estimate.
- After RANSAC, run an **Iteratively Reweighted Least Squares (IRLS)** refinement pass using Huber weights:
  ```
  w_i = 1 / (1 + (e_i / c)^2)
  ```
  where `e_i` is the Doppler residual and `c` is the radar's Doppler resolution (~0.03 m/s).
- This replaces the hard binary inlier/outlier decision with a smooth weighting curve.

### Stage 4B: Replace `off_plane_ratio` Heuristic with Hessian Eigenvalue Check (in `slam_node.py`)
- After running GICP, compute the Hessian `H = J^T W J` from the correspondence set.
- Perform eigendecomposition: if any eigenvalue is below a threshold, that degree of freedom is unobservable.
- Instead of completely disabling GICP on flat ground, **project the GICP update only onto the observable subspace** (X, Y, Yaw) and use the RIO/IMU prediction for the unobservable axes (Z, Roll, Pitch).
- This is the `compute_degenerate_update()` function from the other agent's suggestion.

### Stage 4C: Polar Uncertainty Model
- Replace the current `1/R²` weighting in `doppler_rio.py` with proper polar-to-Cartesian covariance:
  ```
  Σ_cart = J_s * diag(σ_r², σ_θ², σ_φ²) * J_s^T
  ```
- This makes far-away points automatically have less weight (because their angular uncertainty is large), while close points have high weight.

### Stage 4D: Validation Walk Tests
- Repeat Stage 1A, 1B, 1C.
- **Pass Criteria:**
  - SLAM distance error < 5% for straight walks.
  - SLAM distance error < 10% for random walks with turns.
  - Z-axis drift < 0.5m over a 50m walk.

### Stage 4 Decision Gate
- If all walk tests pass → the hand-held system is validated. Proceed to drone integration.
- If Z-drift persists → the Hessian eigenvalue check needs tuning; investigate the eigenvalue threshold.

---

## Stage 5: Drone Integration (Hardware)

**Purpose:** Mount the radar and IMU on the drone and validate in flight.

### Stage 5A: Physical Mounting

> [!IMPORTANT]
> **Radar Mounting:**
> - Mount the radar on the **front of the drone**, pointing forward.
> - Tilt it **10-15 degrees downward** so it can see the ground for velocity estimation while still seeing obstacles ahead.
> - Ensure the mount is **rigid** (no vibration, no flex). Use metal or 3D-printed hard mounts, not rubber.
>
> **IMU Mounting (Option B from Stage 3):**
> - Move the IMU from the radar's back to the **drone's center of gravity** (typically the center of the frame, near the flight controller).
> - Mount it on a **vibration-dampened platform** (double-sided foam tape or silicone standoffs) to isolate it from motor vibration.
> - Align the IMU axes with the drone's body axes: X = forward, Y = left, Z = up.
>
> **Lever Arm Measurement:**
> - Carefully measure the 3D vector from the IMU to the radar's phase center (the front face of the U300 PCB).
> - Example: if the radar is 15 cm forward, 3 cm right, and 5 cm above the IMU, then `t_IR = [0.15, -0.03, 0.05]` meters.
> - This vector is entered into the code as a configuration parameter.

### Stage 5B: Activate the Lever-Arm Correction
- In `doppler_rio.py`, update the Doppler model to use the measured lever arm:
  ```
  v_radar = R_RI * (v_body + omega × t_IR)
  v_d,i = -u_i^T * v_radar
  ```
- This was unnecessary in Stage 3 (IMU on the radar = zero lever arm) but is **critical** now.

### Stage 5C: Motor Vibration Filtering
- Drone motors create high-frequency vibration (typically 100-400 Hz) that contaminates the IMU.
- **Action:** Apply a low-pass filter (e.g., 2nd-order Butterworth at 30 Hz) to the IMU accelerometer data before using it.
- The gyroscope is less affected by vibration but should also be filtered at ~50 Hz.
- **Why:** Without this filter, the accelerometer will think the drone is constantly shaking violently, which corrupts the gravity estimate and any altitude hold.

### Stage 5D: Ground Test (Motors On, Drone Stationary)
- Power up the drone with motors spinning (at idle throttle, held down or tethered).
- Run the full stack for 2 minutes.
- **Pass Criteria:**
  - RIO velocity: < 0.05 m/s (drone is not moving)
  - SLAM displacement: < 0.1m over 2 minutes
  - IMU data is clean (no wild spikes from motor vibration after filtering)
  - No USB disconnections from motor EMI

### Stage 5E: Tethered Hover Test
- Tether the drone (rope to the ground) and hover at 1-2 meters for 30 seconds.
- **Pass Criteria:**
  - SLAM altitude: 1-2m ± 0.5m
  - SLAM X/Y drift: < 0.5m over 30 seconds
  - RIO velocity during hover: < 0.1 m/s (drone is hovering, not translating)

### Stage 5F: Free Flight Test (Short Range)
- Fly the drone in a straight line for 20 meters at 2-3 meters altitude, then land.
- **Pass Criteria:**
  - SLAM distance: 20m ± 2m
  - SLAM altitude: stable at 2-3m ± 0.5m
  - No crash, no divergence

### Stage 5 Decision Gate
- If all flight tests pass → proceed to Stage 6 (Shadow Mode).
- If motor vibration corrupts IMU → increase filtering aggressiveness or add better vibration dampening.
- If USB disconnects during flight → switch to a shielded USB cable or add ferrite beads.

---

## Stage 6: Shadow Mode — Passive Flight Validation (Radar Observes Only, No Commands)

**Purpose:** The radar system runs on the Jetson in parallel during real flights, logging its own position/velocity/map estimates, but it is **NOT connected to the Pixhawk** and sends **ZERO commands** to the flight controller. The drone is flown manually or via GPS waypoints as usual. We compare the radar's estimate against GPS after each flight to measure accuracy before trusting it with any control authority.

> [!CAUTION]
> **The radar must NOT be connected to the Pixhawk's serial/MAVLink port during this stage.** Use the `--no-mavlink` flag. The radar is a passive observer only. The Pixhawk uses its own GPS + barometer for flight as it normally does.

### Stage 6A: Setup
- Mount the Jetson + radar + IMU on the drone (as configured in Stage 5).
- Connect the radar to the Jetson via USB.
- The Jetson runs the full radar stack (`supervisor.py` with `--no-mavlink`).
- Separately, a GPS logger app runs on a phone or the Pixhawk logs its own GPS trajectory to an SD card.
- **The two systems are completely independent.** Radar does its thing; GPS does its thing. We compare afterwards.

### Stage 6B: Shadow Flight — Straight Line (100m)
- Fly the drone in a straight line for ~100 meters at a constant altitude (e.g., 10m), then land.
- Fly manually or use a GPS waypoint mission.
- After landing, download:
  1. The radar's SLAM trajectory (from the CSV log added in Stage 2).
  2. The GPS trajectory (from the Pixhawk log or phone GPS app).
- **Comparison Criteria:**
  - Radar total distance vs. GPS total distance: within 5%
  - Radar displacement vs. GPS displacement: within 5%
  - Radar altitude estimate vs. GPS/barometer altitude: within 2m
  - No NaN, no divergence, no map explosion in the radar log

### Stage 6C: Shadow Flight — Square Pattern (4 × 50m)
- Fly a square pattern (50m per side) at constant altitude using GPS waypoints.
- Total distance: ~200m. Final displacement: ~0m (back to start).
- **Comparison Criteria:**
  - Radar total distance vs. GPS: within 5%
  - Radar final displacement: < 5m (it should know it returned to the start)
  - Radar trajectory shape should visually resemble a square when plotted
  - Altitude should remain stable (no vertical drift > 2m)

### Stage 6D: Shadow Flight — Long Mission (500m+)
- Fly a longer mission (~500m total distance) with multiple waypoints, turns, and altitude changes.
- This stresses the map management, voxel memory, and long-term drift.
- **Comparison Criteria:**
  - Radar total distance vs. GPS: within 8%
  - Radar trajectory shape roughly matches GPS trajectory when overlaid
  - Jetson does not run out of RAM (map stays within `max_map_points`)
  - No USB disconnections over the full mission duration

### Stage 6E: Shadow Flight — GPS-Denied Simulation
- Fly a known GPS waypoint mission, but **after** landing, check: if you had turned off GPS at the start, would the radar's estimate alone have been good enough?
- This is just analysis — the drone still uses GPS for actual flight safety.
- **Comparison Criteria:**
  - If radar distance error is < 2% and trajectory shape is correct → the radar is ready to provide position feedback to the Pixhawk.
  - If radar distance error is > 5% or trajectory shape is wrong → go back and debug before proceeding.

### Stage 6 Decision Gate
- If **all** shadow flights show radar distance within 5% of GPS and trajectory shapes match → the radar is validated for closed-loop control. Proceed to Stage 7.
- If radar consistently under-reports or over-reports distance → investigate RIO velocity bias or GICP drift.
- If radar altitude drifts wildly → the Hessian degeneracy guard (Stage 4B) or ground-plane constraint needs tuning.
- If radar crashes or runs out of RAM → fix map management (voxel size, max_map_points) before proceeding.

> [!WARNING]
> **Do NOT proceed to Stage 7 (connecting radar to Pixhawk) until Stage 6 shadow flights consistently pass.** A radar with 10% error sending position commands to the Pixhawk will cause the drone to fly erratically or crash.

---

## Stage 7: Sensor Fusion & Tight Coupling

**Purpose:** Connect the radar output to a proper EKF for ≤ 0.5% drift. This is the first time the radar is allowed to influence the flight controller.

### Stage 7A: Integrate with PX4 EKF2 or `robot_localization`
- Feed our clean, covariance-tagged RIO velocity and SLAM pose into a standard EKF.
- The EKF fuses: IMU (400Hz) + RIO velocity (20Hz) + SLAM pose (5Hz).
- This gives you tight coupling without writing a custom 24-state filter from scratch.

### Stage 7B: IMU-Driven SE(3) Deskewing
- Replace the simple `xyz -= v_body * dt` deskew in `slam_node.py` with proper IMU-driven motion compensation.
- For each radar point acquired at time `t`, use the IMU trajectory to transform it to the keyframe reference time.
- This eliminates intra-scan motion distortion during fast maneuvers.

### Stage 7C: Closed-Loop Flight Test (Low Speed, Low Altitude)
- Connect the radar's position output to the Pixhawk via MAVLink.
- Fly in GPS-denied mode (GPS disabled or jammed) at low speed (< 2 m/s) and low altitude (2-3m) over a short distance (20m).
- A safety pilot must be ready to take over manual control at any moment.
- **Pass Criteria:**
  - Drone maintains stable hover and straight-line flight using only radar position
  - No oscillation, no altitude runaway, no fly-away

### Stage 7D: Validation with RTK-GPS Ground Truth
- Fly a known trajectory (e.g., a 100m square) with an RTK-GPS module logging ground truth.
- Compare the radar SLAM trajectory against the RTK-GPS trajectory.
- **Target:** ≤ 0.5% relative trajectory error.

---

## Stage 8: Long-Term & Loop Closure (Future)

**Purpose:** Achieve global consistency over long missions.

### Stage 8A: Submap-Based Map Management
- Replace the single growing `map_cloud` with a sliding window of keyframe submaps.
- Old submaps are frozen and stored; only the recent N keyframes are used for GICP matching.
- This prevents map self-contamination (where biased scans corrupt the map over time).

### Stage 8B: Loop Closure
- When the drone returns to a previously visited location, detect the overlap and correct accumulated drift.
- Use radar submap descriptors for place recognition.

### Stage 8C: Global Factor Graph (iSAM2)
- Add a global pose graph using GTSAM/iSAM2.
- Loop closures and landmark constraints are added as factors.
- The graph optimizes the entire trajectory offline or incrementally.

---

## Summary Table

| Stage | Hardware State | Key Action | Pass Criteria |
|-------|---------------|------------|---------------|
| **1** | Radar only, hand-held | Walk test with GICP fix | Distance ±10%, Z-drift < 2m |
| **2** | Radar only, hand-held | Diagnostics + USB fix | Zero crashes, CSV logs working |
| **3** | Radar + IMU (on radar), hand-held | Gyro correction in Doppler | Spin-in-place < 0.05 m/s |
| **4** | Radar + IMU (on radar), hand-held | Adaptive weighting + Hessian | Distance ±5%, Z-drift < 0.5m |
| **5** | Radar + IMU (on drone), flying | Drone integration + lever arm | Hover drift < 0.5m/30s |
| **6** | Radar on drone, **NO commands** | Shadow mode: radar observes, GPS flies | Radar vs GPS < 5% error |
| **7** | Full system, radar commands drone | EKF fusion + closed-loop flight | ≤ 0.5% with RTK ground truth |
| **8** | Full system, long missions | Loop closure + global graph | Global consistency |

---

> [!NOTE]
> **We are currently at the start of Stage 1.** The next physical action is to run the walk test (Stage 1A) with the coordinate-frame-fixed code and verify the results before touching anything else.

