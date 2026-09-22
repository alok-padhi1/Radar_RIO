# STAGED IMPLEMENTATION PLAN
# Single U300 + Cube IMU + U200A — GPS-Denied RIO + SLAM

**Last Updated:** 2026-09-22  
**Hardware:** 1× Linpowave U300 + 1× U200A + Cube Orange+ + Jetson Orin Nano  
**Repository:** `rio_stack copy/` on branch `alok/new/gpsdenied`  
**Baseline Tag:** `baseline_flu_working`

---

# HOW TO READ THIS DOCUMENT

Each **Stage** is a self-contained milestone. Stages are executed **sequentially** — you do not start Stage N+1 until Stage N passes ALL its gate criteria.

```text
┌────────────┐     ┌────────────┐     ┌────────────┐     ┌────────────┐
│  STAGE 1   │────▶│  STAGE 2   │────▶│  STAGE 3   │────▶│  STAGE 4   │
│ Foundation │     │  Validate  │     │   SLAM     │     │  Flight    │
│  (Software)│     │ (Physical) │     │ (Software) │     │  (Physical)│
└────────────┘     └────────────┘     └────────────┘     └────────────┘
   ✅ DONE           NEXT               BLOCKED           BLOCKED
```

---

# STAGE 1 — SOFTWARE FOUNDATION ✅ COMPLETE

## What Was Implemented

| Item | File | Status |
|---|---|---|
| Freeze FLU baseline | `git tag baseline_flu_working` | ✅ Done |
| FRD/NED frame convention | `eskf.py`, `estimator.py`, `raw_imu_reader.py`, `state.py` | ✅ Done |
| IMU: SCALED_IMU2, SI units, no FLU conversion | `drivers/cube/raw_imu_reader.py` | ✅ Done |
| ESKF: gravity `[0, 0, +9.81]` NED | `estimator/eskf_rio/eskf.py` | ✅ Done |
| ESKF: altimeter update with Joseph form | `estimator/eskf_rio/eskf.py` | ✅ Done |
| Radar extrinsics module (T_B_R, lever-arm) | `estimator/radar_extrinsics.py` | ✅ Done |
| Lever-arm formula: `v_body = R_B_R·v_radar − ω×t_B_R` | `estimator/radar_extrinsics.py` | ✅ Done |
| ZUPT independent of radar rejection | `estimator/eskf_rio/estimator.py` | ✅ Done |
| No `radar rejected → velocity = 0` | `estimator/eskf_rio/estimator.py` | ✅ Done |
| Extrinsic values marked PLACEHOLDER | `config/system.yaml` | ✅ Done |
| Frame annotations on all state vectors | `estimator/state.py` | ✅ Done |
| 16 Doppler unit tests | `tests/test_doppler_comprehensive.py` | ✅ 16/16 pass |
| 15 ESKF unit tests | `tests/test_eskf_comprehensive.py` | ✅ 15/15 pass |
| 8 extrinsic unit tests | `tests/test_extrinsics.py` | ✅ 8/8 pass |
| 4 IMU convention tests | `tests/test_imu_convention.py` | ✅ 4/4 pass |
| 7 core integration tests | `tests/test_core_logic.py` + root tests | ✅ 7/7 pass |

## Gate Criteria — ✅ ALL PASSED

```text
[✅] 50/50 unit tests pass
[✅] No FLU references in new estimator code
[✅] Gravity correctly cancels stationary FRD accel
[✅] Lever-arm compensated for pure yaw, pitch, roll
[✅] ZUPT only triggers from IMU variance, never from radar rejection
[✅] Extrinsic values explicitly marked PLACEHOLDER
[✅] Committed and pushed to remote
```

---

# STAGE 2 — PHYSICAL SENSOR VALIDATION 🔜 NEXT

## Purpose
Verify that the software-validated estimator produces correct results with **real hardware** on the bench. No flying. No autonomous output.

## Prerequisites
```bash
# On Jetson
cd ~/rio_flight_test/rio_stack\ copy
git pull origin alok/new/gpsdenied
python3 -m pytest tests/ -v  # must pass on Jetson too
```

## What To Implement Before Testing

| Item | File | Description |
|---|---|---|
| Startup report | `src/supervisor.py` | Print sensor config, frame, extrinsics, gravity, ESKF state at boot |
| Safety CLI flags | `src/supervisor.py` | `--no-mavlink` (default), `--enable-external-nav` required for MAVLink |

## Physical Tests

### Test 2A — 60-Second Stationary (Master Plan §64)

**Procedure:**
1. Place drone/rig on rigid table
2. Power on, run `python3 src/supervisor.py --no-mavlink`
3. Wait 60 seconds
4. Press Ctrl+C

**Pass Criteria:**
```text
[ ] Velocity RMS < 0.05 m/s
[ ] Max velocity < 0.3 m/s
[ ] Max position excursion < 0.5 m
[ ] Total distance < 0.5 m
[ ] No NaN or Inf in logs
[ ] Gravity norm = 9.78–9.83 m/s²
[ ] ZUPT active within 10 seconds
```

**Repeat for 5 minutes if 60-second passes.**

---

### Test 2B — Pure Rotation (Master Plan §65)

**Procedure:**
1. Start logging
2. Slowly yaw the drone ±90° by hand (no translation)
3. Slowly pitch ±30°
4. Slowly roll ±30°
5. Return to start orientation

**Pass Criteria:**
```text
[ ] Attitude tracks rotation visually
[ ] Position excursion < 2.0 m during yaw
[ ] Position excursion < 3.0 m during pitch/roll
[ ] No velocity spikes > 2.0 m/s
[ ] Lever-arm compensation produces no large false velocity
```

**If position excursion > 5m:** Extrinsic calibration is wrong — stop and calibrate T_B_R.

---

### Test 2C — Doppler Sign Verification (Master Plan §67)

**Procedure:**
1. Keep radar stationary
2. Move a reflective object toward the U300
3. Move the same object away from the U300
4. Check sign in logs

**Pass Criteria:**
```text
[ ] Approaching target → positive radial Doppler (after sign correction)
[ ] Receding target → negative radial Doppler
[ ] Sign applied exactly once (in U300 adapter)
```

---

### Test 2D — Hand Translation (Master Plan §66)

**Procedure:**
1. Mark start position on table/floor
2. Start logging
3. Slowly move drone forward 1 metre (measured with tape)
4. Stop and hold for 5 seconds
5. Return to start
6. Repeat for: 2m, 5m
7. Repeat for: backward, left, right

**Pass Criteria:**
```text
[ ] Forward 1m → estimated displacement 0.5–2.0m (order of magnitude correct)
[ ] Forward 5m → estimated displacement 2.5–10.0m (order of magnitude correct)
[ ] Direction of motion matches (forward = +X North in NED)
[ ] Return-to-start displacement < 50% of max displacement
[ ] Velocity sign matches motion direction
```

**NOTE:** Without calibrated extrinsics, absolute accuracy is not expected. We are validating that the system responds to real motion correctly, not that it measures 1.000m exactly.

---

### Test 2E — U200A Altitude (Master Plan §68)

**Procedure:**
1. Place rig at known height (e.g. table = 0.75m above floor)
2. Verify U200A range reading
3. Slowly raise/lower by 0.5m
4. Check ESKF altitude response

**Pass Criteria:**
```text
[ ] U200A range matches measured height ± 0.2m
[ ] ESKF altitude responds to height change
[ ] No altitude runaway when level
```

---

## Stage 2 Gate — All Must Pass Before Stage 3

```text
[ ] Test 2A (stationary) — PASS
[ ] Test 2B (rotation) — PASS
[ ] Test 2C (Doppler sign) — PASS
[ ] Test 2D (hand translation) — at least forward+backward PASS
[ ] Test 2E (U200A) — PASS
[ ] No crashes during any test
[ ] All data logged to JSONL
```

---

# STAGE 3 — SLAM INTEGRATION (Software)

## Purpose
Integrate the existing radar SLAM (scan-to-submap) code into the supervisor for geometric position correction. This adds a second source of position information beyond Doppler velocity.

## What To Implement

| Item | File | Description |
|---|---|---|
| Keyframe manager | `mapping/keyframe.py` [NEW] | Create keyframes at translation/rotation/time thresholds |
| SLAM frontend integration | `src/supervisor.py` | Feed radar point clouds to `mapping/registration.py` |
| Scan-to-submap pipeline | `mapping/local_submap.py` | Accumulate keyframes, provide target for registration |
| Quality gate | `mapping/registration.py` | Only accept registrations that pass fitness+conditioning |
| SLAM → ESKF feedback | `estimator/eskf_rio/estimator.py` | Use relative pose as position correction |
| SLAM health state | `estimator/health.py` | `SLAM_GOOD` / `SLAM_DEGRADED` states |
| Odom/map separation | `estimator/eskf_rio/estimator.py` | `odom→body` smooth, `map→odom` slow correction |
| SLAM unit tests | `tests/test_slam.py` [NEW] | identity, translation, rotation, sparse, degenerate |

## Prerequisites
```text
[  ] Open3D installed on Jetson: python3 -c "import open3d; print(open3d.__version__)"
[  ] Stage 2 gate passed
```

## Pass Criteria
```text
[ ] SLAM registration succeeds on recorded data
[ ] Scan-to-submap fitness > 0.3 for indoor environments
[ ] No position jump > 2m from single SLAM correction
[ ] SLAM failure → graceful degradation (ESKF continues with Doppler only)
[ ] SLAM unit tests pass
[ ] Replay of recorded data produces consistent trajectory
```

---

# STAGE 4 — GRAPH BACKEND + LOOP CLOSURE (Software)

## Purpose
Add pose-graph optimization and verified loop closure for long-term drift correction.

## What To Implement

| Item | File | Description |
|---|---|---|
| Pose graph | `mapping/pose_graph.py` [NEW] | Fixed-lag graph with IMU, Doppler, altitude, scan-match factors |
| Place recognition | `mapping/place_recognition.py` [NEW] | Deterministic Scan Context descriptors |
| Loop closure pipeline | `mapping/loop_closure.py` [NEW] | Candidate → verify → geometric check → factor |
| False loop rejection | `mapping/loop_closure.py` | Consistency test before accepting loop |
| Graph optimizer | `mapping/pose_graph.py` | Robust loss functions, bounded correction |
| Loop closure tests | `tests/test_loop_closure.py` [NEW] | True loop, false loop rejection, geometric verify |

## Prerequisites
```text
[  ] Stage 3 gate passed
[  ] SLAM produces valid registrations on real data
```

## Pass Criteria
```text
[ ] Closed-loop route: drift reduces after loop closure
[ ] False loop closure is rejected (not just accepted)
[ ] Graph optimization converges in < 100ms per window
[ ] Loop closure does not cause > 0.5m position jump in odom frame
[ ] Unit tests pass
```

---

# STAGE 5 — REPLAY + ANALYSIS TOOLS (Software)

## Purpose
Build deterministic replay and offline analysis infrastructure. Every flight must be reproducible and independently analyzed.

## What To Implement

| Item | File | Description |
|---|---|---|
| Deterministic replay | `tools/replay_run.py` | Replay JSONL logs, produce identical trajectory |
| Offline analyzer | `tools/analyze_run.py` [UPDATE] | ATE, RPE, velocity RMSE, heading error, altitude error, closure error |
| Ground truth comparison | `tools/ground_truth_compare.py` [NEW] | Compare RIO trajectory against GPS/RTK reference |
| Plotting | `tools/plot_run.py` [UPDATE] | Position, velocity, innovation, covariance, health over time |

## Pass Criteria
```text
[ ] Repeated replay of same data produces identical trajectory (bit-exact)
[ ] Analyzer correctly computes all metrics from Master Plan §82
[ ] Plots are clear and informative
```

---

# STAGE 6 — MAVLink + HEALTH GATE (Software)

## Purpose
Prepare the MAVLink ODOMETRY output for ArduPilot external navigation. This is the bridge between RIO and the flight controller.

## What To Implement

| Item | File | Description |
|---|---|---|
| Frame ID contract | `autopilot/mavlink_output.py` | Explicit `frame_id` / `child_frame_id` per ArduPilot firmware |
| NED → ArduPilot conversion | `autopilot/mavlink_output.py` | Explicit coordinate conversion at MAVLink boundary |
| Safety gate | `autopilot/mavlink_output.py` | Full pre-publish checklist (§55) |
| Watchdog | `src/supervisor.py` | IMU/radar/altimeter age, NaN, covariance PD |
| Full state machine | `estimator/health.py` | BOOT→INIT→IMU_ONLY→RADAR_GOOD→SLAM_GOOD→LOC_GOOD→DEGRADED→FAILSAFE |
| Navigation gate | `src/navigation_gate.py` [NEW] | Quality → speed limit → failsafe |

## Prerequisites
```text
[  ] Stages 3-5 passed
[  ] ArduPilot firmware version confirmed
[  ] ODOMETRY message format verified against docs
```

## Pass Criteria
```text
[ ] MAVLink output disabled by default (--enable-external-nav required)
[ ] Output pauses when health degrades
[ ] Covariance comes from ESKF posterior (never hardcoded)
[ ] ArduPilot EKF accepts and uses the ODOMETRY messages
[ ] No feedback loop: RIO → ArduPilot → RIO
```

---

# STAGE 7 — TETHERED + ASSISTED FLIGHT (Physical)

## Purpose
First airborne validation. Pilot-controlled, safety-tethered, low altitude.

## Prerequisites
```text
[  ] All Stage 2 bench tests re-passed after SLAM integration
[  ] Stage 6 gate passed
[  ] Physical safety tether attached
[  ] Pilot present with manual override
[  ] Low altitude (< 3m AGL)
```

## Flight Tests

### Test 7A — Hover (Master Plan §78)
```text
Procedure: Arm, take off to 1.5m, hover 30 seconds, land
Pass: position drift < 2m, velocity < 0.5 m/s, no oscillation
```

### Test 7B — Small Translations (Master Plan §78)
```text
Procedure: Hover, move 2m forward, hold, return, land
Pass: estimated displacement matches direction, < 50% scale error
Repeat: backward, left, right
```

### Test 7C — Yaw in Flight (Master Plan §78)
```text
Procedure: Hover, yaw 90°, hold, yaw back, land
Pass: position bounded during yaw, heading tracks rotation
```

## Stage 7 Gate
```text
[ ] Hover stable for 30 seconds
[ ] Forward/backward motion direction correct
[ ] Yaw does not cause large position jump
[ ] No crashes or failsafe triggers during normal flight
[ ] All data logged
```

---

# STAGE 8 — SMALL AUTONOMOUS FLIGHT (Physical)

## Purpose
First autonomous GPS-denied navigation. Small envelope, pilot ready to override.

## Prerequisites
```text
[  ] Stage 7 gate passed with at least 3 successful flights
[  ] ArduPilot configured for external navigation
[  ] Geofence set to 5m × 5m × 3m box
```

## Tests

### Test 8A — 2m × 2m Box (Master Plan §80)
```text
Procedure: Autonomous waypoint mission, 2m square at 1.5m altitude
Pass: completes route, returns within 1m of start, no failsafe
```

### Test 8B — 5m × 5m Box
```text
Procedure: Same but larger
Pass: completes route, returns within 2m of start
```

### Test 8C — 10m × 10m Box
```text
Procedure: Same but larger
Pass: completes route, returns within 3m of start
```

## Stage 8 Gate
```text
[ ] 2m box: 3 successful flights
[ ] 5m box: 3 successful flights
[ ] Position error < 30% of route length
[ ] Loop closure correction observed (if applicable)
```

---

# STAGE 9 — EXPANDED AUTONOMOUS FLIGHT

## Purpose
Larger GPS-denied missions with real operational requirements.

## Tests
```text
- Forward → right → backward → left → return (§81)
- Diagonal patterns
- Figure-eight (§74)
- Long closed-loop route (§75)
- GPS-denied replay comparison (§76)
```

## Pass Criteria
```text
[ ] ATE < 5% of path length
[ ] RPE < 0.5m per 10m segment  
[ ] Closure error < 1m for 100m route
[ ] Health system correctly detects degraded conditions
[ ] System enters DEGRADED/FAILSAFE when observability drops
```

---

# FLIGHT-READINESS CHECKLIST (Master Plan §99)

**The aircraft is NOT ready for unsupervised autonomous flight until ALL are true:**

```text
[ ] SENSOR CONTRACTS PASS — Stage 1
[ ] FRAME CONTRACT PASS — Stage 1
[ ] TIMING PASS — Stage 1
[ ] EXTRINSIC PASS — Stage 2 (calibrated T_B_R)
[ ] DOPPLER TESTS PASS — Stage 1 (16/16)
[ ] ESKF TESTS PASS — Stage 1 (15/15)
[ ] U200A TESTS PASS — Stage 2
[ ] SLAM TESTS PASS — Stage 3
[ ] LOOP TESTS PASS — Stage 4
[ ] FAULT INJECTION PASS — Stage 5
[ ] REPLAY DETERMINISM PASS — Stage 5
[ ] MAVLINK ODOMETRY PASS — Stage 6
[ ] GROUND-TRUTH TEST PASS — Stage 5
[ ] TETHERED FLIGHT PASS — Stage 7
[ ] ASSISTED FLIGHT PASS — Stage 7
[ ] SMALL AUTONOMOUS BOX PASS — Stage 8
[ ] RETURN-TO-START PASS — Stage 8
```

---

# QUICK REFERENCE — Current Status

| Stage | Status | Tests |
|---|---|---|
| 1. Software Foundation | ✅ **COMPLETE** | 50/50 pass |
| 2. Physical Sensor Validation | 🔜 **NEXT** | — |
| 3. SLAM Integration | ⏳ Blocked by Stage 2 | — |
| 4. Graph + Loop Closure | ⏳ Blocked by Stage 3 | — |
| 5. Replay + Analysis | ⏳ Blocked by Stage 3 | — |
| 6. MAVLink + Health Gate | ⏳ Blocked by Stage 5 | — |
| 7. Tethered + Assisted Flight | ⏳ Blocked by Stage 6 | — |
| 8. Small Autonomous Flight | ⏳ Blocked by Stage 7 | — |
| 9. Expanded Autonomous Flight | ⏳ Blocked by Stage 8 | — |

---

# RULES THAT APPLY AT EVERY STAGE

1. **No stage-skipping.** If Stage N fails, do not start Stage N+1.
2. **No `radar rejected → velocity = 0`.** Radar rejection = measurement unavailable.
3. **No hardcoded extrinsics.** All transforms come from `config/system.yaml`.
4. **No fake zero distance.** The estimator reports its true estimate + uncertainty.
5. **No autonomous output by default.** `--enable-external-nav` flag required.
6. **One authoritative state.** No averaging of competing trajectories.
7. **Log everything.** Every accepted/rejected measurement goes into the JSONL log.
8. **Validate against ground truth.** GPS is logged for reference, never fed to the estimator.
