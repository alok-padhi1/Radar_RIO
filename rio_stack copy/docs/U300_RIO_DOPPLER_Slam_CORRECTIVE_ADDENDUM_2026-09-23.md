# U300 RIO / Doppler / SLAM Corrective Addendum
## For Antigravity — Apply Before Any Further Flight Test

### Scope

Hardware:
- 1 × Linpowave U300, nose-mounted, forward-looking
- 1 × Linpowave U200A, belly-mounted, downward-looking
- Cube Orange+ IMU
- Jetson Orin Nano
- ArduPilot

This document supplements the Phase-1 plan. It is intentionally conservative: the goal is not to force the estimator to output a trajectory, but to ensure every output is physically and statistically defensible.

---

# 1. DO NOT IMPLEMENT THE FIVE PROPOSED FIXES BLINDLY

The proposed direction is useful, but the following must be corrected/added.

## Proposed Defect 1 — U300 axis permutation

The old project code documents a U300 native point convention of:

```text
X_R = lateral
Y_R = forward
Z_R = up
```

and converts this to body FRD using an axis permutation followed by a mechanical tilt rotation.

However, the public Linpowave U300 product page only states that it outputs X/Y/Z coordinates and relative velocity; it does not publicly establish the exact axis polarity in the detailed product specification.

Therefore:

### DO NOT accept the old axis matrix as "proven" solely because it exists in old code.

Treat it as a hypothesis to be physically verified.

Required test:

1. Place a known reflector directly in front of the radar boresight.
2. Verify which raw U300 axis changes.
3. Place it to the physical right.
4. Verify lateral sign.
5. Place it physically above/below.
6. Verify vertical sign.

Record the observed mapping.

Only then freeze:

```text
R_B_R
```

and keep the raw U300 coordinate convention isolated inside `u300_adapter`.

Source:
https://www.linpowave.com/product/4d-mmwave-radar-u300-for-drone

---

# 2. DOPPLER SOLVER — USE A HYBRID ROBUST PIPELINE

Do NOT require "RANSAC only".

Different published systems use different robust strategies. 4D iRIOM explicitly notes that RANSAC-like approaches can be unstable on sparse/non-repetitive radar data and uses iterative reweighted least squares; REVE uses a 3-point RANSAC least-squares estimator and explicitly outputs velocity variances for fusion.

Therefore implement:

```text
physical filtering
      ↓
self-return filtering
      ↓
preliminary weighted solve
      ↓
robust hypothesis generation
      ↓
RANSAC / multi-hypothesis gate
      ↓
IRLS refinement
      ↓
conditioning gate
      ↓
posterior sigma gate
      ↓
temporal acceleration gate
      ↓
final velocity + covariance
```

Sources:
https://arxiv.org/abs/2303.13962
https://github.com/christopherdoer/reve

---

# 3. LEVER-ARM COMPENSATION — DEFINE THE MODEL, DO NOT JUST COPY THE OLD FORMULA

Define:

```text
t_B_R =
radar origin relative to body origin,
expressed in body coordinates
```

For body angular velocity:

```text
omega_B
```

the radar-origin velocity is:

```text
v_R_origin_in_B =
    v_B_origin + omega_B × t_B_R
```

Therefore:

```text
v_B_origin =
    v_R_origin_in_B - omega_B × t_B_R
```

If correcting the scalar Doppler before solving, use:

```text
d_corrected_i =
    d_i + u_B_iᵀ (omega_B × t_B_R)
```

provided the canonical Doppler convention is:

```text d_i = -u_B_iᵀ v_B_origin + noise
```

The sign must be verified against the U300 bench experiment.

Do NOT:
- compensate twice,
- compensate after transforming with a second hidden transform,
- use radar-frame omega with body-frame lever arm,
- use body-frame omega with radar-frame lever arm.

All three vectors must be expressed consistently.

---

# 4. VZ PRIOR — USE U200A, NOT THE AUTOPILOT EKF, FOR THE AUTHORITATIVE PRIOR

The old code can use a flight-controller vertical velocity as a soft prior. Do NOT make that the primary solution in the new architecture.

Reason:

```text
Cube/ArduPilot EKF
      ↓
vertical velocity
      ↓
RIO
```

creates the estimator-feedback dependency that this project explicitly wanted to avoid.

The authoritative vertical aid should be:

```text
U200A
   ↓
filtered altitude / AGL
   ↓
vertical measurement in ESKF
```

If a Doppler solver needs an Earth-vertical soft constraint to break a U300 tilt degeneracy, derive it from the U200A/ESKF state with an explicit covariance.

Do not inject a hard Vz value.

Use:

```text
z_vz = v_down
sigma_vz = measured/validated uncertainty
```

not:

```text vz = 0
```

unless an independently confirmed stationary condition is active.

---

# 5. PROCESS NOISE — DO NOT ACCEPT THE BLIND 10× INCREASE

The proposed:

```text
accel: 0.01² → 0.1²
gyro: 0.005² → 0.01²
```

is not automatically correct.

A process-noise parameter is a sensor model, not a magic anti-drift knob.

Do this instead:

1. Record stationary IMU data for at least 10–30 minutes.
2. Measure:
   - mean
   - variance
   - Allan deviation if possible
   - bias stability
   - sample timing jitter
3. Determine whether the configured continuous-time noise densities are plausible.
4. Use those measured values as the initial process-noise parameters.
5. Validate on replay.
6. Tune only one noise term at a time.

If radar is rejected and the IMU-only solution grows rapidly, the system must diagnose WHY:
- attitude error,
- accelerometer bias,
- gravity sign,
- timing,
- calibration,
- integration,
- or genuine sensor noise.

Do not simply increase Q until the trace looks nice.

---

# 6. CRITICAL DEFECT NOT IN THE AGENT'S FIVE: HEALTH MANAGER IS STILL WRONG

The latest physical run demonstrates:

```text estimator velocity → >30 m/s
estimator position → hundreds of metres
navigation mode → RADAR_VELOCITY_GOOD
```

That is unacceptable.

The health layer must independently validate the state.

Add mandatory gates:

```text max_velocity
max_acceleration
max_position_rate
innovation
Mahalanobis
covariance
state freshness
radar measurement freshness
radar accepted timestamp
radar residual
```

Example:

```text if velocity > configured_vehicle_limit:
    estimator_valid = false

if repeated radar innovation rejection:
    radar_health = false

if accepted_radar_age > timeout:
    radar_health = false

if covariance > threshold:
    navigation_valid = false
```

Do not leave the previous `n_static_points` or previous accepted radar state in the health manager when the current frame is rejected.

Every radar frame must explicitly be classified:

```text ACCEPTED
REJECTED
NO_DATA
STALE
```

---

# 7. CRITICAL DEFECT: STALE VALIDITY

An accepted radar frame must not make the system "RADAR_VELOCITY_GOOD" indefinitely.

Maintain:

```text last_radar_velocity_accept_time
last_slam_accept_time
last_u200a_accept_time
```

Health must be based on those timestamps.

Example:

```text radar accepted at t=100.0
current t=102.0
timeout=0.2
→ radar NOT healthy
```

Do not use:
- previous point count,
- previous static count,
- previous residual
as current-frame health.

---

# 8. CRITICAL DEFECT: ESKF ATTITUDE OBSERVABILITY

The current ESKF architecture is still primarily IMU-propagated and does not have a conventional gravity-based attitude correction update after initialization.

A small attitude error leaks gravity into horizontal velocity:

```text gravity error ≈ g * sin(attitude_error)
```

For example, a 1° tilt error creates approximately:

```text 9.81 * sin(1°) ≈ 0.171 m/s²
```

of false acceleration in the affected horizontal direction.

Therefore add a validated attitude/tilt stabilization mechanism:
- stationary gravity alignment
- ZUPT attitude observability
- radar velocity update attitude coupling
- optionally a low-dynamics accelerometer gravity update when its assumptions are valid

Do NOT assume gyro integration will remain perfect for long flights.

---

# 9. CRITICAL DEFECT: DO NOT CLAMP BAD STATE

Do not fix:

```text velocity > 50 m/s
```

by:

```text velocity = 50 m/s
```

That hides the failure.

Instead:

```text velocity > physical limit
    ↓
state invalid
    ↓
health degraded
    ↓
measurement source diagnostic
```

The estimator must preserve the evidence of the fault in logs.

---

# 10. CRITICAL DEFECT: COVARIANCE MUST MATCH THE TRANSFORM

If radar velocity is estimated in radar coordinates:

```text P_R
```

and transformed to body coordinates:

```text v_B = R_B_R v_R
```

then:

```text P_B =
    R_B_R P_R R_B_Rᵀ
```

The lever-arm correction changes the velocity mean but does not create arbitrary additional covariance unless angular-rate uncertainty and extrinsic uncertainty are explicitly propagated.

If omega/extrinsic uncertainty is ignored, that limitation must be documented.

---

# 11. CRITICAL DEFECT: RADAR TIMESTAMP MUST BE THE MEASUREMENT EPOCH

At each U300 frame:

```text radar_timestamp
```

must represent the best available estimate of when the scan was measured.

Do not mix:
- parse time,
- serial receive time,
- queue delivery time,
- estimator processing time.

For first validation, a common Jetson monotonic domain is acceptable if arrival-time error is characterized.

Then refine temporal offset using controlled motion.

Recent RIO research shows temporal delay can materially affect radar-inertial localization.

Source:
https://arxiv.org/abs/2503.02509

---

# 12. CRITICAL DEFECT: FRAME TIMESTAMP + IMU INTERPOLATION

At radar time:

```text t_R
```

integrate the IMU to:

```text t_R
```

and use interpolation where required.

Do not simply fuse:
```text latest IMU
```
against:
```text latest radar
```

when their timestamps differ.

---

# 13. DOPPLER POINT MODEL

For each point:

```text u_i = p_i / ||p_i||
```

and:

```text d_i = -u_iᵀ v_R + n_i
```

Build:

```text A v = b
```

and explicitly calculate:

```text rank(A)
cond(A)
residual RMS
effective inlier count
covariance
```

Do not call:

```text N >= 3
```

sufficient.

---

# 14. SPARSE-PROFILE RULE

Use separate thresholds for:

```text raw point count
usable point count
inlier count
effective angular diversity
conditioning
```

Three points may mathematically solve three unknowns but still be useless because the directions are nearly dependent.

---

# 15. SELF-RETURN RULE

Build a static airframe mask from real U300 data.

Potential sources:
- radar bracket
- frame
- arms
- wiring
- landing hardware

Do not classify self-returns only by Doppler.

They can have:

```text Doppler ≈ 0
```

while being attached to the aircraft.

---

# 16. DYNAMIC OBJECT RULE

Use:
- robust residual rejection
- temporal persistence
- spatial consistency
- map consistency

Moving objects must not dominate the ego-velocity solution.

---

# 17. SLAM — DO NOT ADD IT UNTIL DOPPLER + ESKF PASS

The correct sequence is:

```text sensor
 ↓
frame
 ↓
timing
 ↓
Doppler
 ↓
ESKF
 ↓
physical validation
 ↓
SLAM
```

Once stable, integrate:

```text U300 scan
 ↓
preprocess
 ↓
deskew
 ↓
body-frame transform
 ↓
scan-to-submap
 ↓
robust registration
 ↓
relative pose + covariance
 ↓
ESKF / fixed-lag graph
```

Research supports combining Doppler and geometric registration instead of choosing one. A 2026 4D radar study reports improved localization when ICP-style relative pose and Doppler ego-velocity are integrated in a consistent EKF compared with either source alone.

Source:
https://www.mdpi.com/1424-8220/26/5/1660

---

# 18. SCAN-TO-SUBMAP

Use:

```text current scan → local submap
```

rather than relying only on:

```text current scan → previous scan
```

The iRIOM paper specifically reports advantages from scan-to-submap matching for sparse, non-repetitive radar and combines that with radar ego-velocity and IMU in an iterated EKF.

Source:
https://arxiv.org/abs/2303.13962

---

# 19. SLAM REGISTRATION REQUIREMENTS

A registration is valid only when:

```text enough points
enough correspondences
acceptable residual
acceptable fitness
acceptable translation
acceptable rotation
usable Hessian
good conditioning
consistent with ESKF
```

The output must contain:

```text relative_transform
translation
rotation
RMSE
fitness
correspondence_count
condition_number
covariance
valid
reason
```

---

# 20. LOOP CLOSURE

Implement:

```text candidate place recognition
 ↓
geometric verification
 ↓
relative pose
 ↓
robust consistency gate
 ↓
graph factor
 ↓
pose graph optimization
```

Never accept a loop based only on descriptor similarity.

A false loop can be worse than no loop.

---

# 21. MAP / ODOM SEPARATION

Use:

```text map → odom → body
```

Loop closure changes:

```text map → odom
```

smoothly.

The aircraft controller should receive a continuous local odometry frame.

Do not suddenly teleport the control frame when a loop closes.

---

# 22. U200A ROLE

U200A should:
- constrain vertical state
- validate takeoff/landing height
- provide optional Vz information
- support terrain following

It should not be the primary horizontal navigation solution.

---

# 23. POSITION / DISPLACEMENT / DISTANCE DEFINITIONS

Log all independently.

Position:

```text p(t) = [N,E,D]
```

Net displacement:

```text Δp = p_final - p_initial
```

Magnitude:

```text ||Δp||
```

Path distance:

```text Σ ||p_k-p_(k-1)||
```

Course:

```text atan2(VE,VN)
```

Body velocity:

```text [forward,right,down]
```

Never label path distance as displacement.

---

# 24. "ONE-METRE OUT AND BACK" TEST

For a controlled experiment:

```text start
 ↓
move exactly 1 m
 ↓
return to the same physical point
 ↓
stop
```

The important outputs are:

```text maximum excursion
final displacement
path length
velocity RMSE
heading change
altitude change
radar accepted/rejected timeline
SLAM accepted/rejected timeline
covariance timeline
```

A perfect result would not literally mean numerical zero.

It means the estimate remains within a previously established error bound validated against independent ground truth.

---

# 25. INDEPENDENT GROUND TRUTH

For high-accuracy claims, use:
- motion capture indoors, or
- RTK/PPK outdoors as a validation reference, or
- another independent tracking system.

Do not use RIO to prove RIO.

---

# 26. REQUIRED "SENSOR-TO-STATE" DEBUG TABLE

Every run must produce:

```text
timestamp
R1/radar point count
Doppler accepted?
Doppler velocity
Doppler covariance
Doppler condition number
ESKF predicted velocity
ESKF posterior velocity
innovation
Mahalanobis
U200A
SLAM accepted?
SLAM relative motion
SLAM covariance
final position
final velocity
health state
```

This will finally let you answer:

```text Was Doppler wrong?
Was IMU wrong?
Was attitude wrong?
Was extrinsic wrong?
Was SLAM wrong?
Was timing wrong?
Was the health manager wrong?
```

---

# 27. CURRENT RUN — WHAT THE FAILURE SUGGESTS

The observed sequence:

```text stable stationary
        ↓
physical movement
        ↓
position/velocity grow rapidly
        ↓
position ends hundreds of metres away
        ↓
velocity exceeds 30 m/s
        ↓
health still says RADAR_VELOCITY_GOOD
```

strongly indicates that the remaining issue is not merely the distance counter.

The estimator itself is diverging.

Likely classes to distinguish experimentally:

1. radar-frame/extrinsic error
2. attitude/gravity leakage
3. radar measurement rejection/dropout
4. incorrect radar lever-arm handling
5. Doppler sign/model error
6. stale health state
7. timing mismatch

Do not choose one without the diagnostic logs.

---

# 28. NEXT PHYSICAL TEST — DO NOT FLY

After implementing the corrections:

### Test 1
60 s stationary

### Test 2
90° pure yaw in place

### Test 3
forward 1 m and stop

### Test 4
backward 1 m and stop

### Test 5
right 1 m and stop

### Test 6
left 1 m and stop

### Test 7
1 m out-and-back

### Test 8
vertical 0.5 m up/down using controlled lifting/stand

Do all on a safe ground/bench procedure, not autonomous flight.

---

# 29. REQUIRED PASS CONDITIONS BEFORE SLAM

Every test must show:

```text no unbounded velocity
no unbounded position
no hidden clipping
no stale "RADAR_GOOD"
no invalid covariance
correct signs
correct direction
correct displacement scale
```

Only then integrate SLAM.

---

# 30. FINAL IMPLEMENTATION ARCHITECTURE

```text
                 U300 NOSE RADAR
                        │
                raw point + Doppler
                        │
                        ▼
                radar preprocessing
                        │
                 self-return mask
                        │
                 dynamic rejection
                        │
                        ▼
              robust Doppler estimator
                        │
             velocity + covariance
                        │
                        ▼
                radar → body FRD
                 + lever arm
                        │
                        ▼
Cube IMU ───────────► ESKF ◄──────── U200A
                        │
                        ▼
                 local odometry
                        │
                        ▼
               Radar scan frontend
                        │
                 scan-to-submap
                        │
                  relative pose
                        │
                        ▼
               fixed-lag/graph layer
                        │
                  loop closure
                        │
                        ▼
               final map/odom pose
                        │
                 health + covariance
                        │
                        ▼
                 MAVLink ODOMETRY
                        │
                        ▼
                    ArduPilot
```

---

# 31. EXTERNAL RESEARCH BASIS

### REVE
https://github.com/christopherdoer/reve

Uses robust 3-point RANSAC least squares for 3D radar ego-velocity and estimates velocity variance for subsequent fusion.

### x-RIO
https://github.com/christopherdoer/rio

EKF radar-inertial odometry and drone navigation reference.

### BRIO
https://github.com/ethz-asl/rio

Multicopter-oriented robust radar-inertial odometry with barometer and zero-velocity support.

### HKUST RIO
https://github.com/HKUST-Aerial-Robotics/RIO

4D radar point uncertainty and radar-inertial optimization reference.

### RIV-SLAM
https://github.com/Wayne-DWA/RIV-SLAM

Radar scan matching + IMU/velocity/ground factors + graph SLAM + loop closure.

### 4D iRIOM
https://arxiv.org/abs/2303.13962

Radar ego-velocity + scan-to-submap + iterative EKF + loop closure.

### DopRIO
https://www.sciencedirect.com/science/article/pii/S0921889026002708

Recent tightly coupled Doppler-enhanced radar-inertial odometry direction.

### Radar ICP integrated navigation
https://www.mdpi.com/1424-8220/26/5/1660

Recent result supporting combined Doppler + ICP constraints.

### Temporal calibration
https://arxiv.org/abs/2503.02509

Demonstrates that radar/IMU temporal delay can materially affect localization.

---

# 32. FINAL RULE

Do not attempt to obtain "perfect numbers" from the display.

Obtain a defensible estimator:

```text correct state
+
correct covariance
+
correct measurement rejection
+
correct health
+
correct frame
+
correct time
+
independent validation
```

Only after the estimator passes these checks should SLAM be connected to the flight path, and only after RIO+SLAM are validated should autonomous external navigation be enabled.
