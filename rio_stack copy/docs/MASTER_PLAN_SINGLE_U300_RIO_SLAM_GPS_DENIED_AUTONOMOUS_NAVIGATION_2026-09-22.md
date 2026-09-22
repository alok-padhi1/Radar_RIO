# MASTER ENGINEERING PLAN
# Single Linpowave U300 + Cube Orange+ IMU + U200A for GPS-Denied RIO + Radar SLAM + Autonomous UAV Navigation

**Date:** 2026-09-22  
**Hardware configuration:** EXACTLY 1 × Linpowave U300 + 1 × Linpowave U200A + Cube Orange+ IMU + Jetson Orin Nano  
**U300 location:** nose/front of UAV, forward-looking  
**U200A location:** belly/downward-looking  
**Flight controller:** Cube Orange+ running ArduPilot  
**Compute:** NVIDIA Jetson Orin Nano  
**Mission:** local autonomous navigation in GPS/GNSS-denied environments  
**Primary estimator:** U300 Doppler + IMU Error-State Kalman Filter (ESKF)  
**Geometric localization:** U300 radar scan-to-submap SLAM  
**Long-term correction:** pose graph + verified loop closure  
**Vertical aid:** U200A altitude/AGL update  
**Autopilot interface:** MAVLink ODOMETRY / ArduPilot External Navigation  
**Important:** This document is an implementation specification and validation plan. It does not and cannot guarantee zero-error flight in every environment. The system is considered flight-ready only after passing the staged validation gates described here.

---

# 1. FINAL HARDWARE ARCHITECTURE

The aircraft has one forward-looking U300.

```text
                         FRONT / NOSE
                              ↑
                              │
                       ┌─────────────┐
                       │   U300      │
                       │ 4D RADAR   │
                       └──────┬──────┘
                              │
                         forward scene
                              │
                    ┌────────────────────┐
                    │       UAV          │
                    │                    │
                    │  Cube Orange+ IMU  │
                    │                    │
                    │  Jetson Orin Nano  │
                    └──────────┬─────────┘
                               │
                               ↓
                        ┌─────────────┐
                        │    U200A    │
                        │  ALT/RANGE  │
                        └──────┬──────┘
                               ↓
                             GROUND
```

Roles:

```text
U300:
  radar Doppler
  radar ego velocity
  forward radar geometry
  scan matching
  local radar SLAM
  map/submap structure

Cube IMU:
  high-rate inertial propagation
  attitude
  acceleration
  angular velocity
  bias estimation

U200A:
  downward altitude / AGL measurement
  terrain/vertical constraint

Jetson:
  all sensor fusion
  RIO
  Doppler
  SLAM
  graph optimization
  health
  logging
  external navigation output

ArduPilot:
  attitude/rate control
  actuator control
  flight stabilization
  mission execution
  external-navigation fusion
```

---

# 2. CORE DESIGN DECISION

Do NOT choose one repository and copy it wholesale.

Build one U300-specific estimator from complementary research.

## Research lineage to use

### HKUST RIO
https://github.com/HKUST-Aerial-Robotics/RIO

Use:
- 4D radar uncertainty concepts
- radar point residual concepts
- IMU preintegration
- robust radar data association

Do not blindly copy:
- radar drivers
- sensor frame assumptions
- ARS548-specific fields
- calibration
- noise values

### x-RIO / REVE
https://github.com/christopherdoer/rio

Use:
- radar ego-velocity concept
- EKF-style radar-inertial update
- covariance-aware velocity measurement
- extrinsic calibration concepts
- multi-radar concepts as design references only

### BRIO / ETHZ
https://github.com/ethz-asl/rio

Use:
- multicopter-focused radar-inertial estimation
- robust estimation
- zero-velocity tracking
- altitude/barometric assistance
- GNSS-denied flight methodology
- real flight validation methodology

### RIV-SLAM
https://github.com/Wayne-DWA/RIV-SLAM

Use:
- radar scan matching
- adaptive/probabilistic GICP ideas
- IMU preintegration factor concepts
- velocity factor concepts
- ground factor concepts
- graph SLAM
- sliding-window optimization
- loop closure / Scan Context style place recognition

### 4D iRIOM
https://arxiv.org/abs/2303.13962

Use:
- separation of radar ego-velocity and scan-to-submap matching
- inertial fusion
- robust radar velocity
- submap localization
- loop closure

The final software should have ONE authoritative state estimator, not multiple competing RIO outputs.

---

# 3. FINAL SYSTEM

```text
                  ┌────────────────────┐
                  │ U300 NOSE RADAR    │
                  │ XYZ + Doppler      │
                  └─────────┬──────────┘
                            │
              ┌─────────────┴─────────────┐
              │                           │
              ▼                           ▼
       Radar preprocessing         Radar point cloud
              │                           │
              ▼                           ▼
       Doppler ego velocity       Scan-to-submap SLAM
              │                           │
              │                           ▼
              │                    Relative pose
              │                           │
              └─────────────┬─────────────┘
                            │
                            ▼
                    RADAR → BODY
                   calibrated transform
                            │
                            ▼
                      ┌──────────┐
 Cube IMU ───────────►│   ESKF   │◄──────── U200A
                      │          │
                      │ IMU      │
                      │ Doppler  │
                      │ Altitude │
                      │ ZUPT     │
                      └────┬─────┘
                           │
                    local odometry
                           │
                           ▼
                    SLAM / graph
                           │
                    loop closure
                           │
                           ▼
                    final odometry
                           │
                     health gate
                           │
                           ▼
                    MAVLink ODOMETRY
                           │
                           ▼
                       ArduPilot
```

---

# 4. WHY ONE U300 CAN STILL WORK

One U300 is sufficient for building a radar-inertial navigation system, because the IMU supplies high-rate propagation and the radar supplies body-motion constraints and geometric observations.

Published radar-inertial systems demonstrate that a single radar can provide useful odometry. x-RIO reports single-radar results and autonomous radar-inertial drone navigation. BRIO specifically targets multicopter radar-inertial navigation in difficult GNSS-denied environments.

However:

**one forward radar does NOT provide guaranteed 360-degree sensing.**

That means:

- forward motion can have strong geometric support when the forward scene is rich
- sideways motion can still be estimated using IMU + Doppler + scene geometry when enough returns exist in the FOV
- backward motion can be estimated while the existing scene remains observable in the radar history/submap, but current observations do not directly cover the rear hemisphere
- pure motion in a featureless region can become weakly observable
- absolute yaw is not observable from gravity alone
- prolonged radar dropout causes uncertainty to grow

The correct behavior during weak observability is:

```text
uncertainty ↑
    ↓
health ↓
    ↓
degraded navigation
    ↓
configured autopilot fallback
```

NOT:

```text
weak observations
    ↓
fake confidence
```

---

# 5. REQUIRED SOURCE CODE STRUCTURE

Use one authoritative codebase.

```text
uav_rio/
│
├── config/
│   ├── sensors.yaml
│   ├── frames.yaml
│   ├── extrinsics.yaml
│   ├── timing.yaml
│   ├── imu.yaml
│   ├── doppler.yaml
│   ├── eskf.yaml
│   ├── slam.yaml
│   ├── graph.yaml
│   └── flight_gate.yaml
│
├── src/
│   ├── drivers/
│   │   ├── u300/
│   │   │   ├── u300_reader.cpp
│   │   │   ├── u300_decoder.cpp
│   │   │   └── u300_adapter.cpp
│   │   ├── cube/
│   │   │   ├── imu_reader.cpp
│   │   │   └── mavlink_clock.cpp
│   │   └── u200a/
│   │       ├── u200a_reader.cpp
│   │       └── u200a_adapter.cpp
│   │
│   ├── calibration/
│   │   ├── imu_calibration.cpp
│   │   ├── u300_extrinsic.cpp
│   │   └── temporal_calibration.cpp
│   │
│   ├── preprocessing/
│   │   ├── radar_filter.cpp
│   │   ├── self_return_mask.cpp
│   │   ├── dynamic_filter.cpp
│   │   ├── radar_motion_compensation.cpp
│   │   └── radar_quality.cpp
│   │
│   ├── doppler/
│   │   ├── ego_velocity.cpp
│   │   ├── robust_solver.cpp
│   │   ├── covariance.cpp
│   │   └── observability.cpp
│   │
│   ├── estimator/
│   │   ├── eskf.cpp
│   │   ├── imu_propagation.cpp
│   │   ├── radar_velocity_update.cpp
│   │   ├── altitude_update.cpp
│   │   ├── zupt.cpp
│   │   └── estimator_health.cpp
│   │
│   ├── slam/
│   │   ├── deskew.cpp
│   │   ├── radar_representation.cpp
│   │   ├── gicp.cpp
│   │   ├── distribution_registration.cpp
│   │   ├── submap.cpp
│   │   └── slam_quality.cpp
│   │
│   ├── graph/
│   │   ├── fixed_lag_graph.cpp
│   │   ├── pose_graph.cpp
│   │   ├── loop_closure.cpp
│   │   └── graph_optimizer.cpp
│   │
│   ├── navigation/
│   │   ├── odometry_output.cpp
│   │   ├── mavlink_odometry.cpp
│   │   ├── health_gate.cpp
│   │   └── failsafe.cpp
│   │
│   └── main/
│       └── rio_node.cpp
│
├── tests/
│   ├── frames/
│   ├── timing/
│   ├── doppler/
│   ├── eskf/
│   ├── altitude/
│   ├── slam/
│   ├── graph/
│   └── replay/
│
├── tools/
│   ├── record_run.py
│   ├── replay_run.py
│   ├── analyze_run.py
│   ├── plot_run.py
│   ├── calibrate_u300.py
│   ├── calibrate_time.py
│   └── ground_truth_compare.py
│
├── logs/
│
└── docs/
    ├── SENSOR_CONTRACTS.md
    ├── FRAME_CONTRACT.md
    ├── CALIBRATION.md
    ├── DOPPLER.md
    ├── SLAM.md
    ├── TEST_PLAN.md
    ├── FLIGHT_READINESS.md
    └── FAILURE_MODES.md
```

---

# 6. FRAME CONTRACT

Use one global convention.

Recommended:

```text
Body frame: FRD
+X = forward
+Y = right
+Z = down

World frame: NED
+X = north
+Y = east
+Z = down
```

The U300 has its own native radar frame.

Do not assume its XYZ axes are already the same as FRD.

Create explicit transform:

```text
T_B_R
```

where:
- B = Cube body frame
- R = U300 radar frame

And:

```text
T_B_A
```

for U200A.

Never mix:
- FLU
- FRD
- ENU
- NED
- Z-up
- Z-down

without an explicit conversion.

---

# 7. U300 EXTRINSIC CALIBRATION

Required:

```text
T_B_R =
[R_B_R  t_B_R
 0       1]
```

This contains:
- radar position relative to Cube/vehicle reference
- radar rotation relative to body

Do not use placeholder values such as:

```text
[0.16, 0.05, 0.07]
[0, 50, 0]
```

as if calibrated.

The current project must replace all hidden/hard-coded transforms with this single configured transform.

Validation motions:

```text
pure yaw
pure roll
pure pitch
forward translation
lateral translation
backward translation
```

A rotation must not create a large false translational velocity.

---

# 8. U200A EXTRINSIC CALIBRATION

Create:

```text
T_B_A
```

because U200A is mounted on the belly.

Its beam direction must be represented explicitly.

When the aircraft is level:

```text altitude ≈ U200A ground range
```

only after confirming the physical mounting and coordinate convention.

During large pitch/roll:
- the measurement becomes oblique
- ground intersection changes
- covariance must increase or measurement must be rejected

---

# 9. TIME CONTRACT

Use a common Jetson monotonic time domain for the first validated implementation.

Every sample records:

```text
host_timestamp
sensor_timestamp_if_available
frame_id
sequence_id
```

At minimum:

```text IMU -> Jetson monotonic
U300 -> Jetson monotonic
U200A -> Jetson monotonic
```

Do not directly compare unrelated:
- Cube boot time
- Unix time
- Jetson monotonic
- radar sensor time

Later introduce explicit calibrated offsets:

```text
Δt_U300_IMU
Δt_U200A_IMU
```

Recent radar-inertial research treats temporal delay as a significant source of localization error. Therefore time alignment must be measured, not guessed.

---

# 10. IMU INPUT POLICY

Choose ONE well-defined MAVLink source.

Preferred:
- HIGHRES_IMU if reliably available

Alternative:
- SCALED_IMU / SCALED_IMU2 with documented units

Do not treat RAW_IMU as though it had SCALED_IMU units.

Every conversion must be unit-tested.

The reader must output:

```text
timestamp
acceleration_mps2
gyro_radps
source
frame
```

No duplicate samples.

The supervisor must enqueue every new IMU sample exactly once.

---

# 11. SENSOR QUEUES

Use:

```text
IMU queue
U300 queue
U200A queue
```

Do not rely on "latest sample only" for fusion.

At radar timestamp:

```text
integrate all IMU samples up to radar time
interpolate state if required
apply radar update
```

At U200A timestamp:

```text
integrate IMU to U200A time
apply altitude update
```

---

# 12. U300 PREPROCESSING PIPELINE

```text
raw U300 frame
      ↓
decode
      ↓
timestamp validation
      ↓
range gate
      ↓
FOV gate
      ↓
Doppler gate
      ↓
invalid-point rejection
      ↓
self-reflection mask
      ↓
dynamic/outlier filtering
      ↓
temporal persistence
      ↓
quality scoring
      ↓
Doppler estimator + SLAM frontend
```

Every accepted point retains:

```text
frame_id
timestamp
x
y
z
range
azimuth
elevation
doppler
quality
```

---

# 13. AIRFRAME SELF-RETURN MASK

Because the U300 is nose-mounted, it can see:
- frame
- arms
- brackets
- wiring
- carbon structures
- landing hardware

Do a stationary collection procedure:

```text
several minutes
radar fixed
aircraft stationary
```

Find persistent detections in radar coordinates.

Create a static self-return mask.

Do not rely only on Doppler because airframe reflections are themselves stationary and therefore near-zero Doppler.

---

# 14. ROBUST DOPPLER EGO-VELOCITY

For each accepted static point:

```text
u_i =
[
cos(phi_i) cos(theta_i)
cos(phi_i) sin(theta_i)
sin(phi_i)
]
```

where:
- theta = azimuth
- phi = elevation

Canonical Doppler model:

```text
d_i = -u_i^T v_R + n_i
```

Construct:

```text A v_R = b
```

with:

```text A_i = -u_i^T
b_i = d_i
```

Solve using robust weighted least squares / IRLS.

---

# 15. DOPPLER ROBUST SOLVER

Implementation stages:

## Stage 1 — physical gate

Reject:
- NaN
- Inf
- invalid range
- invalid angle
- impossible Doppler

## Stage 2 — initial solve

Use weighted least squares.

## Stage 3 — IRLS

Huber/Cauchy-style robust weighting.

## Stage 4 — residual

```text r_i = d_i + u_i^T v_hat
```

## Stage 5 — inlier selection

Use robust residual scale.

## Stage 6 — final solve

Recompute:

```text v_hat
```

using final weights.

## Stage 7 — geometry gate

Require:

```text rank(A) = 3
```

and acceptable condition number.

## Stage 8 — temporal consistency

Reject impossible velocity jumps.

---

# 16. IMPORTANT SPARSE RADAR RULE

Do not automatically accept:

```text 3 points = valid 3D velocity
```

Three points may be algebraically sufficient and still physically useless.

Require:
- angular diversity
- matrix rank
- conditioning
- residual consistency
- temporal consistency

If insufficient:

```text radar_velocity.valid = false
```

NOT:

```text radar velocity = 0
```

---

# 17. DOPPLER COVARIANCE

Each Doppler estimate produces:

```text
v_R
P_vR
```

Use weighted information:

```text Λ = Aᵀ W A
```

and derive/inflate covariance according to:
- effective residual noise
- number of points
- conditioning
- quality
- sparse geometry

Poor geometry must produce larger covariance.

Good geometry may produce smaller covariance.

Never use an unrealistically fixed covariance for all conditions.

---

# 18. RADAR VELOCITY TRANSFORM

The Doppler estimate is initially expressed in radar coordinates.

Transform to body:

```text v_B = R_B_R v_R
```

and compensate radar-origin velocity caused by angular motion.

For radar lever arm:

```text t_B_R
```

use the rigid-body relationship with:

```text ω_B × t_B_R
```

The exact sign must be validated experimentally.

The implementation must never mix:
- radar velocity
- body velocity
- world velocity

without explicit transforms.

---

# 19. ESKF STATE

Nominal:

```text p_W
v_W
q_WB
b_a
b_g
```

Error state:

```text δp
δv
δθ
δb_a
δb_g
```

15-dimensional error state.

Covariance:

```text P ∈ R^(15×15)
```

---

# 20. IMU PROPAGATION

Measurement model:

```text a_m = a_true + b_a + n_a
ω_m = ω_true + b_g + n_g
```

Correct:

```text a = a_m - b_a
ω = ω_m - b_g
```

Propagate:

```text q_{k+1} = q_k ⊗ Exp(ω Δt)
```

```text v_{k+1} = v_k + (R_WB a + g_W) Δt
```

```text p_{k+1} = p_k + v_k Δt + 0.5(a_W + g_W)Δt²
```

Use appropriate discretized covariance propagation.

---

# 21. GRAVITY INITIALIZATION

During stationary initialization:

```text 5–10 seconds
```

Estimate average acceleration direction.

Use this to initialize roll/pitch.

Do NOT simply set the gravity vector by magnitude while keeping identity attitude if the body is not aligned with the world frame.

Yaw remains unobservable from gravity alone.

Initialize local yaw to zero unless another heading constraint exists.

---

# 22. RADAR VELOCITY ESKF UPDATE

Radar measurement:

```text z_v = h_v(x) + n_v
```

Residual:

```text r = z_v - h_v(x^-)
```

Innovation:

```text S = H P Hᵀ + R_v
```

Mahalanobis:

```text D² = rᵀ S⁻¹ r
```

If consistent:

```text K = P Hᵀ S⁻¹
```

Apply:

```text δx = K r
```

Inject into nominal state.

Update covariance robustly.

If inconsistent:

```text reject radar update
```

and continue IMU propagation with increased uncertainty.

---

# 23. ZUPT

ZUPT is a separate measurement.

Only generate:

```text v_B = 0
```

when an independent stationary detector confirms:

```text ||gyro|| small
||accel|-g| small
low accel variance
low gyro variance
sustained duration
```

Radar rejection is NEVER sufficient to trigger ZUPT.

This rule is mandatory.

---

# 24. U200A ALTITUDE UPDATE

U200A provides the vertical constraint.

Use:

```text z_h = h(x) + n_h
```

For level aircraft and downward beam:

```text h ≈ measured ground range
```

but only after confirming frame and beam orientation.

During high pitch/roll:
- reject
- or increase covariance
- or compensate using attitude and beam geometry

Do not allow U200A to create incorrect horizontal motion.

---

# 25. SLAM FRONTEND

The SLAM frontend uses U300 geometry.

Pipeline:

```text
current U300 scan
      ↓
timestamp validation
      ↓
motion compensation / deskew where possible
      ↓
radar self-return rejection
      ↓
dynamic/outlier filtering
      ↓
body-frame transformation
      ↓
voxelization / radar representation
      ↓
initial pose from ESKF
      ↓
scan-to-submap registration
      ↓
robust refinement
      ↓
quality/observability test
      ↓
relative pose factor
```

---

# 26. SCAN-TO-SUBMAP

Prefer:

```text current scan → local submap
```

over:

```text current scan → previous scan only
```

The submap contains several validated keyframes.

Example:

```text
k-8
k-7
k-6
k-5
k-4
k-3
k-2
k-1
current
```

The submap size must be tuned from replay data.

---

# 27. RADAR REPRESENTATION

U300 is sparse and noisy.

Do not blindly apply LiDAR assumptions.

Support:
- point-to-plane robust GICP
- distributional registration using local Gaussian neighborhoods

For a neighborhood:

```text μ_i
Σ_i
```

can represent local radar structure more robustly than exact point correspondence.

Use the model appropriate to observed U300 data quality.

---

# 28. GICP QUALITY

A registration is valid only when:

```text enough source points
enough target points
enough correspondences
residual acceptable
fitness acceptable
translation plausible
rotation plausible
Hessian usable
conditioning acceptable
innovation consistent with ESKF
```

A function returning a transform does not automatically mean:

```text SLAM = valid
```

---

# 29. SLAM COVARIANCE

Every accepted relative pose must carry uncertainty:

```text T_relative
R_relative
```

Inflate covariance when:
- few correspondences
- poor geometry
- high residual
- sparse radar
- dynamic scene
- degenerate Hessian

The graph must not trust a weak registration as strongly as a well-observed registration.

---

# 30. KEYFRAMES

Create a keyframe when one or more conditions occur:

```text translation > threshold
OR rotation > threshold
OR time > threshold
OR scene changes substantially
```

Do not make every U300 frame a keyframe.

Each keyframe stores:

```text timestamp
pose
covariance
U300 points
quality
```

---

# 31. LOCAL SUBMAP

Maintain:

```text active_submap
keyframe list
point/distribution representation
submap origin
covariance
```

A submap is removed/rolled forward according to a bounded memory rule.

The local submap should remain small enough for real-time Jetson processing.

---

# 32. GRAPH BACKEND

Graph nodes:

```text X0
X1
X2
...
```

Factors:

```text IMU preintegration
U300 Doppler velocity
U200A altitude
U300 scan matching
ground factor when valid
loop closure when verified
```

Use a fixed-lag graph or local graph for computational stability.

Use robust loss functions.

---

# 33. LOOP CLOSURE

Loop pipeline:

```text candidate detection
      ↓
descriptor similarity
      ↓
temporal/geometric filtering
      ↓
RANSAC or robust registration
      ↓
relative pose
      ↓
consistency test
      ↓
loop factor
```

False loop closures must be rejected.

Never snap the vehicle to a map location based on descriptor similarity alone.

---

# 34. PLACE RECOGNITION

Start with a deterministic radar adaptation of:
- Scan Context
- radar intensity/geometry descriptors

Only introduce learned place recognition after the deterministic implementation passes.

This keeps failure modes understandable.

---

# 35. MAP / ODOM / BODY

Use:

```text
map
 ↓
odom
 ↓
body
```

`odom -> body`:
- smooth local navigation

`map -> odom`:
- slow correction from loop closures

Do not allow loop closure to cause a sudden control-frame jump.

---

# 36. POSITION OUTPUT

Report:

```text position_NED = [N,E,D]
velocity_NED = [VN,VE,VD]
velocity_body = [Vforward,Vright,Vdown]
quaternion
roll,pitch,yaw
```

---

# 37. DISPLACEMENT

Net displacement from start:

```text Δp = p(t) - p(0)
```

Magnitude:

```text ||Δp||
```

Path distance:

```text D_path = Σ ||p_k - p_{k-1}||
```

These are not the same.

Example:

```text move 5m out
move 5m back
```

gives:

```text path distance = 10m
net displacement = 0m
```

---

# 38. DIRECTION OF MOTION

Body-frame:

```text Vforward > 0
    forward

Vforward < 0
    backward

Vright > 0
    right

Vright < 0
    left

Vdown > 0
    down

Vdown < 0
    up
```

World-frame course:

```text course = atan2(VE,VN)
```

only when horizontal speed exceeds a meaningful threshold.

Heading is not the same as direction of motion.

---

# 39. FORWARD MOTION

Expected strongest operating case for the nose U300.

Use:
- Doppler velocity
- ESKF
- forward geometric structure
- scan-to-submap
- loop closure

---

# 40. BACKWARD MOTION

One forward U300 has no direct rear-facing coverage.

However, the estimator can still estimate backward movement for a limited period because:
- IMU propagates continuously
- Doppler may still constrain velocity if the radar rays observe useful static structure
- the local submap contains previously observed geometry
- scan matching can constrain motion against the accumulated scene

But this is not guaranteed in all scenes.

The health system must recognize when reverse motion becomes weakly observable.

For missions requiring long autonomous reverse flight with high confidence, add a rear/omnidirectional perception sensor later.

---

# 41. LEFT/RIGHT LATERAL MOTION

A single U300 can estimate lateral motion if the observed radar directions provide sufficient angular diversity.

The Doppler system must validate:

```text rank(A)=3
condition_number(AᵀWA)
residual
```

If lateral observability is weak:
- covariance increases
- scan-to-submap may still help
- mission controller reduces speed or requests a safer behavior
- external navigation can enter degraded mode

Do not fake lateral velocity.

---

# 42. PURE YAW ROTATION

Pure yaw is important because a radar lever arm can create apparent radar-origin velocity.

Test:

```text yaw
without translation
```

Expected:
- attitude changes
- corrected radar-origin translational component is consistent
- position remains bounded

---

# 43. COMBINED MOTION

The estimator must be tested with:

```text forward + yaw
backward + yaw
left + yaw
right + yaw
diagonal + yaw
roll + translation
pitch + translation
```

This validates the full nonlinear coupling.

---

# 44. RADAR + IMU CONSISTENCY

At every radar update compare:

```text radar velocity
predicted body velocity
SLAM relative velocity if available
```

Log:

```text innovation
Mahalanobis distance
radar covariance
posterior covariance
```

Large persistent disagreement becomes a health event.

---

# 45. RADAR DROPOUT

When U300 is lost:

```text radar invalid
      ↓
no radar update
      ↓
IMU propagation
      ↓
covariance grows
      ↓
health degrades
```

Never:

```text radar invalid → velocity = 0
```

---

# 46. SLAM DROPOUT

When SLAM cannot register:

```text keep ESKF
increase uncertainty
do not freeze pose
```

If uncertainty becomes too large:

```text degraded
```

and configured flight fallback.

---

# 47. DYNAMIC TARGETS

Moving:
- people
- vehicles
- birds
- moving vegetation

must not dominate ego velocity.

Use:
- robust Doppler estimator
- temporal consistency
- spatial consistency
- map consistency
- robust registration

---

# 48. FEATURE-POOR ENVIRONMENTS

Examples:
- open flat field
- blank wall
- repetitive corridor
- severe vegetation clutter
- very sparse radar returns

The system may not have enough information.

Required behavior:

```text uncertainty ↑
quality ↓
health state = DEGRADED
```

This is not a software failure.

It is an observability limitation that must be detected.

---

# 49. ESKF ↔ SLAM COUPLING

Use a two-level design:

## Local tightly coupled estimator

```text IMU
+
U300 Doppler
+
U200A
```

authoritative high-rate state.

## Geometric SLAM layer

```text U300 point cloud
+
submap
+
loop closure
```

provides relative-pose corrections.

The final system is therefore:

**tightly coupled local radar-inertial estimation + graph-based geometric correction.**

Do not average two independently generated trajectories.

---

# 50. OPTIONAL STRONGER TIGHT COUPLING

After the stable ESKF exists, add a fixed-lag factor for U300 geometric registration directly into the local optimization:

```text X_k
 |
 | IMU factor
 |
X_{k+1}
 |
 | U300 Doppler factor
 |
X_{k+1}
 |
 | U300 relative-pose factor
 |
X_{k+1}
```

This allows the optimizer to jointly reason about:
- velocity
- pose
- inertial bias

but it must be added only after the basic ESKF is stable.

---

# 51. WHY NOT START WITH FULL CEREs RIO

The previous stack showed that:
- sparse radar
- incorrect extrinsics
- incorrect frame assumptions
- hard-coded parameters
- weak measurement gates

can cause a large optimization system to produce a mathematically valid but physically wrong result.

Therefore:

1. Build transparent Doppler.
2. Validate ESKF.
3. Validate SLAM.
4. Integrate graph.
5. Only then use a full nonlinear optimizer where beneficial.

Do not hide sensor errors behind optimizer convergence.

---

# 52. U300 SELF-VALIDATION METRICS

For each frame:

```text point_count
inlier_count
velocity
velocity covariance
condition number
residual RMS
innovation
Mahalanobis distance
```

Plot all over time.

---

# 53. SLAM SELF-VALIDATION METRICS

For every registration:

```text source_count
target_count
correspondence_count
fitness
RMSE
translation
rotation
Hessian eigenvalues
condition number
covariance
```

A single scalar "fitness" is insufficient.

---

# 54. ESTIMATOR HEALTH

State machine:

```text BOOT
INITIALIZING
IMU_ONLY
RADAR_GOOD
SLAM_GOOD
LOCALIZATION_GOOD
DEGRADED
FAILSAFE
```

Example `RADAR_GOOD`:

```text IMU fresh
U300 fresh
velocity valid
conditioning acceptable
innovation acceptable
covariance bounded
```

Example `SLAM_GOOD`:

```text registration valid
correspondences sufficient
fitness acceptable
conditioning acceptable
innovation acceptable
```

---

# 55. OUTPUT SAFETY GATE

Before MAVLink:

```text frame_ok
timing_ok
imu_ok
u300_ok or acceptable degraded mode
u200a_ok when required
eskf_ok
covariance_ok
innovation_ok
state_fresh
```

Never publish:

```text valid=true
```

after:
- solver failure
- stale sensor data
- invalid covariance
- NaN/Inf
- rejected required measurement

---

# 56. COVARIANCE RULES

Covariance must be dynamic.

Good measurements:

```text covariance contracts
```

Dropout / weak geometry:

```text covariance grows
```

Loop closure:

```text uncertainty can reduce
```

False confidence is unacceptable.

---

# 57. MAVLink EXTERNAL NAVIGATION

Use ArduPilot external navigation with correctly configured ODOMETRY.

Transmit:
- timestamp
- position
- quaternion
- velocity
- covariance
- estimator quality/state

Coordinate conventions must be explicitly verified against ArduPilot.

Do not feed the estimator:
- ArduPilot LOCAL_POSITION_NED
- ArduPilot EKF output
- its own external estimate

Use a one-way information flow:

```text sensors
   ↓
Jetson RIO/SLAM
   ↓
MAVLink ODOMETRY
   ↓
ArduPilot EKF
   ↓
controllers
```

---

# 58. ANALYZER MUST BE INDEPENDENT

The live supervisor must not be the final judge of its own accuracy.

Create an offline analyzer that independently calculates:

```text path distance
net displacement
ATE
RPE
velocity RMSE
heading error
altitude error
max position excursion
max velocity
```

The analyzer reads logs and does not modify estimator state.

---

# 59. GROUND TRUTH

For validation:

Preferred:
- motion capture indoors
- RTK/PPK outdoors as an independent reference
- surveyed path
- another independent tracking system

GPS can be recorded for reference during development but must not be fed into the GPS-denied estimator.

---

# 60. REQUIRED LOG FORMAT

Every cycle should log:

```text timestamp
imu_age
radar_age
altimeter_age

imu_accel
imu_gyro

u300_points
u300_inliers
u300_velocity_radar
u300_covariance
u300_condition_number
u300_residual_rms

radar_velocity_body
radar_innovation
radar_mahalanobis

u200a_range
u200a_innovation
u200a_quality

position_ned
velocity_ned
velocity_body
quaternion
bias_acc
bias_gyro
state_covariance

slam_valid
slam_translation
slam_rotation
slam_fitness
slam_correspondences
slam_condition_number
slam_covariance

loop_candidate
loop_verified
loop_accepted

health_state
reject_reason
latency
CPU
memory
```

---

# 61. TEST SUITE — DOPPLER

Required:

```text test_stationary
test_forward
test_backward
test_left
test_right
test_up
test_down
test_diagonal
test_outlier
test_multiple_outliers
test_sparse_points
test_rank_deficiency
test_bad_conditioning
test_sign
test_covariance
test_lever_arm
test_rotation
```

Every test must be deterministic.

---

# 62. TEST SUITE — ESKF

Required:

```text test_gravity
test_initial_attitude
test_constant_velocity
test_constant_acceleration
test_yaw
test_pitch
test_roll
test_bias
test_radar_update
test_radar_rejection
test_zupt
test_u200a
test_large_innovation
test_covariance_symmetry
test_covariance_positive_definite
```

---

# 63. TEST SUITE — SLAM

Required:

```text test_identity
test_translation
test_rotation
test_combined_pose
test_sparse_scan
test_outliers
test_dynamic_returns
test_degenerate_geometry
test_submap
test_keyframe
test_loop_closure
test_false_loop_rejection
```

---

# 64. TEST A — 60-SECOND STATIONARY

Place aircraft on a rigid table.

Expected:
- velocity near zero
- bounded position
- stable attitude
- no large covariance anomalies
- no false radar movement
- no false SLAM motion

Do NOT evaluate only:

```text total_distance = 0.00m
```

Measure:
- RMS velocity
- max velocity
- max position excursion
- covariance
- innovation
- accepted/rejected radar updates

Then repeat for 5 minutes.

---

# 65. TEST B — ROTATION ONLY

Rotate:

```text yaw
roll
pitch
```

without intentional translation.

Required:
- attitude tracks rotation
- position remains bounded
- lever-arm compensation works
- no kilometer-scale position drift

---

# 66. TEST C — CONTROLLED HAND TRANSLATION

Physically move known distances:

```text 0.5 m
1 m
2 m
5 m
10 m
```

Forward/backward/left/right.

Compare:

```text estimated displacement
ground truth displacement
```

The estimated order of magnitude must match the real motion.

---

# 67. TEST D — DOPPLER SIGN

Use controlled reflector movement:

```text toward U300
away from U300
```

Verify:
- sign
- magnitude
- repeatability

Correct the U300 sign exactly once.

---

# 68. TEST E — U200A

Measure known vertical distances.

Compare:

```text U200A range
ESKF altitude
ground truth
```

Test:
- level
- small pitch
- small roll

---

# 69. TEST F — SLAM REPLAY

Record a closed route.

Replay offline.

Produce:
- raw Doppler trajectory
- ESKF trajectory
- scan-matching trajectory
- graph-SLAM trajectory
- loop closure correction
- errors

---

# 70. TEST G — RADAR DROPOUT

Inject:
- missing U300 frames
- 1s dropout
- 5s dropout
- 10s dropout

Required:

```text no fake zero
covariance grows
health degrades
```

---

# 71. TEST H — DYNAMIC TARGET

Move reflective objects.

Required:
- robust Doppler
- no large velocity spikes
- no catastrophic SLAM jump

---

# 72. TEST I — FEATURE-POOR

Test:
- open field
- blank wall
- sparse vegetation

Required:
- system recognizes weak observability
- covariance rises
- health degrades
- no false confidence

---

# 73. TEST J — LOOP CLOSURE

Route:

```text start
→ forward
→ turn
→ return
```

Compare:

```text drift before loop closure
drift after loop closure
closure error
```

Loop closure must be geometrically verified.

---

# 74. TEST K — FIGURE EIGHT

Fly/drive/replay:

```text figure 8
```

This tests:
- yaw
- lateral motion
- forward/backward transitions
- repeated scene observations
- loop closure

---

# 75. TEST L — LONG CLOSED LOOP

Build a larger route.

Record:
- RIO without loop closure
- RIO with loop closure

Use independent reference.

Evaluate:
- ATE
- RPE
- closure error
- drift per meter

---

# 76. TEST M — GPS-DENIED REPLAY

Record outdoors with GPS logged only as reference.

Estimator receives:
- IMU
- U300
- U200A

GPS is NOT fed to estimator.

Compare estimated trajectory against GPS/RTK reference offline.

---

# 77. FLIGHT TEST PROGRESSION

Never jump directly to autonomous GPS-denied flight.

Sequence:

```text Level 0
unit tests

Level 1
offline replay

Level 2
stationary bench

Level 3
hand motion

Level 4
ground vehicle / trolley

Level 5
tethered flight

Level 6
manual/assisted flight

Level 7
small autonomous box

Level 8
closed-loop autonomous route

Level 9
larger GPS-denied mission
```

---

# 78. TETHERED FLIGHT

Before autonomous operation:

- aircraft physically constrained
- pilot present
- low altitude
- external-nav output enabled only after bench gates
- no long mission
- monitor live estimator health

Required maneuvers:
- hover
- small forward
- small backward
- left
- right
- yaw
- stop
- land

---

# 79. ASSISTED FLIGHT

Pilot-controlled flight.

Test:

```text hover
forward
backward
left
right
diagonal
yaw + translation
```

Watch:
- estimator covariance
- radar innovation
- SLAM fitness
- position drift

---

# 80. FIRST AUTONOMOUS MISSION

Small box:

```text 2m × 2m
```

or similarly small test envelope appropriate to the aircraft.

No long route.

Then:

```text 5m × 5m
```

Then:

```text 10m × 10m
```

Only expand when quantitative criteria are repeatedly met.

---

# 81. AUTONOMOUS ROUTE

After small tests:

```text start
→ forward
→ right
→ backward
→ left
→ return
```

This tests all four horizontal directions.

Then:
- diagonal patterns
- figure eight
- loop
- return-to-start

---

# 82. ACCURACY METRICS

Required:

### ATE
Absolute Trajectory Error

### RPE
Relative Pose Error

### Velocity RMSE

### Displacement error

```text E_disp = ||Δp_est - Δp_gt||
```

### Scale error

```text E_scale = (D_est-D_gt)/D_gt
```

### Heading error

```text E_yaw
```

### Altitude error

```text E_alt
```

### Closure error

```text E_loop = ||p_final-p_start||
```

---

# 83. "TOP ACCURACY" STRATEGY

Do not chase accuracy by blindly lowering sensor noise values.

Accuracy comes from:

```text correct frames
+
correct extrinsics
+
correct timing
+
robust Doppler
+
truthful covariance
+
good radar filtering
+
motion compensation
+
good submap registration
+
verified loop closure
+
independent ground truth
```

Parameter tuning comes after these are correct.

---

# 84. PERFORMANCE IMPROVEMENT OPTIONS

After the baseline works, evaluate:

```text radar multi-frame aggregation
better Doppler covariance
online temporal calibration
online radar extrinsic refinement
distributional radar registration
adaptive voxelization
radar intensity descriptors
learned radar descriptors
multi-hypothesis loop closures
IMU bias estimation improvements
terrain-plane assistance
map priors / DEM only as an optional constraint
```

Do not add all of these simultaneously.

Measure each improvement independently.

---

# 85. SINGLE-RADAR LIMITATION AND FUTURE EXPANSION

If later missions demand:
- reliable long reverse flight
- strong side-flight observability
- omnidirectional obstacle perception
- aggressive maneuvers in clutter

consider adding:
- rear radar
- side radar
- optical/thermal camera
- LiDAR

But the current software must remain valid with exactly one U300.

Do not architect it so that two radars are silently required.

---

# 86. AUTONOMOUS NAVIGATION LOGIC

Navigation should consume:

```text position
velocity
attitude
covariance
health
```

Before sending a mission command:

```text localization_quality >= threshold
```

During mission:

```text if quality good:
    continue

if quality degraded:
    reduce speed / enter configured safe behavior

if localization invalid:
    invoke configured navigation failsafe
```

The exact fallback is an aircraft safety configuration, not something RIO should guess.

---

# 87. ESTIMATOR WATCHDOG

Monitor:

```text IMU age
U300 age
U200A age
queue size
CPU
memory
temperature
processing latency
NaN
Inf
covariance symmetry
covariance positive definiteness
timestamp discontinuities
optimization time
```

If violated:

```text estimator health = DEGRADED
```

---

# 88. DETERMINISTIC REPLAY

Every recorded run must be replayable.

Repeated replay should produce effectively identical:
- trajectory
- velocity
- health
- estimator decisions

This allows exact bug reproduction.

---

# 89. VERSIONING

Before changing estimator code:

```text git tag:
baseline_bad_old_stack
```

Then:

```text feature/u300-rio-eskf
feature/u300-slam
feature/u300-graph
feature/u300-mavlink
```

Never overwrite the only working copy.

---

# 90. ANTIGRAVITY IMPLEMENTATION RULE

Antigravity must:

1. Read this document before changing estimator code.
2. Freeze the current baseline.
3. Create a branch.
4. Implement one stage at a time.
5. Run unit tests after every stage.
6. Run replay tests after every estimator change.
7. Never silently use hidden defaults.
8. Never hardcode extrinsics.
9. Never hardcode covariance as a universal value.
10. Never convert sensor rejection to fake zero velocity.
11. Never mark failed optimization as valid.
12. Never enable autonomous flight output by default.
13. Log every accepted/rejected measurement.
14. Preserve one authoritative state owner.
15. Stop implementation progression when a lower-level gate fails.

---

# 91. SPECIFIC FILES TO CHANGE FIRST IN THE CURRENT PROJECT

Based on the current `rio_stack copy(3)` generation, the first refactor targets are:

```text
drivers/cube/raw_imu_reader.py
drivers/u300/reader.py
drivers/u300/decoder.py
drivers/u300/adapter.py
drivers/altimeter/reader.py

estimator/doppler.py
estimator/eskf_rio/*
estimator/health.py

config/system.yaml
config/u300_doppler_convention.yaml

src/supervisor.py

tools/analyze_run.py
tests/test_doppler.py
```

Then create:

```text
estimator/radar_extrinsics.py
estimator/time_alignment.py
mapping/submap.py
mapping/radar_registration.py
mapping/loop_closure.py
```

If moving flight-critical code to C++, preserve the Python version as a validated reference until the C++ implementation reproduces it.

---

# 92. SPECIFIC LEGACY CODE RULE

The old C++ HKUST Ceres node should remain available for research/replay, but it must not silently run during the new estimator.

At startup:

```text exactly one authoritative estimator
```

must be identified and printed.

Any stale `rio_node` process must be detected or explicitly killed before a test.

---

# 93. CORRECT LIVE DATA OWNERSHIP

The final design must have:

```text raw sensors
     ↓
drivers
     ↓
preprocessing
     ↓
Doppler / SLAM measurements
     ↓
ESKF / graph
     ↓
one final state
     ↓
health gate
     ↓
MAVLink
```

The analyzer sits OUTSIDE:

```text logs → analyzer
```

and cannot modify live state.

---

# 94. REQUIRED STARTUP REPORT

At startup print:

```text
=== GPS-DENIED RIO ===

U300:
  device:
  frame:
  timestamp:
  extrinsic:
  FOV:
  Doppler sign:

U200A:
  device:
  frame:
  extrinsic:
  range:

IMU:
  source:
  units:
  frame:

ESKF:
  gravity:
  state dimension:

DOPPLER:
  min points:
  condition threshold:
  robust estimator:

SLAM:
  registration:
  submap:
  loop closure:

OUTPUT:
  MAVLink:
  health gate:
  autonomous flight = DISABLED
```

---

# 95. REQUIRED SAFETY DEFAULTS

Default at process launch:

```text autonomous navigation = OFF
MAVLink external-nav output = OFF
```

until the explicit test flag is set.

Example:

```text --enable-external-nav
```

and ideally:

```text --enable-flight-experiment
```

should still require all software health gates.

---

# 96. FAILURE MODE DEFINITIONS

## IMU failure

No valid propagation.

State:

```text FAILSAFE
```

## U300 failure

ESKF can temporarily propagate inertially.

Covariance increases.

State:

```text DEGRADED
```

## U200A failure

Horizontal radar-inertial can continue.

Vertical uncertainty increases.

## SLAM failure

Local Doppler/IMU can continue.

Uncertainty increases.

## Loop closure failure

Continue local navigation.

Do not force correction.

## Timing failure

Reject affected measurements.

## Calibration failure

Do not enable external navigation.

---

# 97. WHY THIS DESIGN IS DIFFERENT FROM THE BROKEN VERSION

The old failure mode was effectively:

```text sparse radar
 ↓
fragile velocity/optimizer
 ↓
wrong velocity
 ↓
IMU integrates wrong state
 ↓
position explodes
```

The new design must be:

```text sparse radar
 ↓
quality/observability check
 ↓
robust Doppler + covariance
 ↓
ESKF innovation test
 ↓
accepted OR rejected
 ↓
SLAM independent geometric cross-check
 ↓
graph correction when valid
 ↓
health
 ↓
only then external navigation
```

---

# 98. NO "0.00 M" CHEATING

Do not change total distance logic to hide estimator drift.

The estimator should report:
- true estimated path
- true estimated displacement
- uncertainty

The analyzer independently computes them.

A display threshold may be used for UI, but it cannot be used as the flight-readiness metric.

---

# 99. FLIGHT-READINESS GATE

The aircraft is NOT ready until all are true:

```text SENSOR CONTRACTS PASS
FRAME CONTRACT PASS
TIMING PASS
EXTRINSIC PASS
DOPPLER TESTS PASS
ESKF TESTS PASS
U200A TESTS PASS
SLAM TESTS PASS
LOOP TESTS PASS
FAULT INJECTION PASS
REPLAY DETERMINISM PASS
MAVLINK ODOMETRY PASS
GROUND-TRUTH TEST PASS
TETHERED FLIGHT PASS
ASSISTED FLIGHT PASS
SMALL AUTONOMOUS BOX PASS
RETURN-TO-START PASS
```

---

# 100. FINAL TARGET ARCHITECTURE

```text
                        U300
                         │
                 ┌───────┴────────┐
                 │                │
                 ▼                ▼
             Doppler            XYZ
                 │                │
                 ▼                ▼
          robust velocity      SLAM frontend
                 │                │
                 │         scan→submap
                 │                │
                 └───────┬────────┘
                         ▼
                    Radar/body
                         │
                         ▼
                    ┌─────────┐
 Cube IMU ─────────►│  ESKF   │◄──────── U200A
                    └────┬────┘
                         │
                    local odom
                         │
                         ▼
                  fixed-lag graph
                         │
                    loop closure
                         │
                         ▼
                  final navigation
                         │
                       health
                         │
                         ▼
                  MAVLink ODOMETRY
                         │
                         ▼
                     ArduPilot
```

---

# 101. FINAL ENGINEERING STATEMENT

The best solution for the current hardware is not:

- pure Doppler
- pure SLAM
- pure IMU
- pure HKUST RIO
- pure BRIO
- pure RIV-SLAM

It is a structured fusion system:

```text IMU
  + 
robust U300 Doppler velocity
  +
U300 scan-to-submap SLAM
  +
U200A altitude
  +
fixed-lag / pose graph
  +
verified loop closure
  +
explicit covariance
  +
health gating
```

That architecture uses complementary information sources rather than asking one weak sensor or one algorithm to solve every problem.

---

# 102. RESEARCH LINKS

HKUST RIO:
https://github.com/HKUST-Aerial-Robotics/RIO

RIV-SLAM:
https://github.com/Wayne-DWA/RIV-SLAM

ETHZ BRIO:
https://github.com/ethz-asl/rio
https://arxiv.org/abs/2408.05764

x-RIO / REVE:
https://github.com/christopherdoer/rio

4D iRIOM:
https://arxiv.org/abs/2303.13962

Temporal radar-IMU calibration:
https://arxiv.org/abs/2503.02509
https://arxiv.org/abs/2603.19958

ArduPilot external navigation:
https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html

Linpowave U300:
https://www.linpowave.com/product/4d-mmwave-radar-u300-for-drone

Linpowave U200A:
https://linpowave.com/product/uav-altitude-measurement

---

# 103. FINAL RULE

**Build and validate the estimator as if it were guilty until proven correct.**

A GPS-denied autonomous system is trustworthy only when:
- its coordinate systems are correct,
- its timing is measured,
- its radar velocity is statistically validated,
- its SLAM geometry is observable,
- its covariance reflects reality,
- its loop closures are verified,
- its faults are detected,
- and the final navigation output has been compared to an independent reference.

The goal is not to make the software always produce a trajectory.

The goal is to make it produce a trajectory only when the evidence supports that trajectory, and to clearly enter degraded/failsafe behavior when that evidence disappears.

# END OF MASTER PLAN
