# RIO + U300 GPS/RF-Denied UAV Navigation — Complete Implementation Blueprint

**Target hardware (fixed):**

- Linpowave U300 4D radar
- Cube Orange+ flight controller
- Jetson Orin Nano
- One dedicated downward/ground-facing altimeter (model-agnostic interface; the current repository contains a U200A-style bridge)

**Purpose:** rebuild the existing `rio_stack` into a tightly coupled, fault-aware, testable GPS-denied local-navigation system, using the HKUST-Aerial-Robotics/RIO implementation as the estimator baseline and keeping all U300-specific modifications inside this copied stack.

**Snapshot reviewed:** 21 September 2026

---

# 0. Executive Decision

## 0.1 The system we are building

The final navigation stack should be:

```text
                    ┌─────────────────────────────┐
                    │          Linpowave U300     │
                    │     XYZ + Doppler @ ~20Hz   │
                    └─────────────┬───────────────┘
                                  │
                                  ▼
                    ┌─────────────────────────────┐
                    │ U300 acquisition / decoder  │
                    │ timestamp preservation       │
                    │ frame validation             │
                    └─────────────┬───────────────┘
                                  │
                                  ▼
                  ┌───────────────────────────────────┐
                  │ Radar measurement preprocessing   │
                  │ - invalid point rejection         │
                  │ - self-return rejection           │
                  │ - range/FOV checks                 │
                  │ - dynamic/static classification   │
                  │ - measurement uncertainty         │
                  └────────────────┬──────────────────┘
                                   │
                      ┌────────────┴────────────┐
                      │                         │
                      ▼                         ▼
               Raw Cube IMU                Altimeter
             gyro + accel + t             range + t + q
                      │                         │
                      └────────────┬────────────┘
                                   ▼
                    ┌─────────────────────────────┐
                    │       HKUST-style RIO       │
                    │                             │
                    │ IMU preintegration          │
                    │ radar tracking               │
                    │ Doppler residual             │
                    │ radar point residual         │
                    │ robust optimization          │
                    │ radar↔IMU extrinsic          │
                    │ uncertainty-aware weighting  │
                    └────────────┬────────────────┘
                                 │
                    local pose + velocity + cov
                                 │
                                 ▼
                    ┌─────────────────────────────┐
                    │     Local radar submaps     │
                    │ scan → submap registration  │
                    │ radar geometry verification │
                    │ optional loop closure       │
                    └────────────┬────────────────┘
                                 │
                                 ▼
                    ┌─────────────────────────────┐
                    │ Navigation Health Manager   │
                    │ observability                │
                    │ residuals                    │
                    │ covariance                   │
                    │ timing                       │
                    │ sensor health                │
                    │ jump / innovation detection  │
                    └────────────┬────────────────┘
                                 │
                                 ▼
                    ┌─────────────────────────────┐
                    │ External ODOMETRY to Cube+  │
                    │ velocity first               │
                    │ pose only after validation  │
                    └────────────┬────────────────┘
                                 │
                                 ▼
                    ┌─────────────────────────────┐
                    │ Cube Orange+ EKF / control  │
                    └─────────────────────────────┘
```

## 0.2 What we are NOT building

Do not continue with this architecture as the flight-critical estimator:

```text
U300
  ↓
Doppler-RANSAC velocity
  ↓
velocity hint
  ↓
Open3D GICP
  ↓
manual axis projection
  ↓
SLAM pose
  ↓
autopilot
```

Do not keep adding thresholds to make that architecture look stable.

The copied HKUST RIO implementation should become the **actual estimator core**, not merely a source of velocity hints.

## 0.3 Primary design principle

The system must distinguish:

```text
MEASUREMENT
  = what the sensor saw

ESTIMATE
  = what the estimator believes

NAVIGATION VALIDITY
  = whether the aircraft is allowed to use that estimate
```

A numerically converged optimizer is not automatically a trustworthy navigation state.

---

# 1. Evidence Base and External References

This blueprint is based on the current copied repository plus the following sources.

## 1.1 HKUST RIO

HKUST RIO describes an optimization-based, point-uncertainty-aware radar-inertial odometry system. The repository contains radar tracking, radar preprocessing, IMU preintegration, Ceres optimization, Doppler residuals, radar point residuals, and radar feature management.

- Repository: https://github.com/HKUST-Aerial-Robotics/RIO
- Paper: https://arxiv.org/abs/2402.16082
- Later RA-L paper title: "Incorporating Point Uncertainty in Radar SLAM"
- The repository is MIT licensed.

Important architectural implication:

> Do not reduce RIO to “Doppler velocity + GICP.”

## 1.2 ETH-ASL BRIO

BRIO is especially relevant because it targets multicopter navigation and combines radar, IMU and barometric information with robust statistical outlier handling.

- Paper: https://arxiv.org/abs/2408.05764
- Repository referenced by the paper: https://github.com/ethz-asl/rio

Reported experiments include real multicopter flights in cities and forests under GNSS-denied conditions.

Use from this work:

- multicopter-oriented estimator design
- robust outlier handling
- barometric aiding
- explicit attention to dynamic objects and radar ghost targets
- flight-oriented validation

Do not copy code blindly; the repository/license and exact hardware differ from the target stack.

## 1.3 4D iRIOM

iRIOM uses a submap concept and combines radar ego-velocity, scan-to-submap matching, IMU and loop closure.

- Paper: https://arxiv.org/abs/2303.13962

Important design lesson:

> Scan-to-submap is preferable to relying only on scan-to-scan registration for sparse radar.

## 1.4 Introspective radar loop closure

Directional 4D radar has limited FOV and sparse/noisy measurements, making loop closure difficult.

- 2024 paper: https://arxiv.org/abs/2404.03940
- 2025 extension: https://arxiv.org/abs/2503.02383

Important lesson:

> Never accept a loop closure from a single descriptor match. Use multiple independent quality checks.

## 1.5 PX4 external navigation

PX4 supports external position/velocity/orientation through MAVLink external-vision/ODOMETRY pathways. Current documentation emphasizes configured fusion selection, correct delay, frame transformations and covariance.

- https://docs.px4.io/main/en/ros/external_position_estimation
- https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf

Important lesson:

> External navigation must provide honest covariance and correct frame/timestamp information.

## 1.6 ArduPilot external navigation

ArduPilot currently documents `ODOMETRY` as the preferred non-GPS external-navigation input and supports position, velocity and yaw source selection through EKF3.

- https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html

Important lesson:

> The bridge must be implemented for the actual autopilot firmware and configured source set; receiving a MAVLink packet is not proof that the autopilot is fusing it.

## 1.7 Linpowave U300

The manufacturer currently lists:

- 76–81 GHz
- 0.2–350 m measuring range
- X/Y/Z + relative velocity output
- about 20 Hz refresh, adjustable
- ±40° azimuth / 120° total azimuth FOV
- ±12° elevation / 24° total elevation FOV
- ±45 m/s velocity detection range
- advertised use for UAVs
- low-light/fog/rain/dust/snow operation claims

Source:

- https://www.linpowave.com/product/4d-mmwave-radar-u300-for-drone

These are sensor specifications, not guarantees of SLAM performance. A 350 m detection range does not imply reliable 6-DOF localization at 350 m.

---

# 2. Current Repository Findings That Must Drive the Redesign

The uploaded `rio_stack` contains:

```text
src/
  altimeter_bridge.py
  doppler_rio.py
  supervisor.py
  mavlink_bridge.py
  filters.py
  slam_node.py
  radar_fanout.py
  nav_node.py
  imu_bridge.py
  gps_logger.py
  visualizer_3d.py

tools/
  analyze_run.py
  analyze_flight.py
  rio_eval.py
  plot_run.py
  ...

tests/
  test_ransac_vz_prior.py
  sitl_test.py

docs/
  implementation_plan.md
  ARCHITECTURE.md
  NAVIGATION.md
  ...
```

The repository already documents multiple previously discovered failures.

The most important ones for the new design are:

1. velocity covariance can be falsely optimistic in a degenerate solve;
2. a condition-number check can fail to see the actual weak direction;
3. coordinate-frame/sign handling has accumulated too many configurable transformations;
4. the old UDP radar path can destroy original frame timestamps;
5. the altimeter path currently treats raw slant range as a consumer-side responsibility;
6. SLAM can accept poses despite weak translational observability;
7. RIO velocity can influence SLAM preprocessing, creating a feedback loop;
8. the current SLAM path uses RIO velocity as a prediction hint instead of a true residual;
9. accumulated radar points are deskewed with simplified constant-velocity/heuristic logic;
10. FC fused attitude/velocity information is mixed into the companion-estimator logic;
11. MAVLink output covariance is not yet a trustworthy measure of the estimator's actual posterior uncertainty.

These are architecture defects, not problems that should be repaired with one more tuning parameter.

---

# 3. Target Software Architecture

## 3.1 New logical modules

Create these logical layers inside the copied `rio_stack`.

```text
rio_stack/
├── rio_core/
│   └── vendor_or_integrated_HKUST_RIO/
│
├── drivers/
│   ├── u300/
│   │   ├── decoder
│   │   ├── frame validator
│   │   └── ROS/MAV adapter
│   │
│   ├── cube/
│   │   ├── raw IMU reader
│   │   └── health/telemetry reader
│   │
│   └── altimeter/
│       ├── decoder
│       ├── quality filter
│       └── timestamp interface
│
├── estimator/
│   ├── rio/
│   ├── calibration/
│   ├── state/
│   ├── uncertainty/
│   └── health/
│
├── mapping/
│   ├── local_submap/
│   ├── registration/
│   ├── loop_closure/
│   └── pose_graph/
│
├── autopilot/
│   ├── mavlink_output/
│   └── ekf_interface/
│
├── tools/
│   ├── replay/
│   ├── evaluation/
│   └── visualization/
│
├── tests/
└── docs/
```

The exact directory naming can be adapted to the current repository, but the boundaries should remain.

---

# 4. Rules for Integrating HKUST RIO

## 4.1 Preserve the estimator mathematics

Initially, do not modify these concepts:

- radar tracker
- radar feature manager
- radar point residual
- Doppler residual
- IMU preintegration
- Ceres optimization
- point uncertainty propagation
- state-window optimization

The adaptation boundary should be:

```text
U300
  ↓
U300 → RIO measurement adapter
  ↓
HKUST RIO
```

not:

```text
HKUST RIO
  ↓
large Python rewrite
  ↓
Open3D
```

## 4.2 Prefer a dedicated C++ RIO process

The original HKUST repository is C++/ROS-oriented and uses Ceres.

On the Jetson, the estimator should be a dedicated native C++ process.

Use:

```text
U300 adapter
      ↓
RIO C++
      ↓
state message
      ↓
mapping/health
      ↓
MAVLink
```

Python may remain for:

- orchestration
- logging
- replay
- analysis
- visualization

but the flight-critical estimator should not depend on the Python interpreter or Open3D.

## 4.3 ROS compatibility

The upstream RIO repository is ROS1-oriented.

Two implementation choices are allowed:

### Option A — ROS1 container / isolated RIO environment

Use the upstream RIO environment as closely as possible and put the U300 adapter at its input.

### Option B — native port of the message boundary

If the Jetson image makes ROS1 deployment impractical, keep the RIO C++ estimator logic and port only the ROS/message boundaries.

Do not port the mathematical estimator to Python.

Document the selected environment in:

```text
docs/BUILD_ENVIRONMENT.md
```

including:

- Ubuntu/L4T version
- CUDA/driver version if used
- compiler
- Eigen
- Ceres
- PCL
- OpenCV
- ROS or non-ROS middleware
- exact Git commit

---

# 5. U300 Adapter Design

## 5.1 Native U300 representation

The current stack receives:

```text
x
y
z
doppler
```

The U300 adapter must preserve:

- original sensor timestamp
- frame number/sequence where available
- point count
- XYZ
- Doppler
- any quality/SNR/target flags exposed by the U300 firmware

Never replace the sensor timestamp with:

```python
time.monotonic()
```

after packet reception if an original timestamp exists.

## 5.2 Required canonical measurement

Internally convert each target to:

```text
RadarMeasurement:
    timestamp
    x_r
    y_r
    z_r
    range
    azimuth
    elevation
    doppler
    quality
    uncertainty
```

Use:

\[
r = \sqrt{x^2+y^2+z^2}
\]

\[
\theta = \operatorname{atan2}(y,x)
\]

\[
\phi = \operatorname{atan2}\left(z,\sqrt{x^2+y^2}\right)
\]

Do not use `asin(y/range/cos(elevation))` where `atan2` is available.

`atan2` avoids unnecessary quadrant ambiguity.

## 5.3 Doppler sign calibration

Do not assume the vendor Doppler sign.

Perform a bench test:

1. Radar stationary.
2. Move a reflector toward radar.
3. Move reflector away.
4. Record Doppler sign.
5. Compare with the RIO convention.
6. Make the conversion exactly once.

Create:

```text
config/u300_doppler_convention.yaml
```

with an explicit field such as:

```yaml
vendor_positive_radial_direction: "TOWARD_RADAR"
rio_positive_direction: "AWAY_FROM_RADAR"
sign: -1
```

The actual value must come from experiment, not assumption.

---

# 6. Coordinate Frames

Define exactly these frames.

```text
W = local world / navigation frame
B = Cube/body frame, FRD
R = U300 radar frame
A = altimeter frame
```

Recommended convention for estimator internals:

```text
B: Forward, Right, Down
R: U300 native documented frame
W: estimator-local navigation frame
```

The autopilot interface can convert to the exact MAVLink NED/FRD convention required by the selected firmware.

## 6.1 One authoritative extrinsic

Create:

```yaml
radar_to_body:
    translation_m: [tx, ty, tz]
    rotation_rpy_deg: [roll, pitch, yaw]
```

or an equivalent rigid transform.

Never apply:

```text
tilt
+
lateral_sign
+
imu_level_points
+
pitch_offset
+
yaw correction
```

independently in multiple files.

There should be one transformation:

\[
T^B_R
\]

and one test suite validating it.

## 6.2 No dynamic radar “leveling”

Do not rotate radar points with the FC attitude before RIO.

Correct architecture:

```text
radar point p_R
       ↓
T_BR
       ↓
body measurement
       ↓
RIO state estimation
```

The estimator itself accounts for the vehicle attitude.

The IMU measurement is not a preprocessing operation on the radar cloud.

---

# 7. Cube Orange+ IMU Interface

## 7.1 Primary input

RIO must receive raw inertial measurements:

```text
accelerometer
gyroscope
timestamp
```

at the highest valid rate available from Cube Orange+.

Do not use:

```text
LOCAL_POSITION_NED
```

as the primary inertial input.

Do not use autopilot-derived velocity to drive RIO.

Do not use autopilot-derived attitude to “repair” radar points.

Autopilot telemetry can be logged for health/reference purposes, but the estimator should remain independent.

## 7.2 Avoid feedback loops

Never allow:

```text
RIO
 ↓
Cube EKF
 ↓
ATTITUDE / LOCAL_POSITION
 ↓
RIO
```

The desired direction is:

```text
raw Cube IMU
       ↓
RIO
       ↓
external ODOMETRY
       ↓
Cube EKF
```

This is essential.

## 7.3 Timestamp

Every IMU sample must retain the closest available measurement timestamp.

Avoid:

```python
time.monotonic()
```

unless the timestamp conversion from the source clock is explicitly documented and synchronized.

---

# 8. Altimeter Architecture

## 8.1 The altimeter is an independent height sensor

The altimeter should output:

```text
timestamp
range_to_surface
quality
validity
```

The estimator should compute vertical height using the full sensor orientation.

For a generic beam-axis vector \(a_A\):

\[
h_{\text{AGL}}
=
r
\left|
e_z^{W\,T}
R_{WA}
a_A
\right|
\]

where:

- \(r\) = measured slant range
- \(a_A\) = altimeter beam direction in its own frame
- \(R_{WA}\) = altimeter orientation in world
- \(e_z^W\) = world vertical unit vector

For a simple downward sensor with known roll/pitch geometry this reduces to the familiar cosine correction.

## 8.2 Do not silently use stale range

Every altitude sample must contain:

```text
t_sample
t_now
age
quality
```

Reject or inflate uncertainty when:

```text
age > configured maximum
```

## 8.3 Do not force an altitude value through the estimator

If the surface is not measurable:

```text
altimeter_valid = false
```

not:

```text
altimeter = last value
```

for unlimited duration.

---

# 9. RIO Estimator

## 9.1 Target state

Use a state containing at minimum:

\[
x =
\left[
p_W,\;
v_W,\;
q_{WB},\;
b_a,\;
b_g
\right]
\]

with radar/IMU extrinsics either:

- fixed from calibrated values, or
- optionally estimated during controlled calibration experiments.

Do not make online extrinsic estimation flight-critical in the first implementation.

## 9.2 IMU preintegration

Use the original RIO preintegration structure.

The prediction interval must be derived from actual sample timestamps:

\[
\Delta t_i = t_{i+1}-t_i
\]

Do not assume a fixed IMU rate.

## 9.3 Radar Doppler residual

For a static target, the measured radial velocity should be consistent with the radar sensor velocity projected onto the target line of sight.

A generic residual is:

\[
r_v
=
u_i^T v_R - v_{r,i}
\]

with sign adapted to the verified U300 convention.

The radar sensor velocity is derived from body/world velocity plus rotational lever-arm contribution:

\[
v_R
=
R_{RB}
\left(
v_B + \omega \times t_{BR}
\right)
\]

with exact sign/order determined by the chosen transform convention.

The important requirement is:

> The Doppler relationship must be a residual in the estimator, not merely a pre-filter that deletes points.

## 9.4 Radar point residual

For a tracked point:

\[
p_{i}^{W}
=
T_{WB_i}
T_{BR}
p_{i}^{R}
\]

The residual compares the estimated world point state with the transformed radar measurement.

The measurement covariance should be propagated from radar coordinates.

## 9.5 Polar uncertainty

The radar naturally measures:

\[
z =
[r,\theta,\phi]
\]

rather than an isotropic Cartesian point.

Use:

\[
p =
\begin{bmatrix}
r\cos\theta\cos\phi\\
r\sin\theta\cos\phi\\
r\sin\phi
\end{bmatrix}
\]

and:

\[
\Sigma_p
=
J
\Sigma_{r\theta\phi}
J^T
\]

where:

\[
J =
\frac{\partial p}{\partial(r,\theta,\phi)}
\]

Start with experimentally calibrated conservative uncertainties.

Do not manufacture per-point precision that the U300 does not actually provide.

---

# 10. U300 Uncertainty Calibration

Because the current U300 point interface does not expose the same uncertainty fields as HKUST's ARS548 data structure, build an experimental measurement model.

## 10.1 Bench calibration

Measure a stationary reflector at:

```text
1 m
2 m
5 m
10 m
20 m
50 m
100 m
```

when feasible.

At each range estimate:

\[
\sigma_r(r)
\]

and angular spread:

\[
\sigma_\theta(r)
\]

\[
\sigma_\phi(r)
\]

Also estimate Doppler noise:

\[
\sigma_v
\]

when the target is stationary.

## 10.2 Incidence angle

Repeat with different reflector angles and radar orientations.

Radar measurement quality can vary strongly with geometry and target characteristics.

## 10.3 Configuration

Create:

```yaml
u300_noise:
    sigma_range_m:
        model: "piecewise"
    sigma_azimuth_rad:
        model: "piecewise"
    sigma_elevation_rad:
        model: "piecewise"
    sigma_doppler_mps:
        model: "constant_or_piecewise"
```

Never commit numbers as “ground truth” until experimentally measured.

---

# 11. Radar Preprocessing

## 11.1 Required filters

Use only physically defensible preprocessing:

1. invalid/NaN/Inf rejection
2. impossible range rejection
3. sensor self-return exclusion zone
4. documented FOV
5. impossible velocity rejection
6. duplicate/near-duplicate suppression where justified
7. dynamic/static classification
8. radar-quality/uncertainty weighting

## 11.2 Do not hard-delete based on a potentially bad RIO velocity

Avoid:

```text
RIO velocity
    ↓
hard Doppler gate
    ↓
delete geometry
```

because it creates a feedback loop.

Instead use:

```text
Doppler disagreement
    ↓
reduce point weight
or mark dynamic
```

and preserve usable geometry whenever possible.

## 11.3 Static target detection

Use:

- Doppler consistency
- temporal persistence
- spatial consistency
- track age
- radar quality

as independent evidence.

A single threshold should not decide that a target is static.

---

# 12. Radar Tracking

Port the HKUST tracking model rather than implementing ad-hoc nearest-neighbor persistence.

The tracker should maintain:

```text
feature_id
first_seen
last_seen
track_age
measurement_history
quality
static_probability
```

Correspondence acceptance should depend on:

- predicted location
- range
- azimuth
- elevation
- uncertainty
- quality/RCS if available
- Doppler/static classification

Do not assume Cartesian Euclidean distance alone is a sufficient radar matching metric.

---

# 13. SLAM / Mapping Back End

RIO and global/local SLAM should be separate layers.

## 13.1 RIO is the local motion estimator

```text
U300 + IMU
      ↓
RIO
      ↓
local pose
```

## 13.2 Mapping is a correction source

```text
current radar scan
      ↓
current local submap
      ↓
registration
      ↓
relative-pose measurement
      ↓
RIO/pose graph
```

GICP/NDT must not be the authority over an underconstrained geometry-only solution.

---

# 14. Submap Design

A submap contains:

```text
20–50 radar frames
```

as a starting experimental range. Tune based on actual U300 density and vehicle speed.

Each point must be stored with:

```text
timestamp
frame_id
radar coordinates
world pose at measurement
measurement covariance
Doppler
static probability
```

This permits later debugging.

## 14.1 Do not accumulate an unbounded global point cloud

Use rolling/local submaps.

Suggested structure:

```text
SUBMAP 0
SUBMAP 1
SUBMAP 2
...
```

Each submap stores:

```text
pose
point cloud
descriptor
quality statistics
time span
```

---

# 15. Motion Compensation / Deskew

This needs to be rebuilt.

Do not use:

```python
xyz -= v_body * dt
```

as the final flight solution.

Use IMU-propagated SE(3) interpolation over the radar accumulation interval.

For every point timestamp \(t_i\):

\[
T_{WB}(t_i)
\]

is interpolated from the IMU state trajectory.

Then:

\[
p_W
=
T_{WB}(t_i)
T_{BR}
p_R(t_i)
\]

and all points can be represented in a common submap frame.

The correct data flow is:

```text
IMU trajectory
       ↓
continuous pose interpolation
       ↓
radar point time
       ↓
SE(3) compensation
```

not:

```text
latest velocity × time
```

This becomes especially important during rapid yaw/pitch/roll.

---

# 16. Local Registration

Start with:

```text
scan/submap
```

not:

```text
scan/scan
```

## 16.1 Candidate methods

Implement one baseline first:

```text
robust point-to-plane / GICP
```

and maintain an experimental path for:

```text
NDT
```

Do not simultaneously implement five registration methods.

## 16.2 Adaptive correspondence radius

The radius should depend on:

- point density
- estimated range
- submap density
- motion prediction uncertainty

Do not use an arbitrary constant across every altitude and terrain type.

## 16.3 Registration result must contain quality data

Return:

```text
T
fitness
RMSE
num_correspondences
inlier_ratio
Hessian
eigenvalues
condition_number
translation_covariance
rotation_covariance
residual_distribution
```

The map layer must never return only:

```text
T
```

---

# 17. Observability and Degeneracy

This is one of the highest-priority parts of the new stack.

Let:

\[
H = J^T WJ
\]

and compute singular values/eigenvalues.

For a 6-DOF registration:

\[
\lambda_1,\ldots,\lambda_6
\]

describe information along different state directions.

A high correspondence count is not enough.

A high fitness score is not enough.

A low RMSE is not enough.

A result is acceptable only when the required degrees of freedom are sufficiently constrained.

## 17.1 Reject underconstrained pose updates

The current stack can accept poses with only about one observable translation axis.

That must be eliminated.

Example:

```text
observable translation:
X = strong
Y = weak
Z = strong
```

Do not publish a trusted unrestricted XYZ correction.

Instead:

```text
constrained directions → update
weak directions → keep prediction / increase covariance
```

## 17.2 Six-DOF observability

For full pose:

```text
Tx
Ty
Tz
Rx
Ry
Rz
```

track each direction independently.

The health system should expose:

```json
{
  "tx": true,
  "ty": true,
  "tz": true,
  "rx": true,
  "ry": true,
  "rz": true
}
```

## 17.3 Rank-aware fusion

When a measurement constrains only a subspace, the estimator should fuse only the observable subspace.

Do not collapse the covariance in directions that the radar geometry did not observe.

---

# 18. Ground Handling

Do not classify the ground as simply “bad.”

Use it as a structured constraint.

## 18.1 Ground returns

Estimate:

```text
ground plane
ground height
plane confidence
surface normal
```

Use the ground to help constrain:

```text
height
roll/pitch
```

when valid.

## 18.2 Off-ground returns

Prefer:

```text
trees
walls
poles
rocks
building edges
terrain discontinuities
```

for translational and yaw observability.

## 18.3 Feature-degenerate ground

If the radar sees mostly a plane:

```text
ground-only
```

increase covariance / reject the corresponding weak directions.

Do not force GICP to invent full 6-DOF motion.

---

# 19. Altitude Fusion

The target stack has:

```text
U300
+
Cube internal barometer
+
external altimeter
```

The Cube's internal barometer is part of the flight controller hardware and is available without adding another external device.

Use these sources differently:

### Altimeter

Best for:

```text
AGL
surface-relative height
```

### Barometer

Useful for:

```text
slow altitude trend
```

### Radar RIO

Provides:

```text
3D relative motion
```

but its vertical component should not be blindly trusted when geometry becomes degenerate.

---

# 20. Navigation State Manager

The navigation state manager should implement explicit states:

```text
INITIALIZING
RIO_ONLY
RIO_GOOD
RIO_DEGRADED
SLAM_GOOD
SLAM_DEGRADED
RECOVERY
INVALID
```

Example:

```text
INITIALIZING
   ↓
enough IMU + radar data
   ↓
RIO_ONLY
   ↓
sufficient radar confidence
   ↓
RIO_GOOD
   ↓
stable submap match
   ↓
SLAM_GOOD
```

If the radar becomes unobservable:

```text
SLAM_GOOD
   ↓
degraded geometry
   ↓
RIO_DEGRADED
   ↓
IMU propagation + conservative covariance
```

If radar disappears:

```text
RADAR LOST
   ↓
IMU propagation
   ↓
uncertainty grows
   ↓
navigation validity eventually false
```

Do not maintain a fake “healthy” state indefinitely.

---

# 21. Recovery Behavior

When radar data becomes unreliable:

```text
DO NOT:
- zero velocity
- snap to last pose
- accept an arbitrary GICP transform
- reset yaw silently
- shrink covariance
```

Instead:

```text
1. Mark external measurement invalid.
2. Continue inertial propagation.
3. Increase uncertainty according to the estimator model.
4. Search for radar recovery.
5. Reinitialize or relocalize only after independent evidence.
```

The flight controller should be informed through its normal external-navigation health/covariance path.

---

# 22. MAVLink Interface

The bridge should be a one-way estimator output layer:

```text
RIO/SLAM
    ↓
MAVLink
    ↓
Cube EKF
```

## 22.1 Stage 1: velocity only

Initially publish:

```text
velocity
velocity covariance
timestamp
quality
```

Do not immediately inject external absolute position.

## 22.2 Stage 2: full external odometry

After SLAM is flight validated:

```text
position
velocity
orientation if validated
covariances
```

Use the exact message/frame/fusion configuration required by the selected autopilot firmware.

## 22.3 Never hard-code covariance

The covariance must come from:

```text
estimator posterior
```

or a defensible conservative uncertainty model.

Not:

```python
cov = [0.05, 0, 0, ...]
```

unless that number is explicitly justified and verified.

---

# 23. PX4 Path

If the Cube runs PX4:

Use the current PX4 external-navigation documentation:

- https://docs.px4.io/main/en/ros/external_position_estimation
- https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf

Verify:

```text
EKF2_EV_CTRL
EKF2_EV_DELAY
EKF2_EV_POS_X
EKF2_EV_POS_Y
EKF2_EV_POS_Z
```

and the exact external velocity/position fusion bits.

PX4's documentation states that covariance information can be supplied through MAVLink ODOMETRY and that external-vision messages are fused according to EKF2 configuration.

Do not copy old parameter values without checking the firmware version actually installed on the Cube.

---

# 24. ArduPilot Path

If the Cube runs ArduPilot:

Use:

- https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html

The current documentation identifies `ODOMETRY` as the preferred external-navigation method.

Configure:

```text
EK3_SRC1_POSXY
EK3_SRC1_VELXY
EK3_SRC1_POSZ
EK3_SRC1_VELZ
EK3_SRC1_YAW
```

according to the actual fusion plan.

Keep a second EKF source or a conservative fallback strategy where practical.

Do not assume a message being accepted by MAVLink means the EKF has actually fused it.

---

# 25. Exact Current-Stack File Disposition

## 25.1 `src/doppler_rio.py`

### Current role

Custom Python RANSAC/WLS/IRLS radar velocity estimator.

### New role

**Experimental/deprecated.**

Keep it temporarily for:

- offline benchmarking
- regression comparison
- debugging
- dataset labeling

Do not make it the primary flight estimator after HKUST RIO is integrated.

Eventually move to:

```text
tools/legacy/
```

or remove from flight startup.

### Preserve useful components

- U300 parser knowledge
- Doppler sign tests
- covariance analysis
- dataset replay
- diagnostic plots

Do not preserve the architecture where Doppler-RANSAC is the main estimator.

---

# 26. `src/slam_node.py`

### Current role

Open3D GICP + accumulation + persistence + manual observability.

### New role

Replace with:

```text
mapping/local_submap_node
```

or equivalent.

Responsibilities:

- receive RIO state
- receive timestamped radar measurements
- construct local submap
- deskew with IMU/RIO trajectory
- perform scan-to-submap registration
- compute observability
- publish registration measurements
- feed corrections to a pose graph or estimator layer

It should not independently invent the flight pose without fusion.

---

# 27. `src/filters.py`

### New policy

Only physically justified filters.

Remove flight-critical logic that says:

```text
if RIO velocity disagrees:
    delete radar point
```

unless the rejection criterion is independently grounded in Doppler/static physics.

Prefer:

```text
quality weight
static probability
uncertainty inflation
```

over blind deletion.

---

# 28. `src/radar_fanout.py`

### New role

It should become the deterministic acquisition layer.

Requirements:

- preserve radar timestamp
- preserve sequence number
- detect dropped packets
- detect reordering
- detect duplicate packets
- record parser errors
- expose queue health
- never silently invent timestamps

Log:

```text
rx_timestamp
sensor_timestamp
sequence
queue_depth
packet_loss
packet_reorder
point_count
```

---

# 29. `src/imu_bridge.py`

### New role

Raw IMU bridge.

Do not use autopilot fused state as a measurement source for RIO.

It can separately expose:

```text
attitude
LOCAL_POSITION_NED
airborne state
```

for:

- diagnostics
- autopilot health
- log comparison

but mark these as:

```text
REFERENCE / TELEMETRY
```

not:

```text
PRIMARY RIO SENSOR
```

unless intentionally justified and documented.

---

# 30. `src/altimeter_bridge.py`

### Keep

- serial decoder
- checksum validation
- spike rejection
- range envelope

### Change

Output:

```text
timestamp
raw_range
corrected/derived AGL
quality
valid
sample_age
```

Do not make all consumers infer whether the range is stale.

---

# 31. `src/mavlink_bridge.py`

### Keep

MAVLink connectivity.

### Change

Split:

```text
estimator_message
```

from:

```text
flight-control command
```

The bridge must not silently arm, disarm, or send autonomous setpoints.

Its job is:

```text
external navigation output
```

until a separately validated control module is enabled.

---

# 32. `src/nav_node.py`

### Current problem

It contains mission-level logic and velocity control while the estimator itself is not yet trustworthy.

### New role

Do not enable autonomous navigation during the estimator bring-up.

It should eventually consume:

```text
NavigationState
```

with:

```text
position
velocity
covariance
health
mode
```

and refuse autonomous execution when:

```text
navigation_valid == false
```

---

# 33. `src/supervisor.py`

Rebuild startup around explicit dependencies:

```text
hardware drivers
    ↓
time synchronization
    ↓
RIO
    ↓
mapping
    ↓
health manager
    ↓
MAVLink bridge
    ↓
optional mission controller
```

The supervisor should refuse autonomous mode unless:

```text
estimator_health == READY
```

and all mandatory sensors report valid.

---

# 34. Data Contract

Create a common internal state message.

Example:

```json
{
  "timestamp": 0.0,
  "frame_id": 1234,

  "pose": {
    "position": [0.0, 0.0, 0.0],
    "quaternion": [0.0, 0.0, 0.0, 1.0]
  },

  "velocity": [0.0, 0.0, 0.0],

  "pose_covariance": [0.0],
  "velocity_covariance": [0.0],

  "rio": {
    "valid": true,
    "n_radar_points": 0,
    "n_static_points": 0,
    "n_tracked_points": 0,
    "doppler_residual_rms": 0.0,
    "imu_residual_rms": 0.0,
    "optimizer_iterations": 0
  },

  "slam": {
    "valid": false,
    "n_correspondences": 0,
    "registration_rmse": 0.0,
    "observable_axes": 0
  },

  "altimeter": {
    "valid": true,
    "agl_m": 0.0,
    "age_s": 0.0
  },

  "health": {
    "radar": true,
    "imu": true,
    "altimeter": true,
    "time_sync": true,
    "observable": true,
    "navigation_valid": true
  }
}
```

This should be a typed C++/ROS message or a carefully specified binary structure rather than an informal collection of UDP packets.

---

# 35. Time Synchronization

This deserves its own subsystem.

The system needs:

```text
U300 clock
Cube clock
Jetson monotonic clock
MAVLink time
```

and a documented conversion path.

## 35.1 Measure

For every frame record:

```text
t_sensor
t_driver_rx
t_decode
t_estimator_input
t_estimator_output
t_mavlink_tx
```

Then calculate:

\[
L_{\text{total}}
=
t_{\text{MAVLink TX}}
-
t_{\text{sensor}}
\]

Track:

```text
mean latency
median latency
P95
P99
jitter
```

## 35.2 No burst timestamping

Never let five radar frames share the same receive timestamp because they arrived in a burst.

That destroys motion compensation.

---

# 36. Logging Specification

Every flight log should record:

## Sensor

```text
radar timestamp
radar sequence
point count
IMU timestamp
IMU sample rate
altimeter timestamp
altimeter quality
```

## RIO

```text
valid
reason
number of radar tracks
number of static tracks
Doppler residual RMS
point residual RMS
optimizer status
iteration count
velocity
velocity covariance
pose
pose covariance
biases
```

## SLAM

```text
keyframe id
number of points
number of static points
correspondences
fitness
RMSE
Hessian eigenvalues
observability
registration transform
registration covariance
```

## Health

```text
radar health
IMU health
altimeter health
clock health
packet loss
queue depth
CPU
RAM
temperature
process latency
```

## Autopilot

```text
EKF status
external-nav fusion status
innovation
failsafe state
mode
armed state
```

---

# 37. No-Radar / Radar-Dropout Behavior

The estimator must explicitly model radar dropouts.

A valid sequence is:

```text
radar valid
   ↓
radar lost
   ↓
IMU propagation
   ↓
covariance increases
   ↓
radar reacquired
   ↓
innovation check
   ↓
resume fusion
```

Never:

```text
radar lost
   ↓
velocity = 0
   ↓
position held fixed
```

unless the vehicle is independently known to be stationary.

---

# 38. Static/Hover Detection

Do not determine stationary state purely from radar.

A robust static hypothesis can use:

```text
gyro magnitude
accelerometer magnitude
radar Doppler distribution
radar track persistence
altimeter rate
vehicle armed/landed state
```

During takeoff/landing, disable assumptions that imply the vehicle is stationary merely because static radar targets dominate.

---

# 39. Known Failure Conditions

The final health manager must explicitly recognize:

### Radar-dominant failures

```text
too few useful points
feature starvation
mostly flat ground
dynamic-dominated scene
multipath/ghost domination
self-reflections
weak angular diversity
large unexplained Doppler disagreement
```

### IMU-dominant failures

```text
sample drop
timestamp discontinuity
gyro saturation
accelerometer saturation
large bias innovation
```

### Mapping failures

```text
too few correspondences
rank-deficient Hessian
ambiguous registration
large residual
large transform jump
submap mismatch
repeated geometry
```

### Navigation failures

```text
external-nav stale
covariance too large
innovation too large
clock delay unstable
sensor unhealthy
```

---

# 40. Environment / Geometry Risk Model

The U300's 120° azimuth and 24° elevation FOV are useful, but a wide horizontal FOV does not guarantee full 6-DOF observability.

## High-confidence environments for development

```text
indoor rooms
warehouse
urban streets with static structure
rocky terrain
structured forest
buildings/walls
```

## Difficult environments

```text
flat field
smooth sand
water
featureless roads
snow-covered uniform terrain
very high altitude over uniform surfaces
dynamic-only scene
```

The system must report:

```text
GEOMETRY QUALITY = LOW
```

rather than pretending the environment is fully observable.

---

# 41. High-Altitude Consideration

Do not equate:

```text
U300 detection range = 350 m
```

with:

```text
SLAM usable at 350 m altitude
```

At higher altitude the radar may return a large quantity of ground information without sufficient independent 3D structure.

The problem is information geometry, not just detection range.

For every high-altitude test record:

```text
AGL
radar point density
vertical spread
azimuth spread
static point count
Hessian eigenvalues
SLAM observability
```

---

# 42. Single-Radar Limitation

Design the software so it can eventually support:

```text
Radar A
Radar B
```

but do not require the second radar for the first implementation.

The single U300 can be made into a useful local estimator if the scene supplies adequate static geometry, but a second viewpoint is a natural future upgrade for reducing geometric degeneracy.

Do not compensate for a lack of information with increasingly aggressive software heuristics.

---

# 43. Global GPS-Denied Positioning

Local RIO/SLAM gives a local trajectory.

It does not automatically provide globally georeferenced coordinates.

For true mission-scale GPS-denied navigation, add a later layer:

```text
local RIO/SLAM
       +
terrain/DEM matching
or
map/place recognition
       ↓
global correction
```

The global localization layer should be introduced only after local odometry is independently validated.

---

# 44. Terrain Matching Roadmap

Since the target system already considers DEM/SRTM-style terrain matching, the future architecture is:

```text
RIO local pose
      ↓
local terrain estimate
      ↓
height/elevation signature
      ↓
DEM candidate search
      ↓
geometric verification
      ↓
global pose correction
```

The correction should be a measurement/factor.

Do not directly overwrite RIO with the DEM answer.

---

# 45. Loop Closure Roadmap

Stage 1:

```text
no loop closure
```

Stage 2:

```text
radar descriptor
+
candidate retrieval
```

Stage 3:

```text
descriptor
+
geometric verification
+
Doppler consistency
+
altitude consistency
+
pose-graph factor
```

Stage 4:

```text
introspective confidence
+
false-loop rejection
```

A loop closure must never be able to create an unverified global jump.

---

# 46. Registration Algorithm Roadmap

## Version 1

Robust GICP / point-to-plane registration.

Goal:

```text
correctness
```

## Version 2

Adaptive NDT / distribution registration.

Goal:

```text
robustness to sparse radar geometry
```

## Version 3

Research upgrade:

```text
learned radar correspondence
```

or other modern methods.

Only introduce after Version 1 produces a trustworthy baseline.

---

# 47. Compute Plan for Jetson Orin Nano

Keep the flight-critical path bounded.

Target process structure:

```text
Thread 1: radar I/O
Thread 2: IMU I/O
Thread 3: RIO estimator
Thread 4: local mapping
Thread 5: health/logging
Thread 6: MAVLink
```

The mapping thread must never block the estimator thread indefinitely.

Set:

```text
queue maximum
processing deadlines
dropped-frame counters
watchdogs
```

A stale registration should be dropped rather than queued for seconds.

---

# 48. Watchdogs

Each critical process needs:

```text
heartbeat
last_input_time
last_output_time
processing_latency
```

The supervisor should detect:

```text
RIO stalled
SLAM stalled
MAVLink stalled
radar silent
IMU silent
altimeter silent
```

and transition to a degraded state.

---

# 49. Safety Boundary

The estimator itself must not command the aircraft directly during development.

Required chain:

```text
Estimator
  ↓
external navigation measurement
  ↓
autopilot EKF
  ↓
validated flight controller
```

Mission/control logic should be a separate process.

---

# 50. Phased Implementation Plan

# Phase 0 — Freeze and Baseline

Do not fly autonomously.

Tasks:

- freeze current working branch;
- tag baseline;
- save current logs;
- preserve `analyze_run.py` outputs;
- create reproducibility manifest;
- record Jetson OS;
- record Cube firmware;
- record U300 firmware;
- record exact Python/compiler versions.

Deliverable:

```text
docs/BASELINE.md
```

Gate:

```text baseline build reproducible
baseline logs replayable
```

---

# Phase 1 — U300 Sensor Correctness

Tasks:

1. Validate XYZ frame.
2. Validate Doppler sign.
3. Validate range.
4. Validate sequence numbering.
5. Validate sensor timestamps.
6. Validate FOV.
7. Measure packet loss.
8. Measure point rate.
9. Characterize self-reflections.
10. Characterize static reflector behavior.

Bench tests:

```text stationary reflector
approaching reflector
receding reflector
lateral reflector
multiple reflectors
tilted radar
```

Gate:

```text no unexplained sign inversion
no timestamp corruption
no silent packet loss
```

---

# Phase 2 — Cube IMU Correctness

Tasks:

- obtain raw gyro/accelerometer;
- establish timestamp mapping;
- measure actual sample rate;
- test saturation;
- calibrate bias;
- verify FRD/body convention;
- verify sign of gravity;
- verify stationary acceleration vector.

Gate:

```text stationary:
gyro ≈ 0
|acc| ≈ g
timestamps monotonic
```

---

# Phase 3 — Radar↔IMU Extrinsic Calibration

Measure:

\[
T^B_R
\]

Use:

- mechanical measurement
- inclinometer
- controlled motion
- radar/IMU consistency experiments

Never tune radar mount angles separately in three files.

Gate:

```text transform reproduced exactly by all modules
```

---

# Phase 4 — Integrate HKUST RIO

Tasks:

- bring upstream C++ estimator into copied stack;
- build isolated;
- run official sample dataset;
- validate official sequence;
- validate expected output;
- only then connect U300 adapter.

Gate:

```text official dataset reproduces expected qualitative behavior
```

---

# Phase 5 — U300→RIO Adapter

Tasks:

- map XYZ→polar;
- map Doppler;
- provide uncertainty model;
- preserve timestamps;
- provide radar frame;
- disable unsupported fields initially;
- document every conversion.

Gate:

```text U300 bag/replay feeds RIO deterministically
```

---

# Phase 6 — U300 Bench RIO

Tests:

```text stationary
yaw rotation
pitch rotation
roll rotation
straight translation
combined translation/rotation
```

Use an independent reference whenever possible.

Gate targets:

- no frame/sign jumps;
- no unexplained velocity spikes;
- covariance grows during weak geometry;
- invalid frames are truly invalid;
- no artificial zero-velocity claims.

---

# Phase 7 — Hand-Carried / Ground Rig

Use:

```text walking
cart
vehicle
```

Do not connect estimator output to flight controls.

Evaluate:

```text velocity error
position drift
availability
latency
covariance consistency
observability
```

Gate:

```text repeatable across multiple datasets
```

---

# Phase 8 — Local Radar Submap

Add:

```text scan accumulation
IMU-based deskew
scan-to-submap registration
registration covariance
observability
```

Do not add loop closure yet.

Gate:

```text stable local trajectory
no 10+ metre pose jumps
no accepted rank-deficient registration
```

---

# Phase 9 — Altimeter Fusion

Add:

```text AGL
altimeter quality
barometer trend
```

Validate:

```text level
tilted
climb
descent
uneven terrain
```

Gate:

```text correct AGL under vehicle attitude changes
stale readings rejected
```

---

# Phase 10 — MAVLink Velocity Fusion

First connect:

```text RIO velocity only
```

with:

```text conservative covariance
```

No mission control.

Test:

```text receive
fuse
reject
recover
```

Gate:

```text autopilot confirms actual EKF fusion
```

---

# Phase 11 — Passive Flight

Aircraft flying with conventional safety/navigation reference.

Radar estimator has:

```text zero flight-control authority
```

Record:

```text GPS/RTK reference
Cube EKF
RIO
SLAM
altimeter
timestamps
```

Goal:

```text compare
```

not:

```text control
```

---

# Phase 12 — Tethered Flight

Enable external navigation in a controlled manner.

Start with:

```text velocity fusion
```

then:

```text position
```

only after verification.

Gate:

```text no estimator jumps
no EKF innovation explosion
no unexpected mode transitions
```

---

# Phase 13 — Low-Altitude Controlled Flight

Test:

```text hover
forward
backward
left
right
climb
descend
yaw
figure-eight
```

Use structured terrain.

---

# Phase 14 — Geometry-Degraded Tests

Deliberately test:

```text flat field
uniform ground
sparse terrain
water-like surfaces where appropriate
dynamic clutter
```

The success criterion is not “never loses radar.”

The criterion is:

```text detects degradation
does not publish false confidence
recovers correctly
```

---

# Phase 15 — Higher Altitude

Increase altitude gradually.

Record:

```text point density
geometry spread
observability
velocity covariance
SLAM covariance
```

Find the actual operating envelope of the U300/RIO system instead of assuming it from the radar's maximum detection range.

---

# Phase 16 — Long-Duration Drift Validation

Run:

```text 100 m
250 m
500 m
1 km+
```

where the environment permits safe testing.

Calculate:

\[
\text{drift per distance}
=
\frac{\text{translation error}}{\text{path length}}
\]

Do not compare partial RIO coverage with the total reference trajectory.

Always use identical time support.

---

# Phase 17 — Loop Closure

Only after local odometry is validated:

```text descriptor
+
candidate
+
geometry
+
multi-condition verification
+
pose graph
```

---

# Phase 18 — Terrain / Global Localization

Only after local SLAM is stable:

```text DEM
+
local RIO/SLAM
+
altitude
```

Build global correction.

---

# 51. Quantitative Validation Metrics

Every test must report:

## RIO

\[
RMSE_v
\]

\[
MAE_v
\]

\[
RMS_{\text{Doppler}}
\]

\[
\text{validity rate}
\]

\[
\text{coverage}
\]

\[
\sigma_v
\]

## SLAM

\[
ATE
\]

\[
RPE
\]

\[
\text{drift/distance}
\]

\[
\text{registration RMSE}
\]

\[
N_{\text{correspondences}}
\]

\[
\lambda_{\min}(H)
\]

## Timing

\[
L_{\text{radar}}
\]

\[
L_{\text{RIO}}
\]

\[
L_{\text{SLAM}}
\]

\[
L_{\text{MAVLink}}
\]

and P95/P99.

---

# 52. Covariance Validation

This is mandatory.

If:

\[
e = x_{\text{estimate}} - x_{\text{reference}}
\]

then normalized error should approximately satisfy:

\[
e^T\Sigma^{-1}e
\]

with a statistically reasonable distribution.

Do not allow:

```text
actual error ≈ 0.5 m/s
claimed sigma ≈ 0.007 m/s
```

This is a dangerous mismatch.

The goal is not the smallest covariance.

The goal is:

> covariance that honestly describes uncertainty.

---

# 53. Regression Testing

Every code change touching:

```text
frames
timestamps
doppler
RIO
SLAM
covariance
MAVLink
```

must run:

```text unit tests
synthetic estimator tests
replay tests
previous-flight regression tests
```

Maintain a fixed corpus:

```text datasets/
  stationary/
  walk/
  vehicle/
  structured/
  flat/
  dynamic/
  high_altitude/
```

---

# 54. Synthetic Tests

At minimum test:

## Geometry

```text full 3D
planar
line
single point
sparse
```

## Motion

```text translation only
rotation only
combined
high angular velocity
acceleration
```

## Sensor failures

```text missing radar frame
missing IMU sample
timestamp jump
packet reorder
packet duplication
wrong Doppler sign
extrinsic error
stale altimeter
```

## Estimator responses

Expected behavior:

```text failure → uncertainty increase / invalid state
```

not:

```text failure → confident garbage
```

---

# 55. Specific Tests for the Current Bugs

Create regression tests for every previously observed defect.

## Test R1

Weak radar geometry must not produce falsely tiny velocity covariance.

## Test R2

Condition/observability metric must include all actual solved dimensions.

## Test R3

Lateral sign/vertical axis must never change the physical meaning of the Z axis.

## Test R4

A physical right-side reflector must map to positive body Y under the documented convention.

## Test R5

Two radar frames with different sensor timestamps must not become timestamp-identical after UDP transport.

## Test R6

Tilted altimeter must produce correct vertical height.

## Test R7

Robust weighting must produce covariance consistent with the actual residual distribution.

## Test R8

Gimbal/frame edge cases must not silently create a discontinuity.

---

# 56. Analyzer Redesign

The analyzer must never make the estimator look better or worse because of bookkeeping.

For every comparison:

```text same time interval
same coordinate frame
same reference source
same valid-data mask
```

## 56.1 Reference hierarchy

Best:

```text RTK/independent external reference
```

Second:

```text high-quality external motion reference
```

Weakest:

```text autopilot EKF/GPS-derived state
```

Do not label autopilot EKF output as absolute truth.

---

# 57. RIO Distance Evaluation

Never do:

```text
RIO integrated over valid segments
vs
full-flight GPS distance
```

Instead:

```text identify RIO-valid intervals
        ↓
integrate RIO over those intervals
        ↓
integrate independent reference over exact same intervals
        ↓
compare
```

---

# 58. SLAM Evaluation

Report separately:

```text absolute trajectory error
relative pose error
shape/alignment error
drift
```

Do not confuse Kabsch-aligned shape error with true absolute navigation error.

---

# 59. Navigation Quality State

Recommended output:

```text
quality_level:
    0 = INVALID
    1 = CRITICAL
    2 = DEGRADED
    3 = GOOD
    4 = EXCELLENT
```

Do not convert this into a single “confidence score” without exposing the reasons.

Example:

```json
{
  "quality_level": 2,
  "reasons": [
    "low_horizontal_observability",
    "radar_point_density_low"
  ]
}
```

---

# 60. Hard Navigation Gates

At minimum:

```text
radar healthy
AND
IMU healthy
AND
time synchronized
AND
state not stale
AND
covariance finite
AND
innovation reasonable
AND
required DOF observable
```

Otherwise:

```text
external_nav_valid = false
```

---

# 61. Soft Degradation

Some failures should reduce trust rather than immediately invalidate:

```text slightly fewer points
moderate dynamic clutter
moderate increase in residual
temporary altitude dropout
```

The response should be:

```text uncertainty inflation
```

not:

```text fake certainty
```

---

# 62. Final Autonomous Navigation Architecture

The mature system should look like:

```text
                 ┌─────────────┐
                 │   U300      │
                 └──────┬──────┘
                        │
                        ▼
              ┌──────────────────┐
              │ Radar Adapter     │
              │ + quality         │
              └────────┬─────────┘
                       │
                       │
┌─────────────┐        │        ┌─────────────┐
│ Cube Raw IMU├────────┼───────►│    RIO      │
└─────────────┘        │        └──────┬──────┘
                       │               │
┌─────────────┐        │               │
│ Altimeter   ├────────┘               │
└─────────────┘                        │
                                       ▼
                              ┌────────────────┐
                              │ Local Submaps  │
                              │ Registration   │
                              └───────┬────────┘
                                      │
                                      ▼
                              ┌────────────────┐
                              │ Loop Closure   │
                              │ Pose Graph     │
                              └───────┬────────┘
                                      │
                                      ▼
                              ┌────────────────┐
                              │ Health Manager │
                              └───────┬────────┘
                                      │
                                      ▼
                              ┌────────────────┐
                              │ MAVLink ODOM   │
                              └───────┬────────┘
                                      │
                                      ▼
                              ┌────────────────┐
                              │ Cube EKF       │
                              └───────┬────────┘
                                      │
                                      ▼
                                   Control
```

---

# 63. What Success Means

The project should not be declared successful because:

```text
trajectory looks smooth
```

or:

```text
GICP fitness = 1
```

Success requires:

### Sensor correctness

```text
frames correct
timestamps correct
Doppler sign correct
extrinsics verified
```

### RIO correctness

```text
Doppler residual physically correct
IMU integration correct
covariance honest
degeneracy visible
```

### SLAM correctness

```text
submap registration
observable DOF
no false pose jumps
```

### Navigation correctness

```text
autopilot actually fuses data
stale/invalid measurements rejected
fallback behavior works
```

### Repeatability

```text
multiple flights
multiple environments
multiple motion profiles
```

---

# 64. Operating Envelope Must Be Empirically Measured

Create a table after testing:

| Environment | Altitude | Speed | Radar coverage | RIO validity | SLAM validity | Drift | Status |
|---|---:|---:|---:|---:|---:|---:|---|
| indoor | | | | | | | |
| forest | | | | | | | |
| urban | | | | | | | |
| rocky terrain | | | | | | | |
| flat field | | | | | | | |
| high altitude | | | | | | | |

Do not fill values from manufacturer specifications.

These become properties of **your complete system**.

---

# 65. Development Priorities

## P0 — Must be fixed before autonomous flight

```text
1. Replace custom flight-critical Python RIO with actual HKUST RIO architecture.
2. Preserve timestamps end-to-end.
3. Correct all coordinate frames.
4. Calibrate radar→body extrinsic.
5. Use raw Cube IMU, not FC fused position/velocity as RIO input.
6. Correct altimeter geometry/staleness.
7. Implement truthful covariance.
8. Implement observability-aware acceptance.
9. Remove hard RIO→SLAM geometry deletion.
10. Implement IMU-based SE(3) deskew.
11. Implement explicit health states.
12. Keep MAVLink output conservative during validation.
```

## P1 — Required for robust operation

```text
13. Local submaps.
14. Scan-to-submap registration.
15. Registration covariance.
16. Robust dynamic-object handling.
17. Replay system.
18. Failure injection.
19. Multi-flight regression corpus.
20. MAVLink fusion verification.
```

## P2 — Long-duration / mission-scale upgrades

```text
21. Loop closure.
22. Pose graph.
23. Terrain/DEM localization.
24. Multi-radar support.
25. Learned radar correspondence.
26. Advanced NDT/SDF methods.
```

---

# 66. Recommended First Implementation Commit Structure

Do not make one giant unreviewable commit.

Suggested commits:

```text
01-freeze-baseline
02-u300-time-and-frame-contract
03-cube-raw-imu-contract
04-altimeter-contract
05-integrate-hkust-rio
06-u300-rio-adapter
07-rio-replay-harness
08-rio-observability-health
09-local-submap
10-imu-deskew
11-scan-to-submap-registration
12-registration-health
13-mavlink-velocity-output
14-mavlink-pose-output
15-loop-closure
16-terrain-global-localization
```

Each commit should have:

```text
build result
unit-test result
replay result
known limitations
```

---

# 67. Repository Rules

Add:

```text
docs/
  SYSTEM_ARCHITECTURE.md
  FRAME_CONVENTIONS.md
  TIME_SYNC.md
  SENSOR_CONTRACTS.md
  RIO_INTEGRATION.md
  SLAM_INTEGRATION.md
  NAVIGATION_HEALTH.md
  MAVLINK_INTERFACE.md
  TEST_PLAN.md
  FLIGHT_TEST_PLAN.md
  OPERATING_ENVELOPE.md
```

Create one configuration source of truth:

```text
config/system.yaml
```

Do not duplicate:

```text
tilt
sign
lever arm
frame convention
threshold
```

across Python command lines.

---

# 68. Configuration Philosophy

Every flight-critical parameter must say:

```text
what it means
units
coordinate frame
source
how calibrated
allowed range
reason
```

Example:

```yaml
radar:
  frame: U300_NATIVE
  max_range_m: 200.0
  doppler_sign: -1

extrinsics:
  radar_to_body:
    translation_m: [0.0, 0.0, 0.0]
    rotation_rpy_deg: [0.0, 0.0, 0.0]

estimator:
  max_state_latency_ms: 100
  robust_loss: HUBER

health:
  covariance_consistency_ratio_max: 2.0
  max_measurement_age_ms: 100
```

Actual values must be calibrated/tested.

---

# 69. Important Non-Goals for the First Release

Do not attempt to solve all of these simultaneously:

```text
perfect global localization
perfect loop closure
learned correspondences
multi-radar fusion
autonomous route planning
dynamic obstacle avoidance
```

The first milestone is:

> **A trustworthy local radar-inertial state with honest uncertainty and safe degradation.**

Everything else depends on this.

---

# 70. First Flight-Capable Milestone

The first meaningful flight milestone is:

```text
U300
+
Cube raw IMU
+
RIO
+
altimeter
+
MAVLink velocity/odometry
```

with:

```text
no GICP authority
no loop closure
no DEM authority
no automatic global reset
```

The estimator must be able to say:

```text
GOOD
```

or:

```text
DEGRADED
```

or:

```text
INVALID
```

and the Cube must react according to a deliberately tested fusion configuration.

---

# 71. Final Engineering Position

The target system should be treated as a **fault-aware state-estimation system**, not as a point-cloud matching application.

The main mathematical structure is:

\[
\boxed{
p,v,q,b_a,b_g
\leftarrow
\{
\text{IMU},
\text{Doppler},
\text{radar geometry},
\text{altimeter},
\text{submap constraints}
\}
}
\]

through a coupled estimator.

The map layer then provides additional constraints:

\[
z_{\text{submap}}
=
h(x)
+
n
\]

and the global layer later provides:

\[
z_{\text{global}}
=
h_{\text{global}}(x)
+
n.
\]

Every measurement must carry:

```text
timestamp
frame
measurement
uncertainty
validity
```

Every estimate must carry:

```text
state
covariance
quality
age
```

Every failure must result in:

```text
degradation
```

rather than:

```text
fabricated certainty.
```

---

# 72. Concrete End State

For the hardware fixed by this project:

```text
┌─────────────────────────────────────────────┐
│               UAV HARDWARE                  │
│                                             │
│  Linpowave U300                             │
│       │                                     │
│       ├──────────────┐                      │
│       │              │                      │
│  Cube Orange+     Altimeter                 │
│       │              │                      │
│       └──────┬───────┘                      │
│              │                              │
│      Jetson Orin Nano                       │
│              │                              │
│      ┌───────▼────────┐                     │
│      │ U300 Adapter   │                     │
│      └───────┬────────┘                     │
│              ▼                              │
│      ┌─────────────────┐                    │
│      │ HKUST RIO       │                    │
│      └────────┬────────┘                    │
│               ▼                             │
│      ┌─────────────────┐                    │
│      │ Local Submaps   │                    │
│      └────────┬────────┘                    │
│               ▼                             │
│      ┌─────────────────┐                    │
│      │ Health Manager  │                    │
│      └────────┬────────┘                    │
│               ▼                             │
│      ┌─────────────────┐                    │
│      │ MAVLink ODOM    │                    │
│      └────────┬────────┘                    │
│               ▼                             │
│      ┌─────────────────┐                    │
│      │ Cube EKF        │                    │
│      └─────────────────┘                    │
└─────────────────────────────────────────────┘
```

The first target is **reliable local GPS-denied navigation**.

The second target is **drift-limited radar SLAM**.

The third target is **global GPS-denied localization through terrain/map correction**.

---

# 73. Definition of Done

The stack is not ready for autonomous GPS-denied navigation until all of the following are true:

- [ ] U300 coordinate system experimentally verified.
- [ ] Doppler sign experimentally verified.
- [ ] U300 timestamps preserved end-to-end.
- [ ] Radar packet loss measured.
- [ ] Cube raw IMU timestamp/rate verified.
- [ ] Radar→body extrinsic calibrated and unit-tested.
- [ ] Altimeter beam orientation calibrated.
- [ ] Altimeter slant-range correction verified.
- [ ] HKUST RIO sample dataset runs successfully.
- [ ] U300 adapter feeds RIO deterministically.
- [ ] RIO Doppler residual verified.
- [ ] RIO point residual verified.
- [ ] RIO covariance experimentally validated.
- [ ] RIO weak-geometry behavior verified.
- [ ] no FC fused-state feedback into RIO.
- [ ] constant-velocity deskew removed from final flight path.
- [ ] IMU-based SE(3) deskew working.
- [ ] submap registration working.
- [ ] registration covariance working.
- [ ] rank-deficient registration cannot be published as healthy.
- [ ] dynamic radar targets handled robustly.
- [ ] radar dropout behavior validated.
- [ ] altimeter dropout behavior validated.
- [ ] Jetson latency measured.
- [ ] MAVLink external-nav messages confirmed to be fused by the actual firmware.
- [ ] velocity-only fusion validated before pose fusion.
- [ ] passive flight validated.
- [ ] tethered flight validated.
- [ ] low-altitude controlled flight validated.
- [ ] degraded-geometry flights validated.
- [ ] multi-flight regression completed.
- [ ] operating envelope documented.

---

# 74. Final Recommendation

The copied RIO stack should now be treated as a **new estimator platform**, not as another patch cycle for the old Python/GICP stack.

The implementation order should be:

```text
U300 correctness
      ↓
Cube raw IMU correctness
      ↓
extrinsic calibration
      ↓
HKUST RIO
      ↓
U300 adapter
      ↓
RIO replay
      ↓
RIO validation
      ↓
IMU deskew
      ↓
local radar submaps
      ↓
scan-to-submap registration
      ↓
health/observability
      ↓
MAVLink velocity
      ↓
MAVLink pose
      ↓
controlled flight
      ↓
loop closure
      ↓
terrain/global localization
```

The single most important architectural rule is:

> **Do not use a downstream heuristic to compensate for a missing constraint in the upstream estimator. Add the constraint where the measurement belongs.**

For this project that means:

```text
Doppler → Doppler factor
IMU → IMU factor
Radar geometry → radar point factor
Uncertainty → measurement weighting
Altimeter → height constraint
Submap registration → mapping factor
Global terrain/map → global correction factor
Health → fusion gating
```

That is the path from the current experimental codebase toward a system that can be rigorously tested as a GPS-denied navigation system.
