# U300 + Cube Orange+ + Jetson Orin Nano — Proper Radar/IMU Fusion Redesign

Date: 2026-09-22

## Executive conclusion

The current 60-second stationary run demonstrates that the present implementation is not fusing the U300 radar in a stable, trustworthy way.

The observed log starts with only 3 radar points and 3 static points, then produces a non-zero velocity of approximately:

```text
[-0.71, +0.33, -0.16] m/s
```

After that, static-point availability collapses while the estimated velocity and position diverge continuously.

This is visible directly in the supplied run log: the estimator begins with 3 static points and a large non-zero velocity, later repeatedly reports zero static points, and ultimately reaches approximately:

```text
position = [-1069.34, 1457.91, -382.27] m
```

after a stationary test.

### The most important finding

This is not primarily a problem of "IMU being too strong."

The actual failure chain is closer to:

```text
very sparse U300 frame
        ↓
3-point Doppler solve
        ↓
3 equations trying to estimate 3D velocity
        ↓
noise / geometry conditioning creates false velocity
        ↓
predicted velocity becomes wrong
        ↓
static-point test becomes inconsistent
        ↓
static points disappear
        ↓
Doppler update disappears
        ↓
IMU prediction carries the state
        ↓
position/velocity diverges
```

The current native HKUST RIO Ceres implementation is therefore the wrong estimator to use unchanged with the present U300 measurement density.

The replacement architecture should be:

```text
Cube raw IMU
       ↓
IMU propagation
       ↓
ESKF / EKF prediction
       ↑
       │
U300 robust ego-velocity measurement
       │
       └── covariance + observability + innovation gate
       ↑
altimeter height update
       ↓
local radar submap registration
       ↓
position/yaw correction
       ↓
navigation health manager
       ↓
MAVLink ODOMETRY
       ↓
Cube EKF
```

The first milestone is **radar-inertial odometry**, not full SLAM.

---

# 1. Research conclusion

Several published/open-source systems point to the same design direction, but they are not interchangeable.

## HKUST RIO

HKUST RIO is an optimization-based radar-inertial system. It models radar point uncertainty in polar coordinates, performs radar tracking/data association, and combines IMU preintegration with Doppler and radar point residuals.

Sources:

- https://github.com/HKUST-Aerial-Robotics/RIO
- https://arxiv.org/abs/2402.16082

The important research ideas to keep are:

- radar uncertainty
- Doppler residual
- radar point residual
- IMU preintegration
- robust data association
- joint estimation

The important thing to change for the U300 system is the assumption that the available radar points provide enough independent constraints for every update.

HKUST's released ARS548 configuration explicitly enables both Doppler and point-to-point residuals and uses an observation threshold for persistent point features. Those settings were developed around the ARS548 data format and experiment platform, not the present sparse U300 stream.

Therefore:

> Use HKUST RIO as a research reference, not as an unchanged flight estimator for the current U300 input.

---

## REVE / EKF-RIO / x-RIO

The older RIO family uses an EKF architecture and a dedicated radar ego-velocity estimator.

Sources:

- https://github.com/christopherdoer/reve
- https://github.com/YLRK/rio
- https://github.com/christopherdoer/rio/tree/main/x_rio

REVE explicitly estimates radar ego velocity and covariance for later fusion.

That architectural separation is valuable:

```text
radar scan
   ↓
robust radar ego velocity + covariance
   ↓
EKF/RIO measurement update
```

The radar velocity is a **measurement**, not the navigation state.

The x-RIO project reports online drone navigation demonstrations and publishes both filter state and covariance.

This is much closer to what is needed for the first U300 flight stack.

---

## ETH BRIO

BRIO is especially relevant to this project because it was designed for multicopter radar-inertial navigation and includes barometer support and zero-velocity tracking.

Source:

- https://github.com/ethz-asl/rio
- https://arxiv.org/abs/2408.05764

The useful ideas are:

- graph-based radar-inertial state estimation
- robust statistical treatment of radar outliers
- barometric height support
- zero-velocity tracking
- multicopter flight validation
- handling moving objects and ghost targets

For the current U300 system, use the same concepts even if the exact implementation is not copied.

---

## iRIOM

Source:

- https://arxiv.org/abs/2303.13962

iRIOM uses:

```text
radar ego velocity
+
scan-to-submap registration
+
6D IMU
+
iterative EKF
+
loop closure
```

This is a strong architectural reference for the second stage.

Important lesson:

> Do not make scan-to-scan GICP the primary estimator. Use radar ego velocity and IMU for the local state, then use scan-to-submap geometry as another measurement source.

---

## Temporal calibration

Sources:

- https://arxiv.org/abs/2503.02509
- https://arxiv.org/abs/2603.19958

These works demonstrate that radar/IMU temporal misalignment can materially affect radar-inertial odometry.

The current stack's use of Jetson monotonic time for both radar and Cube IMU is acceptable as a temporary common time base, but the actual sensor-to-host timing and jitter still need to be measured.

Do not introduce separate clock domains again.

---

# 2. The present U300 failure is a low-information radar update

The critical line in the supplied run is:

```text
Radar: 3 pts, 3 static, pos=[-0.07, 0.03, -0.02],
vel=[-0.71, 0.33, -0.16]
```

at the beginning of a stationary test.

Source: supplied run log.

A stationary vehicle should not suddenly acquire approximately 0.8 m/s of 3D velocity from three weak radar detections.

The later log repeatedly reports:

```text
0 static
```

while velocity becomes tens of m/s and position becomes hundreds/thousands of metres.

By the end, the run reports approximately:

```text
[-1069.34, 1457.91, -382.27] m
```

with 2148.77 m of integrated trajectory distance during a stationary experiment.

Source: supplied run log.

This proves that the estimator is not receiving a reliable radar correction.

---

# 3. Why 3-point Doppler is dangerous on the U300

For a static radar target, a simplified measurement model is:

\[
d_i = -u_i^T v_R + n_i
\]

where:

- \(d_i\) is measured Doppler;
- \(u_i\) is the unit line-of-sight vector;
- \(v_R\) is radar ego velocity;
- \(n_i\) is measurement noise/model error.

For \(N\) detections:

\[
A v_R = b
\]

with:

\[
A =
\begin{bmatrix}
-u_1^T\\
-u_2^T\\
\vdots\\
-u_N^T
\end{bmatrix}
\]

and:

\[
b =
\begin{bmatrix}
d_1\\
d_2\\
\vdots\\
d_N
\end{bmatrix}.
\]

With exactly 3 measurements:

\[
A\in\mathbb{R}^{3\times3}.
\]

If the matrix is invertible, a velocity solution exists.

That does **not** mean the solution is accurate.

If:

\[
A^{-1}
\]

is poorly conditioned, small Doppler errors become large velocity errors.

This is the exact reason a sparse radar update must use an information/conditioning test.

The current U300 stream frequently provides only a handful of detections. Therefore a 3-point solution must never automatically become the navigation truth.

---

# 4. Correct U300 velocity estimator

The new U300 radar front end should estimate a body-frame radar ego velocity measurement and its covariance.

## Step 1 — convert every point to a unit direction

\[
u_i = \frac{p_i}{\|p_i\|}
\]

where:

\[
p_i=[x_i,y_i,z_i]^T.
\]

## Step 2 — apply one verified Doppler sign convention

Exactly one module owns the sign conversion.

Do not flip Doppler in both Python and C++.

## Step 3 — compensate radar/IMU lever arm

The radar velocity is affected by vehicle angular velocity when the radar is offset from the IMU/body origin.

Use the calibrated extrinsic vector:

\[
t_{BR}.
\]

The radar velocity includes the rotational contribution:

\[
v_R = v_B + \omega_B\times t_{BR}
\]

with the exact transform convention fixed and unit-tested.

## Step 4 — robust weighted estimation

Start from:

\[
\hat v_R =
(A^T W A)^{-1}A^T W b.
\]

But do not use a fixed identity \(W\).

Weights should be reduced for:

- Doppler outliers;
- dynamic targets;
- poor radar quality;
- inconsistent track history.

Use a robust loss such as Huber/Tukey or a graduated/non-convex weighting strategy.

iRIOM uses graduated non-convexity to make radar ego-velocity robust to moving objects and multipath.

## Step 5 — covariance

Estimate:

\[
P_v=(A^TWA)^{-1}
\]

with appropriate measurement noise propagation.

Do not report extremely small covariance just because the algebraic system has a solution.

## Step 6 — information test

Compute:

\[
H_v=A^TWA.
\]

Check:

\[
\lambda_{\min}(H_v)
\]

and/or the singular values of \(A\).

The update is invalid when the information matrix is nearly singular.

---

# 5. Minimum-data rule

For the first U300 implementation:

```text
N < 6:
    do not perform a full 3D radar velocity update

N >= 6:
    check rank and conditioning

rank < 3:
    no full 3D velocity update

poor conditioning:
    no full 3D velocity update

good rank + good conditioning:
    radar velocity update allowed
```

These are starting engineering gates, not universal constants.

They should be calibrated from actual U300 logs.

A better future implementation can allow partial-direction updates when only a subspace is observable.

---

# 6. Use short temporal radar batches

Because your U300 can be sparse per frame, do not force every single radar frame to solve 3D velocity independently.

The U300 manufacturer lists approximately 20 Hz refresh and 120° azimuth / 24° elevation FOV.

Source:

https://www.linpowave.com/product/4d-mmwave-radar-u300-for-drone

A short temporal batch can be used:

```text
frame k
frame k-1
frame k-2
frame k-3
...
```

for example over roughly 100–300 ms initially.

Each detection keeps its timestamp.

IMU attitude/rotation is used to compensate the temporal differences.

This increases the number of Doppler constraints without pretending that one frame contains 20 strong targets.

The batch must remain short enough that acceleration does not invalidate the assumed velocity model.

---

# 7. Never let the bad predicted velocity decide which points are static

The current RIO tracker uses predicted velocity to calculate static points.

That creates a dangerous circular dependency:

```text
bad velocity prediction
        ↓
wrong static-point classification
        ↓
wrong radar update
        ↓
worse velocity
```

Instead use an iterative robust procedure:

```text
initial broad static hypothesis
        ↓
robust ego-velocity estimate
        ↓
compute Doppler residuals
        ↓
reduce weight of inconsistent targets
        ↓
re-estimate velocity
        ↓
converge
```

Do not turn one previous bad velocity into a hard point deletion rule.

---

# 8. Correct fusion architecture

The core should be an Error-State Kalman Filter (ESKF) or a carefully implemented EKF.

This is what I recommend as the first U300 flight estimator.

## State

\[
x=
[p_W,\;v_W,\;q_{WB},\;b_a,\;b_g]
\]

with covariance:

\[
P.
\]

## Prediction

Use the high-rate Cube IMU:

\[
\dot p=v
\]

\[
\dot v=R(q)(a_m-b_a-n_a)+g
\]

\[
\dot q=
\frac12\Omega(\omega_m-b_g-n_g)q.
\]

Prediction happens at the IMU rate.

## Radar update

Radar supplies:

\[
z_v = \hat v_R
\]

with:

\[
R_v=P_v.
\]

The innovation is:

\[
r=z_v-h_v(x).
\]

Then:

\[
S=H P H^T+R_v
\]

\[
K=P H^T S^{-1}.
\]

State correction:

\[
\delta x=Kr.
\]

Covariance should be updated using the Joseph form:

\[
P^+
=
(I-KH)P^-(I-KH)^T
+
KRK^T.
\]

This explicitly answers the user's concern:

> The estimator should be driven by IMU prediction between radar measurements, but the radar must continuously correct velocity and state covariance through actual measurement updates.

The filter should never simply integrate IMU and occasionally replace values.

---

# 9. Radar update must be visible in the log

Every radar update must log:

```text
v_pred
v_radar
innovation
innovation_covariance
Kalman_gain
P_before
P_after
accepted/rejected
reject_reason
```

Example:

```text
RADAR UPDATE
predicted velocity: [ 0.03, -0.02, 0.01]
radar velocity:     [ 0.01, -0.04, 0.00]
innovation:         [-0.02, -0.02, -0.01]

Mahalanobis distance: 1.3
accepted: YES

velocity covariance:
before: trace=0.42
after : trace=0.09
```

When radar is wrong:

```text
RADAR UPDATE
accepted: NO
reason: ill_conditioned
```

This makes it possible to prove whether radar is actually correcting the IMU.

---

# 10. Stationary initialization

The very first operation should not be ordinary flight estimation.

Use a stationary initialization interval.

For approximately 5–10 seconds:

```text
Cube IMU
+
U300
+
altimeter
```

Estimate:

\[
b_g
\]

and an initial accelerometer/gravity relationship.

Stationary detection should combine:

```text
low gyro RMS
+
accelerometer norm close to g
+
stable radar Doppler
+
stable altimeter
```

not one sensor alone.

Then initialize:

\[
v=0.
\]

---

# 11. Add ZUPT

Zero-velocity updates are highly appropriate for your stationary test and can also be useful during flight when the vehicle genuinely settles.

When stationary:

\[
z_{ZUPT}=
\begin{bmatrix}
0\\0\\0
\end{bmatrix}
\]

and:

\[
H_{ZUPT}=
\frac{\partial v}{\partial x}.
\]

Fuse it as a normal measurement.

BRIO explicitly includes zero-velocity tracking as part of its multicopter radar-inertial design.

Do not enable ZUPT merely because radar has few points.

Stationarity must be independently detected.

---

# 12. Altimeter update

The altimeter provides an independent vertical measurement.

Use it as:

\[
z_h=h_{\text{AGL}}.
\]

This constrains vertical state but cannot determine horizontal X/Y position.

For the current hardware:

```text
U300      → velocity / radar geometry
Cube IMU  → high-rate dynamics
altimeter → height
```

This division is clean.

---

# 13. What happens in a stationary 60-second test

A correct system should do approximately:

```text
0–10 s
    stationary initialization
    estimate gyro bias
    establish gravity
    establish radar zero-velocity statistics

10–60 s
    IMU prediction
    radar reports near-zero ego velocity
    radar covariance is finite
    radar update drives velocity toward zero
    ZUPT can confirm zero velocity when valid
    altimeter keeps vertical state stable

position:
    remains bounded near origin
```

A result such as:

```text
position = [-1000, +1500, -382] m
```

is an automatic test failure.

---

# 14. Radar-only position is not the solution

A key physics point:

Doppler radar primarily provides velocity.

It does not directly give global X/Y position.

Therefore:

```text
IMU + Doppler
```

can provide local velocity and dead-reckoned position.

But:

```text
IMU + Doppler
```

alone cannot guarantee bounded position drift indefinitely.

To control long-term position drift, add:

```text
radar scan-to-submap
```

and later:

```text
loop closure
```

and for mission-scale global localization:

```text
terrain/DEM matching
```

This is consistent with iRIOM's architecture.

---

# 15. SLAM stage — only after RIO is stable

Once:

```text
U300 + IMU + radar velocity EKF
```

passes all bench tests, add:

```text
local radar submaps
```

Architecture:

```text
RIO pose
   ↓
deskew radar with IMU trajectory
   ↓
local submap
   ↓
robust GICP/NDT
   ↓
registration measurement
   ↓
EKF / pose graph
```

GICP is therefore:

```text
measurement source
```

not:

```text
navigation truth
```

---

# 16. Ground handling

Do not throw away the ground.

Ground can be used as:

```text
height constraint
roll/pitch evidence
surface structure
```

but a perfectly flat plane does not give unrestricted 6-DOF pose information.

If the radar sees only a plane:

```text
translation/yaw observability becomes weak
```

and the health manager must report degraded geometry.

---

# 17. U300 FOV must drive mounting decisions

The manufacturer currently lists approximately:

```text
Azimuth FOV:    120°
Elevation FOV:  24°
```

Source:

https://www.linpowave.com/product/4d-mmwave-radar-u300-for-drone

This means a downward-looking U300 has a comparatively narrow vertical field.

At altitude, you may see mostly ground rather than useful vertical structure.

Therefore the mounting must be selected experimentally.

For pure local odometry, the radar should ideally see a mixture of:

```text
ground
+
static vertical structure
+
terrain variation
```

rather than only a flat surface.

---

# 18. U300 should not be treated as ARS548

This is one of the most important design decisions.

HKUST's released ARS548 configuration assumes fields such as:

```text
range
azimuth
elevation
rangeSTD
azimuthSTD
elevationSTD
velocitySTD
RCS
```

and uses corresponding uncertainty models.

The U300 integration currently provides:

```text
XYZ
Doppler
```

plus the uncertainty values we assign ourselves.

Therefore:

> Do not copy ARS548 noise numbers and claim they describe the U300.

Build an empirical U300 noise model.

---

# 19. U300 uncertainty calibration

Perform static reflector tests at several ranges.

At each range estimate:

```text
range mean/std
azimuth mean/std
elevation mean/std
Doppler mean/std
```

Repeat across:

```text
center FOV
left/right FOV
up/down FOV
different target sizes
different surfaces
```

Use the resulting model as:

\[
R_v
\]

and:

\[
\Sigma_{r\theta\phi}.
\]

---

# 20. Do not use current fake "quality=1.0"

Because the current U300 firmware interface does not expose the same target-quality/RCS/SNR fields as the HKUST ARS548 interface, the adapter must not claim:

```text
quality = 1.0
```

for every point.

Instead start with:

```text
quality = UNKNOWN
```

and derive confidence from:

```text
range
Doppler residual
track persistence
geometry
measurement covariance
```

If the firmware later provides SNR/noise/quality information, add it explicitly.

---

# 21. Time architecture

Keep all incoming sensor samples in one host monotonic time domain initially.

Use:

```text
U300 receive timestamp
Cube receive timestamp
altimeter receive timestamp
```

and measure:

```text
sensor timestamp if exposed
host receive timestamp
processing timestamp
```

Do not mix:

```text
Cube boot time
Jetson monotonic
UNIX epoch
```

without an explicit conversion.

---

# 22. IMU source

The current `SCALED_IMU2` conversion is unit-wise appropriate because MAVLink specifies that SCALED_IMU2 uses:

```text
accel = mG
gyro  = mrad/s
```

Source:

https://mavlink.io/en/messages/common.html

That part should remain.

But the axis transformation must be experimentally verified.

Do not assume that:

```text
FRD → FLU
```

is correct for every consumer without validating the actual RIO convention.

The stationary gravity vector being approximately:

```text
[0.038, -0.022, 9.852]
```

is encouraging for the magnitude but does not prove all axis/sign conventions.

---

# 23. IMU test that must pass before RIO

Put the Cube flat and stationary.

Expected:

\[
\|\mathbf a\|\approx g
\]

and:

\[
\|\boldsymbol\omega\|\approx0.
\]

Then physically rotate the board:

```text
+90° roll
+90° pitch
+90° yaw
```

and verify the transformed axes.

This is the fastest way to discover an axis sign or frame error before RIO is involved.

---

# 24. RIO Ceres core disposition

Do not delete the copied HKUST RIO code.

Move it to:

```text
rio_core/HKUST_RIO/
```

and classify it as:

```text
RESEARCH_REFERENCE
```

rather than the initial flight estimator.

The current U300 stack should use a dedicated:

```text
u300_radar_velocity
+
eskf_rio
```

core first.

After that works, we can experiment with feeding additional U300 measurements into a factor-graph formulation.

This avoids forcing a sparse point cloud into an ARS548-oriented estimator before the sensor contract has been validated.

---

# 25. Recommended repository structure

```text
rio_stack/
    drivers/
        u300/
        cube/
        altimeter/

    estimator/
        u300_velocity/
        eskf_rio/
        state.py
        covariance.py
        health.py

    mapping/
        local_submap/
        registration/
        observability/

    autopilot/
        mavlink_output/

    rio_core/
        HKUST_RIO/
            # research/reference
        BRIO_reference/
            # optional reference
        REVE_reference/
            # optional reference

    tools/
        replay/
        analyze/
        calibration/

    tests/
        sensor/
        velocity/
        fusion/
        timing/
        regression/
```

---

# 26. First implementation milestone

The first working milestone must contain only:

```text
U300
+
Cube IMU
+
ESKF
+
robust radar ego velocity
+
altimeter
```

No:

```text
GICP
loop closure
DEM
autonomous navigation
```

yet.

---

# 27. Exact data flow

```text
U300 frame @ ~20Hz
    ↓
validate
    ↓
convert XYZ → direction
    ↓
estimate radar ego velocity
    ↓
compute covariance
    ↓
observability check
    ↓
innovation check
    ↓
Radar measurement
          │
          ▼
     ESKF update
          ↑
          │
Cube IMU @ high rate
          │
          ▼
      prediction

Altimeter
   ↓
height update

ESKF state
   ↓
health manager
   ↓
MAVLink ODOMETRY
   ↓
Cube EKF
```

---

# 28. Required state logs

At every update:

```text
timestamp
dt

IMU:
    accel
    gyro
    bias
    attitude

Radar:
    point_count
    static_count
    velocity_measurement
    velocity_covariance
    rank
    singular_values
    condition

Fusion:
    predicted_velocity
    radar_velocity
    innovation
    innovation_covariance
    Mahalanobis distance
    Kalman gain
    posterior velocity
    posterior covariance
    accepted/rejected
    reason

Altimeter:
    raw
    AGL
    validity
    age

Health:
    state
    quality
    reasons
```

This is necessary for diagnosing "radar is not fusing."

---

# 29. Test 1 — stationary

Pass criteria:

```text
gyro RMS small
acc norm ≈ g
radar velocity ≈ 0
radar update accepted when information is sufficient
ESKF velocity ≈ 0
position remains bounded
altimeter stable
```

Failure:

```text
radar gives > small non-zero velocity
```

→ investigate radar front end before running the ESKF.

Failure:

```text radar velocity ≈ 0
 but ESKF velocity ≠ 0
```

→ investigate EKF measurement model/frames/covariance.

Failure:

```text radar update repeatedly rejected
```

→ investigate covariance, sign, frame, or observability.

---

# 30. Test 2 — hand translation

Move approximately:

```text
0.5 m
1.0 m
```

Use an independent ruler/reference.

Expected:

```text velocity measurement follows movement
position estimate remains same order of magnitude
```

Do not demand millimeter-level accuracy.

The purpose is to establish correct physical direction and scale.

---

# 31. Test 3 — hand rotation

Do:

```text yaw
pitch
roll
```

without intentional translation.

Expected:

```text orientation changes
position does not run away
```

This isolates gyro and frame errors.

---

# 32. Test 4 — radar-only velocity

Temporarily do:

```text radar velocity estimator
```

without ESKF.

Record:

```text radar velocity
covariance
rank
residual
```

while the drone is stationary.

If this module cannot produce:

```text approximately zero velocity
```

with trustworthy covariance, there is no reason to debug the ESKF yet.

---

# 33. Test 5 — IMU-only

Run ESKF prediction without radar updates.

This is intentionally expected to drift.

This test tells us the baseline IMU error.

Then enable radar.

The improvement must be measurable.

---

# 34. Test 6 — radar + IMU

Compare:

```text IMU-only
vs
IMU + radar
```

Metrics:

```text velocity RMSE
velocity drift
position drift
attitude drift
covariance
```

This is the proof that fusion is actually working.

---

# 35. Test 7 — radar dropout

Remove radar input.

Expected:

```text state remains continuous
covariance grows
health degrades
```

When radar returns:

```text innovation checked
radar reaccepted
covariance contracts
```

---

# 36. Test 8 — bad radar points

Inject artificial:

```text huge Doppler outlier
wrong sign
wrong range
dynamic target
```

Expected:

```text measurement rejected/downweighted
state remains stable
```

---

# 37. Test 9 — flat ground

Place the radar over uniform ground.

Expected:

```text geometry quality degraded
velocity may still be available from Doppler
position/yaw geometry correction becomes weak
```

The health manager must distinguish:

```text radar velocity valid
```

from:

```text radar SLAM geometry invalid.
```

---

# 38. Test 10 — high-altitude scenario

When eventually testing higher altitude:

record:

```text AGL
point count
vertical spread
azimuth spread
radar velocity covariance
SLAM observability
```

Do not infer performance from the manufacturer's maximum detection range.

---

# 39. Navigation states

Use:

```text INITIALIZING
RADAR_VELOCITY_GOOD
RADAR_VELOCITY_DEGRADED
RIO_IMU_ONLY
SLAM_GOOD
SLAM_DEGRADED
RECOVERY
INVALID
```

Important distinction:

```text RIO_IMU_ONLY
```

means:

> The estimator is running, but radar is not currently trusted.

It must not be reported as healthy radar-inertial navigation.

---

# 40. What the user should see on the terminal

Instead of:

```text Radar: 3 pts, 3 static, pos=..., vel=...
```

log:

```text
[RIO]
radar_pts=8
static_prob=0.82
v_radar=[0.01,-0.02,0.00]
sigma_v=[0.08,0.07,0.09]
rank=3
cond=4.2

prediction:
v=[0.03,-0.01,0.02]

innovation:
[-0.02,-0.01,-0.02]

mahalanobis=1.4
radar_update=ACCEPT

posterior:
v=[0.012,-0.018,0.004]

covariance_trace:
before=0.19
after=0.08
```

This immediately tells us whether radar is actually correcting IMU.

---

# 41. Important rule for covariance

Never make radar covariance smaller just to force the ESKF to trust radar.

Never make IMU covariance larger just to force radar to dominate.

Use measured sensor statistics.

Fusion weight should emerge from:

\[
K=P H^T(HPH^T+R)^{-1}.
\]

That is the mathematically correct answer to "which sensor should be trusted more?"

---

# 42. Why the current stationary result got worse

The current run gives strong evidence that the new Ceres-based integration made the system more unstable because:

1. U300 provides very few detections.
2. The Doppler residual has been allowed to use those few points.
3. Three points can provide a 3D algebraic solution but can be numerically fragile.
4. The resulting false velocity becomes the predicted state.
5. Static-point classification then collapses.
6. With no reliable static points, radar stops providing useful Doppler correction.
7. IMU integration continues the wrong state.
8. The result explodes.

This is exactly the opposite of the desired fusion behavior.

---

# 43. Correct strategy for this project

The new development plan should therefore be:

```text
DO NOT:
keep tuning current Ceres RIO

DO:
1. build robust U300 ego velocity measurement
2. compute honest covariance
3. add rank/conditioning gate
4. implement ESKF measurement update
5. implement stationary/ZUPT handling
6. add altimeter height update
7. demonstrate radar actually contracts covariance
8. only then add scan-to-submap SLAM
```

---

# 44. Why this is preferable

It gives us independent diagnostics:

```text
U300 velocity front end
        ↓
is radar correct?

ESKF
        ↓
is fusion correct?

submap SLAM
        ↓
is geometric localization correct?

MAVLink
        ↓
is autopilot fusion correct?
```

At present these questions are entangled.

The redesign makes them separable.

---

# 45. Long-term system

The mature system becomes:

```text
                  ┌──────────────┐
                  │     U300     │
                  └──────┬───────┘
                         ↓
                Radar Ego Velocity
                  + covariance
                         │
                         ▼
Cube IMU ───────►   ESKF-RIO   ◄──── Altimeter
                         │
                         ▼
                  local state
                         │
                         ▼
                  radar submaps
                         │
                         ▼
               scan-to-submap update
                         │
                         ▼
                   pose graph
                         │
                         ▼
               global localization
                 DEM / terrain
                         │
                         ▼
                 health manager
                         │
                         ▼
                 MAVLink ODOMETRY
                         │
                         ▼
                    Cube EKF
```

---

# 46. Definition of success for the next test

Do not run another 60-second stationary test until the following standalone tests pass:

### A. IMU

```text
stationary gravity correct
gyro near zero
axis signs verified
timestamp monotonic
```

### B. Radar

```text
stationary radar ego velocity near zero
no 3-point false full-3D solution
rank/conditioning logged
covariance reasonable
```

### C. Fusion

```text radar update visibly changes velocity estimate
radar update reduces covariance
radar rejection leaves estimator stable
```

### D. Altimeter

```text stable height
stale samples rejected
```

Only then run:

```text
60-second stationary
```

Expected result:

```text
distance ≈ very small
displacement ≈ very small
velocity ≈ zero
```

---

# 47. Sources

HKUST RIO:
- https://github.com/HKUST-Aerial-Robotics/RIO
- https://arxiv.org/abs/2402.16082

ETH-ASL BRIO:
- https://github.com/ethz-asl/rio
- https://arxiv.org/abs/2408.05764

Original RIO/x-RIO:
- https://github.com/christopherdoer/rio
- https://github.com/YLRK/rio

REVE:
- https://github.com/christopherdoer/reve

4D iRIOM:
- https://arxiv.org/abs/2303.13962

Radar-inertial temporal calibration:
- https://arxiv.org/abs/2503.02509
- https://arxiv.org/abs/2603.19958

MAVLink IMU message definitions:
- https://mavlink.io/en/messages/common.html

Linpowave U300:
- https://www.linpowave.com/product/4d-mmwave-radar-u300-for-drone

---

# 48. Final design decision

For this U300 project:

> **Do not continue with the current HKUST point-level Ceres RIO as the first flight estimator.**

Keep the HKUST implementation as a research/reference branch.

Build:

```text
ROBUST U300 EGO VELOCITY
        +
ESKF RIO
        +
ALTIMETER
```

first.

Then:

```text
ESKF RIO
        +
RADAR SUBMAP SLAM
```

Then:

```text
LOCAL SLAM
        +
TERRAIN / GLOBAL LOCALIZATION
```

This path is more inspectable, easier to validate, and much less likely to hide a broken radar measurement behind a large nonlinear optimizer.

Most importantly, it gives a direct quantitative answer to the user's concern:

> **We will be able to see, at every radar update, exactly how much the radar measurement changed the IMU prediction, how much the covariance contracted, and why the radar update was accepted or rejected.**
