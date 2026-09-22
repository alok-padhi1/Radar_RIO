# RIO Stack — Critical Runtime Audit (2026-09-22)

## Executive conclusion

The current build must **not** be used for flight navigation. The observed result:

- physical hand motion: ~0.5–1 m
- reported RIO distance: 273.76 m
- reported net displacement: 273.54 m
- X: +168.20 m
- Y: -215.21 m
- Z: -14.77 m

is not a normal tuning problem.

The current copied stack contains multiple implementation regressions. Several are sufficient to make the estimator effectively IMU-only or to feed it physically inconsistent timestamps/measurements.

The most important problems, in order:

1. **Radar and IMU timestamps are in different clock domains.**
2. **The native RIO tracker/preprocessor initialization was omitted from the ZMQ port.**
3. **Therefore radar correspondences/static points can collapse to zero, disabling the radar factors.**
4. **`usePoint2PointResidual` is hard-coded false in the copied ZMQ RIO.**
5. **The U300 Doppler sign is corrected in Python, then the C++ RIO applies another `-point.doppler`.**
6. **The ZMQ publisher sends `curState`, while optimization writes the result into `predState`.**
7. **Ceres termination is hard-coded to `CONVERGENCE` instead of using the actual solver result.**
8. **`system.yaml` is loaded by the supervisor but its values are not actually passed to the modules.**
9. **The supervisor does not launch/own `rio_node`; the runbook requires a separate native process. This allows stale `rio_node` instances.**
10. **The native binary in the uploaded copy is older than the modified C++ source.**
11. **`RawIMUReader` mixes `RAW_IMU`, `SCALED_IMU`, and `SCALED_IMU2`, even though they do not share the same unit contract.**
12. **The `RAW_IMU` conversion assumes scaled mg/mdeg/s data even though MAVLink defines RAW_IMU as unscaled raw values.**
13. **The IMU latest sample is repeatedly resent at the supervisor loop rate instead of being consumed once per incoming sensor sample.**
14. **Health/time-sync state is not actually updated, so `INVALID` is expected and currently unrelated to estimator correctness.**

---

# 1. Why 273.76 m is plausible from this code

The native RIO implementation has no usable radar constraints in the present ZMQ port.

`RadarTracker` contains these defaults:

```cpp
double distanceThreshold = 0;
double velThreshold = 0;
double SigmaR = 0;
double SigmaTheta = 0;
double SigmaPhi = 0;
double pdfThreshold = 0;
```

The original HKUST RIO setup configures the tracker with:

```cpp
scan2scanTracker.setMatchingThreshold(1, 0.3);

scan2scanTracker.setMatchingParameters(
    SigmaRange,
    SigmaAzimuth,
    SigmaElevation,
    numSigma,
    useRCSFilter);

scan2scanTracker.setPredictedVelocityThreshold(0.3);
```

That setup is present in upstream `rosWarper.cpp` but is absent from the copied ZMQ `zmqWarper.cpp`.

Because the copied port does not call these methods:

- distance matching threshold stays 0;
- predicted velocity threshold stays 0;
- tracking is unable to establish normal matches;
- predicted static-point selection can reject everything;
- `staticPoint` can become empty.

At the same time, the copied ZMQ RIO sets:

```cpp
useDopplerResidual = true;
usePoint2PointResidual = false;
```

So even though Doppler is nominally enabled, there may be no static points for the Doppler factor, and the point factor is explicitly disabled.

That leaves the optimization dominated by IMU propagation.

This is the first major explanation for the enormous drift.

---

# 2. Timestamp bug — highest priority

## Radar

`drivers/u300/reader.py` does:

```python
self.latest_frame = self.adapter.convert_frame(
    pts,
    health.rx_timestamp
)
```

The decoder's `health.rx_timestamp` is:

```python
time.monotonic()
```

So radar time is Jetson host monotonic time.

## IMU

`drivers/cube/raw_imu_reader.py` does:

```python
t_sensor = getattr(msg, 'time_usec', 0) / 1e6
```

and passes this directly into RIO.

For MAVLink `RAW_IMU`, `time_usec` can be UNIX epoch time or time since system boot. The receiver is expected to determine which representation is being used from its magnitude.

The C++ RIO then treats radar and IMU timestamps as if they were directly comparable:

```cpp
ros::Time(timestamp)
```

and:

```cpp
imuData.getDataWithinInterval(startTime, endTime)
```

No clock-offset conversion exists.

Therefore:

```text
RADAR = Jetson MONOTONIC
IMU   = Cube time_usec
```

is being used as though:

```text
RADAR = IMU
```

This is mathematically invalid.

A radar-inertial estimator is highly sensitive to sensor time alignment. Recent 2025–2026 radar-inertial work explicitly studies temporal calibration because timestamp offsets can materially degrade odometry.

---

# 3. IMU unit bug

The current reader accepts:

```text
RAW_IMU
SCALED_IMU
SCALED_IMU2
```

and treats RAW_IMU like a scaled message:

```python
ax = msg.xacc / 1000.0 * 9.80665
...
gx = np.radians(msg.xgyro / 1000.0)
```

That is not a valid general RAW_IMU conversion.

MAVLink defines:

- `SCALED_IMU`: accelerometer in mG, gyro in mrad/s.
- `RAW_IMU`: raw sensor values without scaling.
- `HIGHRES_IMU`: SI units (m/s² and rad/s) in body/NED convention.

Use one known message type, not a mixture.

For the first hardware validation, use:

```text
SCALED_IMU
```

or:

```text
HIGHRES_IMU
```

and explicitly convert its documented units.

Do not accept arbitrary `RAW_IMU` data until the exact Cube/firmware raw scaling has been experimentally established.

---

# 4. Gyro scaling can produce exactly this kind of drift

Suppose a message is actually in mrad/s and contains:

```text
1000 mrad/s
```

which is:

```text
1 rad/s
```

The current RAW_IMU code computes:

```python
np.radians(1000 / 1000)
```

which is:

```text
0.01745 rad/s
```

a factor of approximately 57 too small.

If vehicle rotation is underestimated, the gravity vector is projected into the wrong horizontal directions.

Even a persistent gravity-leakage error on the order of ~0.5–1 m/s² can produce hundreds of metres of integrated displacement over tens of seconds.

That is consistent with the magnitude of the observed failure.

This is why the IMU input must be fixed before evaluating RIO or SLAM.

---

# 5. The current native binary is stale

In the uploaded copy:

```text
rio_core/HKUST_RIO/rio/build/rio_node
```

has an older modification time than:

```text
rio_core/HKUST_RIO/rio/node/zmqWarper.cpp
```

The observed files are approximately:

```text
binary:     2026-09-21 22:12
C++ source: 2026-09-22 16:02
```

Therefore, unless you rebuilt after modifying the C++ source, the running `rio_node` cannot contain those source changes.

The runbook also requires the native binary to be started separately:

```bash
./rio_node
```

while the Python supervisor only attaches to the ZMQ endpoints.

Therefore:

> If a `rio_node` from an earlier run is still alive, the new supervisor can receive state from that old process.

This is especially important because the supervisor itself never launches `rio_node`.

---

# 6. `system.yaml` is currently not the single source of truth

The supervisor does:

```python
sys_config = load_config(self.config_path)
```

but then creates:

```python
time_cfg = TimeSyncConfig()
health_cfg = HealthConfig()
out_cfg = OutputConfig()
```

and:

```python
U300Adapter()
```

and:

```python
RIOInterface()
```

without using the loaded YAML configuration.

Therefore the YAML values for:

- extrinsic
- radar parameters
- noise
- health
- timing
- output

are not actually applied to those objects.

The native C++ RIO has another independent problem: it does not read `system.yaml` at all. It hard-codes:

```cpp
useWeightedResiduals = true;
useDopplerResidual = true;
usePoint2PointResidual = false;
SigmaRange = 0.1;
SigmaAzimuth = 0.05;
SigmaElevation = 0.05;
...
```

and:

```cpp
radarExParam.vec.setZero();
radarExParam.vec.z() = -0.05;
radarExParam.rot.setIdentity();
```

So the `50°` radar pitch in YAML is not being used.

The YAML comment calls these extrinsic values placeholders. They must not be treated as calibrated values.

---

# 7. Doppler sign is currently applied twice

Python adapter:

```python
v_corrected = v_raw * self.doppler_sign
```

and default configuration:

```yaml
sign: -1
```

Then native RIO does:

```cpp
RadarImuVelocityResidual(
    angularVel,
    -point.doppler,
    ...
)
```

So the pipeline is currently:

```text
U300 raw Doppler
    ↓
Python sign × (-1)
    ↓
C++ sign × (-1)
    ↓
effective original sign
```

That violates the design rule:

```text
apply Doppler convention exactly once
```

The fix is to choose one ownership location.

Recommended:

```text
U300 adapter
    ↓
verified RIO convention
    ↓
C++ uses point.doppler directly
```

The actual sign still must be verified on the bench.

---

# 8. Optimized result is not what the ZMQ publisher sends

The copied C++ optimizer does:

```cpp
ceres::Solve(option, &problem, &summary);
recoverState(ceres::CONVERGENCE);
```

`recoverState()` writes the optimized solution into:

```cpp
predState
```

but `publish()` sends:

```cpp
curState
```

Specifically:

```cpp
msg["position"].append(curState.vec.x());
msg["position"].append(curState.vec.y());
msg["position"].append(curState.vec.z());
```

and:

```cpp
msg["velocity"].append(curState.vel.x());
...
```

Upstream RIO's ROS publisher uses `predState`.

Therefore the ZMQ adaptation has a state-publication bug.

---

# 9. Ceres failure handling is unsafe

The copied code calls:

```cpp
ceres::Solve(option, &problem, &summary);
recoverState(ceres::CONVERGENCE);
```

It never passes:

```cpp
summary.termination_type
```

So an actual solver failure can still be treated as convergence.

For flight use, this must become:

```cpp
ceres::Solve(option, &problem, &summary);

if (summary.IsSolutionUsable() &&
    summary.termination_type != ceres::FAILURE) {
    recoverState(summary.termination_type);
} else {
    publishInvalidState(...);
}
```

The exact acceptance policy should be based on Ceres's actual termination/usable-solution state, not hard-coded success.

---

# 10. Health is not currently protecting the output

The terminal shows:

```text
Navigation mode: INITIALIZING → INVALID
```

That is expected from the current code.

The supervisor never calls:

```python
update_radar_health(...)
update_imu_health(...)
```

and never sets:

```python
time_sync_healthy = True
```

The health layer therefore cannot become valid.

This does NOT explain the 273 m itself.

It does explain why the system's new safety layer is not yet functional.

Also, because `--no-mavlink` is used, MAVLink output is disabled, but RIO processing continues and its position is still used for the shutdown summary.

---

# 11. IMU samples are being duplicated

The supervisor does:

```python
imu_sample = self.imu_reader.latest_raw
```

and:

```python
self.rio_bridge.send_imu(imu_sample)
```

every ~5 ms.

The IMU reader itself updates `latest_raw` only when an incoming IMU message arrives.

Therefore the same IMU sample can be transmitted repeatedly between actual sensor updates.

The native C++ IMU manager therefore sees:

```text
sample t=10.000
sample t=10.000
sample t=10.000
sample t=10.020
sample t=10.020
...
```

These duplicate samples have zero delta time, so they are not the primary cause of 273 m, but they violate the estimator's input contract and must be removed.

The reader should queue each sample exactly once.

---

# 12. What to do immediately

## Do not fly this version.

Also do not tune GICP, Doppler thresholds, covariance thresholds, or radar mounting from this version.

First restore estimator correctness.

---

# 13. Safe recovery sequence

## Step 1 — kill every RIO process

On the Jetson:

```bash
pkill -f '/rio_node' || true
```

Then:

```bash
pgrep -af rio_node
```

Expected:

```text
no output
```

## Step 2 — remove old IPC endpoints

After all RIO/supervisor processes are stopped:

```bash
rm -f /tmp/rio_sensor_in
rm -f /tmp/rio_state_out
```

## Step 3 — rebuild the C++ estimator

```bash
cd ~/rio_flight_test/rio_stack\ copy/rio_core/HKUST_RIO/rio

rm -rf build
mkdir build
cd build

cmake ..
make -j"$(nproc)"
```

Then verify:

```bash
stat ./rio_node
stat ../node/zmqWarper.cpp
```

The binary must be newer than the source used to build it.

## Step 4 — do not start the supervisor yet

First test the Cube IMU stream independently.

We need to verify:

```text
message type
timestamp magnitude
accelerometer units
gyro units
axis directions
```

Use SCALED_IMU or HIGHRES_IMU.

## Step 5 — put both sensors onto one clock

For the first implementation:

```text
U300 timestamp = Jetson monotonic
IMU timestamp  = Jetson monotonic
```

using a measured Cube-time → Jetson-time offset.

Do not use two unrelated time origins.

## Step 6 — configure the native RIO tracker

Restore the upstream initialization:

```cpp
radarPreprocessor.addFOVParams(...);
radarPreprocessor.setVelParams(...);
radarPreprocessor.setDistanceParams(...);

scan2scanTracker.setMatchingThreshold(...);
scan2scanTracker.setMatchingParameters(...);
scan2scanTracker.setPredictedVelocityThreshold(...);
```

Then validate tracker output independently.

## Step 7 — make Doppler sign single-owner

Use:

```text
U300 adapter → verified RIO Doppler convention
```

and no second sign flip inside RIO.

## Step 8 — publish `predState`

Change the ZMQ publisher to use the actual optimized state.

## Step 9 — use real Ceres termination

Never publish an invalid optimization as a successful RIO state.

---

# 14. First validation test

Do NOT test walking first.

Test:

## Test A — stationary drone on a table

Run for 60 seconds.

Expected:

```text
gyro ≈ 0
|accel| ≈ g
radar timestamps monotonic
IMU timestamps monotonic
radar–IMU time difference stable
velocity ≈ 0
position does not run away
```

The absolute position can be any local-origin value. The important result is that it remains bounded.

## Test B — manually rotate without translating

Rotate the drone:

```text
yaw
pitch
roll
```

The estimator's orientation must follow.

Position should remain comparatively stable.

This test specifically exposes gravity/gyro frame errors.

## Test C — 0.5–1 m hand translation

Only after A/B pass.

Then check:

```text
actual displacement ≈ estimator displacement
```

not:

```text
1 m → 273 m
```

## Test D — controlled radar reflector

Use a stationary wall/reflector.

Move the radar/drone toward and away from it.

Verify:

```text Doppler sign
radar range
radar velocity
```

before allowing RIO to consume the complete scene.

---

# 15. The correct debugging hierarchy

Use this order:

```text
Level 1
U300 raw packet
    ↓
Level 2
U300 decoded XYZ/Doppler
    ↓
Level 3
Cube raw/scaled IMU
    ↓
Level 4
common timestamps
    ↓
Level 5
radar↔IMU extrinsic
    ↓
Level 6
RIO radar tracking
    ↓
Level 7
RIO Doppler residual
    ↓
Level 8
RIO IMU residual
    ↓
Level 9
optimized state
    ↓
Level 10
local submap
    ↓
Level 11
MAVLink
```

Do not debug Level 9 by changing Level 10.

Do not debug Level 8 by changing Level 11.

---

# 16. What should NOT be enabled yet

Do not enable:

```text
GICP correction
loop closure
DEM matching
autonomous control
external position fusion
```

until RIO alone is stable.

The first target is:

```text
U300 + raw/scaled Cube IMU
        ↓
correctly timestamped
        ↓
correctly framed
        ↓
actual HKUST RIO
        ↓
stable local velocity/pose
```

---

# 17. Final diagnosis

The new blueprint architecture itself is not the problem.

The problem is that the current implementation is only a **partial integration** of the blueprint.

The biggest accidental regression is:

```text
Blueprint:
raw IMU + timestamped radar
        ↓
tightly coupled RIO

Actual current code:
wrong/mixed IMU message units
+
different timestamp clock
+
RIO tracker not initialized
+
Doppler potentially double-signed
+
point residual disabled
+
optimized state not published
+
stale native binary possible
        ↓
mostly IMU-driven / inconsistent estimate
```

That can absolutely produce the kind of enormous drift you are seeing.

---

# 18. External technical references

MAVLink Common Message Set:
- RAW_IMU is defined as unscaled raw sensor values.
- SCALED_IMU uses mG and mrad/s.
- HIGHRES_IMU provides SI-unit accelerometer/gyro measurements.
- `time_usec` may represent UNIX epoch time or time since boot.

Source:
https://mavlink.io/en/messages/common.html

Radar-inertial temporal calibration research:
- 2025: "Impact of Temporal Delay on Radar-Inertial Odometry"
- 2026: "Radar-Inertial Odometry with Online Spatio-Temporal Calibration via Continuous-Time IMU Modeling"

The papers explicitly investigate how radar/IMU temporal misalignment affects RIO.

---

# 19. Immediate success criterion

Before changing any SLAM algorithm:

```text
60 s stationary:
  no 10 m / 100 m / 273 m position runaway

manual rotation:
  correct orientation response
  no large horizontal position drift

0.5–1 m hand walk:
  RIO displacement same order as physical motion

radar/IMU timing:
  same clock domain

RIO tracker:
  non-zero valid tracks

Doppler residual:
  non-zero usable static points

Ceres:
  actual termination reported

publisher:
  optimized state

binary:
  rebuilt after source change
```

Only after these pass should SLAM/submaps be reintroduced.

