# Field Runbook: Bench → Walking → Vehicle → SITL → Drone, GPS-Denied Waypoint Flight

One consolidated walkthrough of everything in `ARCHITECTURE.md` and `NAVIGATION.md`,
in the order you actually run it, with what you're checking for at each step and the
explicit go/no-go gate before moving to the next one. Nothing here is optional or
skippable — each stage exists because it isolates one variable that would otherwise
be undebuggable in the air.

---

## Stage 0 — Before you touch a drone at all

- Safety pilot identified, with an RC transmitter that can retake manual control by
  a single mode switch, at every stage from Stage 5 onward.
- Autopilot's own geofence, RC-loss failsafe, and battery failsafe configured and
  tested independently of anything in this software stack — `nav_node.py`'s internal
  checks are a second layer, never the only layer.
- Airspace/regulatory clearance for the flight sorted before Stage 5b/5c.
- Everything below through Stage 4 needs **no propellers, no armed vehicle, no
  airspace** — do all of it on a bench and in simulation first, because it's where
  bugs are cheap.

**Cross-verification:**
```bash
python3 sitl_test.py --plumbing-only
```
All self-tests, rotation math, and port conflict checks must PASS.

---

## Stage 1 — Bench, static, radar + RIO + SLAM only

**Goal:** confirm the sensor pipeline itself is healthy before anything moves.

```bash
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 --no-mavlink
```

Radar sits still on the bench, tilted at your target mount angle, pointed at real
ground/clutter (not a wall a foot away). Watch the `[slam]` log lines:

### What to check:
| Metric | Healthy | Unhealthy |
|---|---|---|
| `raw → final` point counts | Stable frame to frame | Collapsing to near-zero (over-filtering) or barely dropping (under-filtering) |
| `n_ground` vs `n_off_plane` | Ground dominant on flat terrain | Ground minority → retune thresholds |
| `fitness` / `n_corr` | Nonzero and roughly consistent | Flapping between converged and rejected every frame |
| Exceptions/crashes | None | Any → fix before proceeding |

### Cross-verification:
- Run for **5+ minutes** stable, no exceptions
- Record the steady-state `raw → final` numbers — this is your Phase 1 exit baseline
- Verify ground plane inlier count (`n_ground`) is large majority of `n_final`

**Gate to Stage 2:** 5+ minutes stable run, ground plane dominant, GICP converging
on most keyframes.

---

## Stage 2 — Handheld walking

**Goal:** validate RIO velocity and SLAM's incremental pose against real motion,
where you have a real independent reference (paced or GPS-logged distance).

Same command as Stage 1. Carry the radar+companion computer on a chest rig or
handheld boom, tilt unchanged.

### Protocol:
1. Walk a measured straight line (50–100 m, paced or phone-GPS-logged as ground truth)
2. Walk a figure-eight to exercise turning
3. Log RIO's integrated velocity and SLAM's pose chain for the whole walk

### What to check:
| Metric | Gate | Method |
|---|---|---|
| RIO integrated distance | Within ~5% of measured distance | Compare logged distance vs tape/GPS |
| RIO inlier ratio | Above threshold for >90% of frames | Watch `doppler_rio` output |
| SLAM keyframe convergence | >80% converge during walking | Watch `slam_node` output |
| End-to-end latency | Measured and stable | Log UART → MAVLink timestamps |

### Cross-verification:
- Compare RIO velocity direction during turns (should rotate with you)
- Any velocity discontinuity = RIO produced garbage instead of `valid=False` → fix RANSAC gate
- Measure latency — you need this number for Stage 4's `EKF2_EV_DELAY`

**Gate to Stage 3:** distance error within 5%, latency measured and stable, no
unexplained velocity discontinuities.

---

## Stage 3 — Vehicle-mounted (car roof rack or pole out window)

**Goal:** validate behavior at flight-realistic speeds (5–15 m/s) and over longer
distances than walking allows.

Same stack, same checks as Stage 2, now at higher speed, 30+ minutes continuous,
varied terrain (open road, near buildings, near vegetation).

### What to check specifically:
- **Graceful degradation:** when radar can't see usable ground (open field, water,
  bridge), RIO should report `valid=False` and SLAM should reject the keyframe —
  NOT produce plausible-looking garbage.
- **Range gap behavior:** cross a bridge or open lot deliberately, confirm
  `valid=False` / rejected-keyframe, not a smooth-looking wrong trajectory.

### Cross-verification:
- GPS-log the drive for ground truth comparison
- Look for the `valid=False` count during known illumination gaps
- If you see a smooth but wrong trajectory during a gap → that's the dangerous
  failure mode from PIPELINE_NOTES.md — fix before flight

**Gate to Stage 4:** 30+ minutes, no crashes, confirmed `valid=False` behavior
during genuine illumination gaps.

---

## Stage 4 — SITL, full nav loop, no real airframe

**Goal:** validate `nav_node.py`'s control logic, MAVLink wiring, arming/mode
switching, and failsafe behavior end-to-end with zero risk.

### Step 4a: MAVLink plumbing check
```bash
# In separate terminal: bring up PX4/ArduPilot SITL
python3 mavlink_bridge.py --selftest
python3 mavlink_bridge.py --autopilot px4 --mavlink-dest udp:127.0.0.1:14540
```
Confirm in QGroundControl that EKF shows external vision/velocity source.

### Step 4b: Full stack, nav not armed
```bash
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 \
    --platform px4 --mavlink-dest udp:127.0.0.1:14540
```
Confirm all UDP hops and MAVLink bridge alive together.

### Step 4c: Full stack with nav_node active
```bash
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 \
    --platform px4 --mavlink-dest udp:127.0.0.1:14540 \
    --enable-nav --waypoints "0,10,5;10,10,5;10,0,5;0,0,5"
```
Type `go` once state shows `HOLD`. Watch the 10×10m square at 5m altitude.

### Step 4d: Deliberate failure injection

| Test | Action | Expected Result |
|---|---|---|
| Radar loss | Kill `radar_fanout` mid-mission | `MISSION → HOLD → FAILSAFE_RTL` |
| Obstacle | Inject low `fwd_range` | `OBSTACLE_STOP`, velocity zeroed |
| Obstacle clear | Remove synthetic obstacle | `OBSTACLE_STOP → MISSION` resumes |
| Geofence breach | Fly past `max_radius` | `FAILSAFE_RTL` |
| Manual land | Type `land` | Landing command sent |
| Manual RTL | Type `rtl` | RTL command sent |

### Cross-verification:
- Run each failure test **at least twice** — "worked once" is not sufficient
- Check `AttitudeListener` is receiving ATTITUDE messages (visible in nav_node logs)
- Verify attitude rotation: yaw SITL vehicle 90°, dead-reckoned position should
  track correctly (< 1m drift over 10s)

**Gate to Stage 5:** waypoint square flies correctly, ALL failure injection tests
produce correct failsafe states, repeatable across multiple runs.

---

## Stage 5 — Real airframe

### 5a — Bench power-on, props off, full stack, no flight
Wire radar → companion → flight controller. Run full stack against real FC with
props off. Confirm arming/mode-switch commands reach real autopilot.

### 5b — Tethered, low altitude, radar aiding logged but not authoritative
Vehicle on a physical tether/safety line, GPS still primary, radar pipeline
running and logged as passive comparison. No `nav_node.py` mission — this
validates that vibration/motor RF doesn't break the radar pipeline.

### 5c — Tethered, nav_node running a short mission, GPS backstop available
Run actual small waypoint square with `nav_node.py` in control, tethered, low
altitude, GPS in background for safety pilot to compare.

```bash
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 \
    --platform px4 --mavlink-dest <real-link> \
    --enable-nav --waypoints "0,5,3;5,5,3;5,0,3;0,0,3" \
    --max-speed 1.5
```

### 5d — Untethered, short hops, low altitude, GPS-denied for real
Only after 5c shows consistent, repeatable, bounded-error behavior across at
least two independent runs.

### At every sub-stage:
- Pre-briefed "disable radar aiding / switch to GPS or manual" procedure
- Safety pilot can execute instantly
- No progression until the previous sub-stage is repeatable (not "worked once")

---

## Quick Reference: Complete Command Sequence

```bash
# Stage 0: Run offline checks
cd "/home/alok/radar/testing RIO+3d"
python3 sitl_test.py --plumbing-only

# Stage 1: Bench static
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 --no-mavlink

# Stage 4a: MAVLink plumbing (needs SITL running)
python3 mavlink_bridge.py --selftest
python3 mavlink_bridge.py --autopilot px4 --mavlink-dest udp:127.0.0.1:14540

# Stage 4c: Full mission in SITL
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 \
    --platform px4 --mavlink-dest udp:127.0.0.1:14540 \
    --enable-nav --waypoints "0,10,5;10,10,5;10,0,5;0,0,5"

# Stage 5d: Real GPS-denied flight
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 \
    --platform px4 --mavlink-dest <real-MAVLink-link> \
    --enable-nav --waypoints "0,10,5;10,10,5;10,0,5;0,0,5" \
    --max-speed 2.0
```

---

## UDP Port Map (after Phase A fix)

| Port | Producer | Consumer | Data |
|---|---|---|---|
| 5005 | `radar_fanout.py` | `doppler_rio.py` | Raw radar frames (x,y,z,v) at 20 Hz |
| 5006 | `doppler_rio.py` | `slam_node.py` | RIO velocity (t,vx,vy,vz,n_inliers) |
| 5007 | `doppler_rio.py` | `mavlink_bridge.py` | RIO velocity (same format) |
| 5008 | `doppler_rio.py` | `nav_node.py` | RIO velocity (same format) |
| 5010 | `radar_fanout.py` | `slam_node.py` | Decimated radar frames (~5 Hz) |
| 5011 | `slam_node.py` | `nav_node.py` | Pose + fwd_range (t, n_map, fwd_range, T_4x4) |
