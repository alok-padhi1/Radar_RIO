# From Bench Visualizer to Autonomous GPS-Denied Navigation

Straight answer first: **what you're looking at in the screenshot (`radar_live_map`
raw scatter) is not the rendering tool you need — it's the stage before filtering.**
The rainbow scatter is exactly the raw arc/clutter problem from `PIPELINE_NOTES.md`.
The fix isn't a different visualizer, it's switching to the filtered pipeline
(`slam_node.py`, which already does the rendering-equivalent job of building a clean
accumulated map) that you already have from the last two deliveries. You don't need
a new tool — you need to run the pieces you already have, together, instead of the
old standalone `radar_live_map`/`radar_3d_viewer` script.

## 1. Combining the threads — this is `radar_fanout.py` + `supervisor.py`

You already have this; it just needs to be run as one stack instead of separate
manually-launched scripts. New file **`supervisor.py`** launches everything in the
right order, tags every line of output by process name, and takes the whole stack
down if a critical component (fanout, RIO, SLAM) dies rather than limping along on
stale data:

```bash
# Bench/Phase 1-2: radar + RIO + SLAM only, no autopilot involved yet.
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 --no-mavlink
```

This is the direct answer to "process it using some rendering tool" — `slam_node.py`
*is* that tool now: it consumes the fanned-out points, runs the full filter funnel
from `PIPELINE_NOTES.md`, does ground-plane split + adaptive-covariance GICP, and logs
`raw → final` point counts and fitness per keyframe. Watch that log, not a raw scatter
plot, to judge whether the pipeline is healthy.

## 2. What "autonomous navigation in GPS-denied environment" actually requires

Four layers, each depending on the one below it. You have layers 1–2 built; this
delivery adds 3–4:

| Layer | Role | File |
|---|---|---|
| 4. Mission/waypoint logic | "fly to these points, in this order, stop safely if something's wrong" | `nav_node.py` *(new)* |
| 3. Position estimate | "where am I right now, in a local frame with no GPS" | `PoseTracker` inside `nav_node.py` *(new)*, fed by SLAM pose + RIO velocity |
| 2. Ego-velocity + local map | "how fast am I moving, what's around me" | `doppler_rio.py`, `slam_node.py` (already built) |
| 1. Raw sensor stream | "clean, fanned-out point data" | `radar_fanout.py`, `filters.py` (already built) |

**`nav_node.py`** is the new piece: it's a state machine (`INIT → WAIT_FOR_POSE →
HOLD → MISSION → OBSTACLE_STOP / FAILSAFE_RTL`) that reads SLAM pose + a forward-cone
obstacle range (new field `slam_node.py` now emits on the pose packet — see below),
does proportional guidance toward each waypoint capped at `--max-speed`, scales/zeroes
the forward velocity component reactively when the obstacle range gets close, and
sends `SET_POSITION_TARGET_LOCAL_NED` velocity setpoints to PX4 (OFFBOARD) or
ArduPilot (GUIDED) over MAVLink.

### What changed in `slam_node.py` for this delivery
It now computes a cheap forward-cone minimum range (±20° around body +x, any
filtered point — ground or off-plane) every keyframe and includes it in the pose UDP
packet. This is deliberately separate from the accumulated map: it's the fast,
low-latency signal `nav_node.py` uses for reactive braking, while the map remains the
slower path for anything more sophisticated you build later (path planning around
mapped obstacles, not just "stop if something's close ahead").

### Be honest about what `PoseTracker` is and isn't
It fuses SLAM's ~5 Hz pose with RIO's 20 Hz velocity by simple dead-reckoning
between SLAM updates — **not** the monograph's 15-state EKF. That's a real
simplification, called out directly in the code's docstring: it assumes RIO's
body-frame velocity is already expressed in the nav frame, which is only true if
you're also tracking attitude (yaw) from somewhere. Wiring the flight controller's
own `ATTITUDE`/`AHRS2` MAVLink messages in to rotate RIO's velocity correctly is the
next real piece of work before this is flight-trustworthy — flagged in the code, not
hidden. Building the full EKF from Section 1.6 of your monograph is the correct
long-term replacement; that's a separate, substantial task from "get the navigation
loop wired end-to-end," which is what this delivery does.

## 3. Complete code delivered this round

- **`nav_node.py`** — waypoint state machine, MAVLink arm/mode/velocity-setpoint
  interface, reactive obstacle braking, geofence + pose-staleness failsafes.
- **`supervisor.py`** — single-command launcher/monitor for the whole stack.
- **`slam_node.py`** (updated) — now emits forward-obstacle-range alongside pose.

## 4. Setup — bench through SITL through real flight, in that order

**Step 1 — confirm the filtered pipeline is healthy (repeat of Phase 1/2 gates).**
```bash
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 --no-mavlink
```
Watch for steady `[slam] keyframe OK ... fitness=...` lines with a sane
`raw → final` ratio and `n_ground` dominating `n_off_plane` on flat bench terrain.
Do not proceed until this is stable — everything above depends on it.

**Step 2 — PX4 or ArduPilot SITL, no radar yet, confirm MAVLink plumbing.**
```bash
# separate terminal: bring up PX4 SITL (or ArduPilot SITL) per its own docs
python3 mavlink_bridge.py --selftest        # message-encoding sanity check, no link needed
python3 mavlink_bridge.py --autopilot px4 --mavlink-dest udp:127.0.0.1:14540
```
Confirm in QGroundControl/MAVProxy that the vehicle sees external velocity aiding.

**Step 3 — full stack against SITL, `nav_node` NOT armed yet.**
```bash
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 \
    --platform px4 --mavlink-dest udp:127.0.0.1:14540
```
Radar can be bench-static here (SITL doesn't care that the real vehicle isn't
moving) — the goal is purely to confirm every UDP hop and the MAVLink bridge are
alive together before nav_node enters the picture.

**Step 4 — SITL with `nav_node` active, extensively, before any hardware.**
```bash
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 \
    --platform px4 --mavlink-dest udp:127.0.0.1:14540 \
    --enable-nav --waypoints "0,10,5;10,10,5;10,0,5;0,0,5"
```
In the `nav_node` terminal, type `go` once state reaches `HOLD` to arm and start the
mission. Watch it fly the square in SITL. Deliberately inject failures here — kill
the radar_fanout process mid-mission and confirm `nav_node` drops to `HOLD` then
`FAILSAFE_RTL`, not silently to zero velocity forever; walk the simulated vehicle
into a simulated obstacle and confirm `OBSTACLE_STOP` triggers.

**Step 5 — real airframe, tethered, safety pilot on the sticks, per
`ARCHITECTURE.md`'s Test Stage 5.** Nothing in this delivery changes that
progression — `nav_node.py` is what Stage 5 flies, once Stages 1–4 have already
passed on real (non-SITL) radar data and the autopilot's own failsafes/geofence are
independently configured and tested.

## 5. Non-negotiable safety notes (repeating because this is the layer that arms/flies)

- This script **arms the vehicle and sends control setpoints** — a different trust
  level than everything delivered before it, which deliberately never did that.
- Configure the autopilot's **own** geofence, RC-loss failsafe, and battery failsafe
  independently — `nav_node.py`'s internal checks are a second layer, not a
  replacement.
- Always fly with a safety pilot able to retake manual control by mode switch.
- SITL-validate every change to `nav_node.py`'s control logic before it goes near a
  real airframe.
- This project does not know or enforce your local airspace regulations — that's on
  you before any real flight.
