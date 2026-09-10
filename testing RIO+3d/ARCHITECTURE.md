# Dual-Thread RIO-SLAM Architecture — Linpowave U300, Jetson Orin Nano
Companion-computer implementation plan for 100–200 m AGL GPS-denied flight (PX4/ArduPilot).

Builds directly on what you already have working: `radar_streamer.py` (UART→UDP),
`radar_3d_viewer.py` (Open3D tilt visualization), and the monograph's tilt math
(`R_R^B(θtilt)`, Doppler-RANSAC, WLSQ ego-velocity). This plan replaces the single UDP
fan-out with a proper multi-consumer architecture and adds the missing SLAM back-end
and MAVLink bridge wiring.

---

## 1. Multi-Threading Architecture — one UART stream, two consumers, non-blocking

**Problem with the current setup:** `radar_streamer.py` owns the serial port and blasts
every frame to one UDP port. If you point both a RIO process and a SLAM process at that
same port, you get non-deterministic packet delivery (UDP doesn't fan out to two
`recvfrom()` binds on the same port/interface reliably) and no back-pressure control —
if SLAM's GICP call takes 150 ms, frames pile up in the OS socket buffer and RIO starts
seeing stale-but-not-dropped data.

**Fix:** one process owns the serial port. It parses frames on a dedicated I/O thread
and fans them out to two `queue.Queue` instances feeding two independent worker
threads. No process ever calls `ser.read()` except the reader thread.

```
                 ┌─────────────────────────────────────────────┐
                 │              radar_fanout.py (1 process)      │
                 │                                                │
  UART 921600 ──▶│  SerialReaderThread                            │
                 │    - owns ser.read(), parses magic word/TLV1   │
                 │    - timestamps each frame (monotonic)         │
                 │    - pushes to BOTH queues, non-blocking        │
                 │        rio_q  (maxsize=2, drop-oldest)          │
                 │        slam_q (maxsize=1, drop-oldest, 1-in-4)  │
                 │              │                    │             │
                 │       ┌──────▼──────┐      ┌──────▼──────┐     │
                 │       │ RIOWorker    │      │ SLAMWorker   │     │
                 │       │ thread       │      │ thread       │     │
                 │       │ 20 Hz target │      │ 1-5 Hz target│     │
                 │       │ Doppler-     │      │ GICP/NDT +   │     │
                 │       │ RANSAC+WLSQ  │      │ pose graph   │     │
                 │       └──────┬───────┘      └──────┬───────┘     │
                 │              │                      │            │
                 │      UDP :5006 (v_body)      UDP :5007 (pose)    │
                 └──────────────┼──────────────────────┼────────────┘
                                 ▼                      ▼
                      mavlink_bridge.py          map/pose consumer
                      → PX4 ODOMETRY /            (Open3D live view,
                        ArduPilot VISION_SPEED     logging, RViz2, etc.)
```

Key decisions and why:

- **One process, one serial handle.** Avoids the classic "two scripts fight over
  `/dev/ttyUSB0`" bug. Everything else is threads, not processes, because both workers
  are I/O/numpy-bound, not CPU-bound Python loops — the GIL is released during
  `numpy`/`open3d` C calls, so this scales fine on a 6-core Orin Nano without needing
  multiprocessing/shared memory.
- **Different queue depths and drop policy per consumer, by design:**
  - `rio_q`: `maxsize=2`, and the reader does a non-blocking `put_nowait`, catching
    `queue.Full` and popping the oldest before pushing. RIO must always run on the
    *freshest* frame — a stale velocity aiding PX4's EKF is worse than a dropped one.
  - `slam_q`: reader only enqueues **every 4th frame** (20 Hz → 5 Hz) *before* the
    queue, not after — this is a deliberate decimation, not backpressure, because you
    want SLAM keyframes evenly spaced in time, not "whatever was left after RIO ate
    the queue." `maxsize=1` with drop-oldest means if GICP is still chewing on the
    previous keyframe, you skip a keyframe rather than queue up a backlog that makes
    the map trail reality.
- **Frames are timestamped once, at parse time, by the reader thread**, and that same
  timestamp rides with the frame into both queues. This is what lets you fuse
  RIO-derived velocity with the SLAM pose graph's timestamps later without a second
  clock to reconcile.
- **Cross-thread communication is UDP loopback, not a shared Python object,** between
  the RIO/SLAM workers and their downstream consumers (`mavlink_bridge.py`, a
  visualizer). This is intentional: it keeps `mavlink_bridge.py` and any RViz2/Open3D
  viewer as fully separate, restartable processes — you can bounce the MAVLink bridge
  without touching the radar pipeline, exactly as your current `doppler_rio.py` →
  `mavlink_bridge.py` split already does.
- **Jetson Orin Nano thread/core budget:** pin `SerialReaderThread` and `RIOWorker`
  with `os.sched_setaffinity` (or a `nice -n -5`) to a fast core pair if you see UART
  read jitter under SLAM load; GICP is the only thread worth letting the scheduler
  roam freely across the remaining cores.

---

## 2. Doppler-RANSAC Velocity Solver — implementation strategy

Already implemented and self-tested in `doppler_rio.py` from your working set — the
strategy below documents *why* it's built that way, since that's the part reviewers
(and future-you debugging a bad estimate in the field) will need.

1. **Per-frame pipeline:** parse `(x,y,z,v)` in radar frame → rotate+translate to body
   frame via `Eq.(2)` (`P^B = R_R^B(θtilt) P^R + t^B_R`) → build unit LOS vectors
   `u_i = P^B_i / ‖P^B_i‖` → range-gate `[0.3 m, 350 m]` → RANSAC → weighted refit.
2. **RANSAC minimal set = 3 points**, solving the 3×3 linear system
   `-u_i^T v = V_i` exactly (not least-squares) per hypothesis — this is what makes
   each of the 60 iterations cheap enough to run at 20 Hz on an embedded core.
   Degenerate (near-coplanar) samples are caught via `LinAlgError` and skipped, not
   silently accepted with a garbage solution.
3. **Inlier test** `|V_i + u_i^T v_k| < ε` with `ε ≈ 0.15 m/s` as a starting point —
   tune this against your radar's stated Doppler resolution, not against how good the
   fit "feels"; too tight and you throw away good ground returns in the intermittent
   350 m-edge regime, too loose and slow-moving foliage/other traffic sneaks into the
   inlier set.
4. **Reject, don't force, low-confidence frames.** `min_inlier_ratio ≈ 0.35`: below
   that, the frame returns `valid=False` rather than a low-confidence velocity. This
   is a direct architectural consequence of Sec. 1.3/3.4 of the monograph — an
   intermittent-illumination gap is expected, and the correct response is "the EKF
   coasts/widens covariance," not "RIO reports something anyway."
5. **Weighted refit, not simple least-squares**, with `w_i ∝ 1/R_i²` (see the
   `doppler_rio.py` docstring — your firmware's TLV type 1 has no per-point SNR field,
   so the monograph's `SNR/R²` weight degrades cleanly to `1/R²` when SNR isn't
   available; wire in true SNR if a Side-Info TLV becomes available). This down-weights
   noisy far-range ground returns near the 350 m edge without discarding them outright.
6. **Emit velocity + full 3×3 covariance**, not velocity alone. `cov = (AᵀWA)⁻¹` is
   the thing that lets the downstream EKF (Sec. 1.6 of the monograph) correctly widen
   whichever axis the tilted-cone geometry observed weakly *that frame* — collapsing
   this to a single scalar "confidence" throws away exactly the information a
   real EKF needs.

---

## 3. Dense SLAM Back-End — Adaptive/GICP or 4D-NDT on sparse radar clouds

Radar point clouds are 1–2 orders of magnitude sparser than LiDAR — 30–150 points per
20 Hz frame is typical for the U300 in a ground-dominated tilted-mount footprint. Naive
GICP tuned for LiDAR density will either reject every scan (too few correspondences)
or silently produce garbage (correspondences forced through noise). Strategy:

1. **Keyframe accumulation before registration, not scan-to-scan.** Register a
   *keyframe* built by accumulating ~4–8 consecutive raw frames (already
   body-frame-transformed) into one denser local cloud, motion-compensated using the
   RIO velocity output from the same window (cheap: you already have `v_body` at
   20 Hz from Section 2). This is the single highest-leverage fix for sparse-radar
   SLAM — it turns "30 points, mostly noise" into "150–500 points, real structure"
   before GICP ever sees it, at the cost of the ~0.2–0.4 s the keyframe covers.
2. **Adaptive per-point covariance from k-NN**, per `Eq.(21)` of the monograph:
   estimate each point's local covariance from its k=5–10 nearest neighbors *within
   the keyframe*, not from a fixed isotropic sensor-noise model. On a ground-dominated
   footprint this naturally produces flat (planar) covariances for the dominant ground
   returns and more isotropic covariances for the sparse off-plane structure (poles,
   tree trunks, building corners) — exactly the points you want GICP leaning on for
   in-plane constraint, since the ground plane itself only constrains the vertical
   axis (the "ground-plane degeneracy" failure mode in Sec. 3.4).
3. **Ground-plane segmentation as a pre-filter (Sec. 2.3), before GICP.** RANSAC-fit
   the dominant plane per keyframe; keep it as an explicit planar factor for the
   vertical axis, and hand GICP the *off-plane residual points preferentially
   weighted* for the in-plane (x,y) constraint. This directly addresses the planar
   degeneracy failure mode rather than hoping GICP's covariance weighting alone
   handles it.
4. **Correspondence search radius wide, correspondence count small.** With
   Open3D's `registration_generalized_icp`, set `max_correspondence_distance` to
   2–4× your keyframe's typical nearest-neighbor spacing (radar point clouds are not
   locally dense the way LiDAR is — a tight radius just means "0 correspondences,
   registration silently no-ops"). Reject a keyframe registration outright if fewer
   than ~15 correspondences converge, same "reject, don't force" philosophy as RIO.
5. **Doppler-consistency term, `Eq.(23)`, as a soft prior, not a hard gate.** Because
   you already compute `v_body` in the RIO thread every 20 Hz frame, folding the mean
   `v_body` over the keyframe window into the GICP cost as
   `-λ_v Σ (V_i + u_i^T v̂_body)²` is nearly free and disambiguates the in-plane
   translation exactly where the ground-plane-degeneracy problem above is weakest —
   this is the one piece of "4D" (using the velocity channel) that a naive
   XYZ-only GICP port from LiDAR SLAM would miss entirely.
6. **Loop closure, staged by risk, not "build GTSAM day one":**
   - **Stage A (what ships in `slam_node.py` below):** pure incremental
     keyframe-to-local-submap GICP, odometry-only, no loop closure. This alone gives
     you a locally-consistent, drift-corrected-relative-to-RIO map — sufficient for
     Stage 1–3 of the test plan below and for feeding PX4 as a smoothed
     `VISION_POSITION_ESTIMATE` once you trust it.
   - **Stage B (production upgrade path):** add Scan-Context-style descriptor loop
     candidates + `iSAM2`/GTSAM back-end exactly as the monograph's `Eq.(24)`
     specifies, in C++ for the compute budget. Do this only after Stage A odometry is
     flight-validated — loop closure debugging on top of an unvalidated front-end is
     how you get a map that's wrong in an interesting new way instead of just drifting.

---

## 4. MAVLink Encoding Strategy — avoiding EKF rejection

Already implemented in `mavlink_bridge.py`; the rules that prevent silent EKF rejection:

| Autopilot | Message | Frame fields | Fusion-side setting |
|---|---|---|---|
| PX4 (EKF2) | `ODOMETRY` (#331) | `frame_id=LOCAL_NED`, `child_frame_id=BODY_FRD`, position/attitude/angular-rate fields **NaN** until you trust SLAM's pose | `EKF2_EV_CTRL` bit for velocity, `EKF2_EV_DELAY` set to your measured pipeline latency |
| ArduPilot (EKF3) | `VISION_SPEED_ESTIMATE` (#103) | velocity-only | `EK3_SRC1_VELXY = 6` (ExternalNav) — **`ODOMETRY` is silently ignored by ArduPilot**, not an error, so this is the single most common "why isn't it fusing" bug |
| Both | `DISTANCE_SENSOR` (#132) | `orientation=MAV_SENSOR_ROTATION_PITCH_270`, `type=MAV_DISTANCE_SENSOR_RADAR` | n/a — this is a direct rangefinder input, not EKF-source-selected |

Additional rules baked into the implementation, worth restating so they don't get
"simplified away" later:

- **Send velocity-only `ODOMETRY`/`VISION_SPEED_ESTIMATE` from the RIO thread at
  ~20 Hz; only add `VISION_POSITION_ESTIMATE` (#102) once the SLAM back-end's pose is
  flight-validated** (Stage 4+ of the test plan). Fusing an unvalidated absolute pose
  into position aiding is a much worse failure than fusing a slightly-off velocity —
  velocity errors get bounded by the EKF's own process model; a bad jump in fused
  *position* can produce a real flight-control response.
- **Populate the velocity covariance diagonal from `doppler_rio.py`'s real
  `(AᵀWA)⁻¹`, not a fixed placeholder.** A constant covariance means the EKF can't
  down-weight a low-inlier-count frame — exactly the frames the monograph's Sec. 1.3
  says will happen routinely near the 350 m illumination edge.
- **`EKF2_EV_DELAY` / ArduPilot's equivalent must reflect your actual measured
  UART→parse→RANSAC→UDP→MAVLink pipeline latency**, not zero. Measure it (see Stage 2
  below) before first flight; a wrong delay estimate silently degrades fusion quality
  without throwing any error.
- **`self_test()` in `mavlink_bridge.py` packs every message with the real dialect
  encoder with no live link required** — run this in CI/pre-flight, not just once
  during development, since a `pymavlink` version bump changing a field order is
  exactly the kind of bug you want caught on the bench.
- **The bridge never arms, disarms, or sends setpoints.** Keep that boundary through
  every stage below — it only ever pushes aiding data into a source the vehicle's own
  EKF already knows how to sanity-check, reject, or blend down-weighted.

---

## 5. Progressive 5-Stage Physical Testing Plan

Each stage has a single new variable versus the last, and an explicit
pass/fail gate before moving on — the point is to isolate whether a problem is in the
radar/parsing, the RIO math, the SLAM front-end, or the MAVLink/EKF fusion, rather than
debugging all four at once in the air.

**Stage 1 — Handheld walking, tethered laptop, no autopilot in the loop.**
Radar + Orin on a chest rig or handheld boom, tilted at the target 35–40°, walking
straight lines and figure-eights outdoors. Run `radar_fanout.py` + `doppler_rio.py`
logging to disk (no MAVLink yet). *Gate:* integrated RIO velocity over a measured
50–100 m straight walk matches paced/GPS-phone ground truth to within ~5%; inlier
ratio stays above `min_inlier_ratio` for >90% of frames outdoors on natural
ground/vegetation.

**Stage 2 — Handheld walking, SLAM front-end live, latency measurement.**
Add `slam_node.py` consuming the decimated queue; run the Open3D live viewer from your
existing `radar_3d_viewer.py` pattern against the accumulated keyframe map instead of
raw points. Instrument and log end-to-end latency (UART frame timestamp → MAVLink
packet send timestamp) for both the RIO and SLAM paths. *Gate:* keyframe-to-keyframe
GICP converges (correspondence count above threshold) on >80% of keyframes during
walking motion; measured RIO-path latency is stable and low enough to set
`EKF2_EV_DELAY` with confidence (target: <60 ms end-to-end for the velocity path).

**Stage 3 — Vehicle-mounted (car roof rack or similar), no autopilot fusion.**
Same software stack, now at higher sustained speed (5–15 m/s) and longer straight-line
distances than walking allows, to validate the Doppler-RANSAC solver's behavior at
speeds closer to actual flight and to stress-test the 350 m-edge intermittency
behavior from Sec. 1.3 against real terrain. *Gate:* no crashes/exceptions over a
30+ minute continuous run; RIO correctly reports `valid=False` (not a garbage
velocity) during genuine illumination gaps rather than producing a discontinuous
velocity spike.

**Stage 4 — Bench-to-SITL MAVLink fusion validation, vehicle stationary/tethered.**
Run PX4 SITL or a bench-powered real flight controller (props off, or fully tethered
if testing on the real airframe) with `mavlink_bridge.py` live, feeding it *recorded*
Stage 3 velocity/pose logs played back at real-time rate (not live radar yet) so the
fusion behavior is reproducible while you tune `EK3_SRC1_VELXY`/`EKF2_EV_CTRL`,
`EKF2_EV_DELAY`, and covariance scaling. *Gate:* `mavcmd` / QGroundControl EKF status
shows the external vision/nav source accepted and contributing (not just received) —
this is the stage to catch the ArduPilot `ODOMETRY`-silently-ignored class of bug
described in Section 4, on the bench, not in the air.

**Stage 5 — Tethered, then untethered flight, incremental.**
5a: multirotor on a tether/safety line, low altitude (well within visual range, radar
active but flight-critical navigation still primary/GPS), comparing RIO/SLAM output
against the vehicle's own GPS+baro estimate purely as a passive log comparison — no
control authority yet. 5b: short untethered hops at low altitude with radar aiding
*enabled but down-weighted* (conservative covariance scaling) alongside GPS, watching
for EKF innovation warnings. 5c: extend altitude/duration and reduce reliance on GPS
aiding only after 5a/5b show consistent, bounded-error agreement over multiple
flights. *Gate at every sub-stage:* an immediate, pre-briefed "disable external
aiding, revert to GPS/baro-only" procedure, and no progression to the next sub-stage
until the previous one shows repeatable, non-diverging behavior across at least two
independent flights.

---

### File map (this deliverable)

| File | Role | Status |
|---|---|---|
| `radar_streamer.py` | UART→UDP, single consumer | **superseded** by `radar_fanout.py` below |
| `radar_3d_viewer.py` | Open3D raw-point viewer | keep for bench debugging |
| `doppler_rio.py` | Doppler-RANSAC + WLSQ velocity | keep as-is, now fed by `radar_fanout.py`'s `rio_q` via UDP :5005 (unchanged interface) |
| `mavlink_bridge.py` | MAVLink ODOMETRY/VISION_SPEED/DISTANCE_SENSOR | keep as-is |
| `radar_fanout.py` | **new** — single-owner UART reader, threaded fan-out to RIO/SLAM queues | this delivery |
| `slam_node.py` | **new** — keyframe accumulation, adaptive-covariance GICP, incremental pose chain | this delivery |
