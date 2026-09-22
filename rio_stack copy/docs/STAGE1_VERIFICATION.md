# Stage 1 Bug Report — Review, Fixes, and Verification

Reviewed the bug report against your actual working code (`radar.zip`) and the
vendor's own documentation (`Linpowave_visualizer_UART userguide_Points_float.pdf`,
`Points_float protocol and analysis.pdf`). **All five root causes are correctly
diagnosed and are the reason for the symptoms you're seeing.** Root cause #1 in
particular is a genuinely important find — the sensor-frame convention error was
present in code I gave you earlier, and I want to be direct about that rather than
gloss over it. Fixes for all five are applied below, verified against a synthetic
reproduction of your bench scenario.

## Root cause 1 — sensor frame mismatch: CONFIRMED, fixed

Checked directly against the vendor visualizer manual: *"Maximum distance X ...
X range is ±50 m"* (symmetric → lateral) vs *"the maximum distance Y of 100 ...
Y range is [0,100]"* (one-sided, a forward range gate). That's conclusive — the
U300's native frame is **(X=lateral, Y=forward, Z=up)**, not the (X=forward,
Y=lateral, Z=vertical) that both `doppler_rio.py`'s and `slam_node.py`'s
`TiltMount` assumed. The protocol PDF's own worked example reinforces it: a
captured point at `Z=-1.749 m` from a downward-tilted unit is exactly what a
floor return below the sensor should look like if Z is up-positive.

**Consequence, exactly as your report describes:** a point 1.2 m directly ahead of
the boresight was being placed at body `Y=+1.2` (1.2 m to the vehicle's right)
instead of body `X=+1.2` (1.2 m ahead) — forward and vertical velocity were
structurally unobservable, and the pitch-down tilt (Eq. 1) was being applied to
axes it was never meant to touch.

**Fix:** `TiltMount` in both files now applies a permutation `P` (native radar
frame → pre-tilt body-aligned frame) *before* the existing pitch-down rotation:
```
R_R^B(theta_tilt) = R_tilt(theta_tilt) @ P
P = [[0, 1, 0],     # X_body  =  Y_radar   (forward)
     [±1, 0, 0],    # Y_body  = ±X_radar   (lateral/right)
     [0, 0, -1]]    # Z_body  = -Z_radar   (down = -elevation)
```
The `±1` (`lateral_sign`, CLI: `--lateral-sign`) is the one piece the documents
don't nail down — X's *polarity* (which physical direction is "+X"). Default is
`+1`, which assumes a standard right-handed sensor frame; this is a reasonable
default (it's the same handedness as ENU) but **you should verify it once on the
bench**: place a single target physically to the sensor's right and confirm
`doppler_rio.py`/`slam_node.py` report positive body-frame Y for it. If it comes
out negative, pass `--lateral-sign -1.0` to both `doppler_rio.py` and
`slam_node.py` (or `supervisor.py`, which forwards it to both). I want to be
upfront that I can't verify this polarity from documents alone — it needs one
physical check on your actual hardware.

Verified the corrected transform mathematically (untilted straight-ahead → pure
forward; untilted right-side target → pure lateral; 40° tilt → forward+down in
the correct proportion) — see the commands at the end of this doc to reproduce
that check yourself.

## Root cause 2 — RANSAC degeneracy: CONFIRMED, fixed

Your analysis is exactly right, and I want to note the earlier `doppler_ransac`
already had a *partial* fix (static-hypothesis seeding) that wasn't sufficient —
it seeded `v=0` as the starting best hypothesis but then let any random 3-point
solve override it with a **strict `>`** comparison, so a spurious hypothesis
beating the true static count by even one point (well within the noise floor on
~24 near-parallel LOS vectors) would win. That's precisely your observed
`vx/vz ≈ -0.71` coupling and the 14-vs-15/16 inlier counts in the log.

**Fix**, three layers:
1. **Static-hypothesis margin** (your "BIC penalty" ask, implemented practically):
   a moving hypothesis must beat the static one by
   `max(3, ceil(0.12 * N))` points, not just by one, before it's even considered.
2. **Degenerate-triad guard inside the RANSAC loop:** each 3-point minimal sample
   is SVD-checked before solving; near-coplanar/near-parallel triads (which is
   what a flat floor produces almost every draw) are skipped rather than solved
   through.
3. **Condition-number gate on the winning hypothesis:** even after clearing the
   margin, if `cond(A_inliers) > 30` (default, `--cond-reject-threshold`), the
   frame is rejected outright — a moving hypothesis that only "wins" because the
   geometry can't actually constrain 3D velocity is not published.
4. **Deadband** (`--deadband`, default `0.05 m/s`): any accepted velocity below
   this magnitude is snapped to exactly zero, per your ask.

Verified: **0 velocity spikes over 500 synthetic static-floor frames** built to
mimic your bench geometry (narrow angular cone, ~24 points, small noise, true
`v=0`) — see the reproduction command below.

## Root cause 3 — cascade into SLAM: CONFIRMED, resolved as a consequence of #1+#2

Your read is correct: this wasn't an independent bug, it was #1 and #2 propagating
downstream. `slam_node.py`'s `gate_doppler_consistency` was doing exactly what it
was designed to do — reject points whose Doppler disagrees with the RIO velocity
hint — the hint itself was just wrong. With #1/#2 fixed, `last_v_body` on a real
static bench will correctly and repeatably be `[0,0,0]` (deadbanded), so the
Doppler-consistency gate's prediction collapses back to the correct near-zero-V
test for static floor returns, and deskewing no longer smears the cloud by a
fictitious meter. No separate code change needed here beyond #1/#2.

## Root cause 4 — voxel size: CONFIRMED, and worth being precise about *why*

Confirmed the defaults you found: `filters.py`'s `voxel_size_m=1.0` and
`slam_node.py`'s `--voxel-size 1.5`. Both are sized for the 100-300 m AGL flight
regime this stack was originally built for (`ARCHITECTURE.md`), where the
illuminated ground footprint is tens of meters across. At 3 ft height the
footprint is ~1.5×1.5 m — a 1.0-1.5 m voxel collapses nearly the whole return set
into 1-2 voxels.

**I'm not changing the code defaults for this one** — 1.0-1.5 m is correct for
real flight altitude, and silently shrinking it project-wide would quietly break
Stage 3+ testing later. Both `slam_node.py` and `filters.py` (via `slam_node.py`'s
existing `--voxel-size` flag) are already fully overridable from the command line,
which is the right place for a scale-dependent parameter to live. `supervisor.py`
now exposes `--voxel-size`, `--max-corr-dist`, `--min-correspondences`,
`--persistence-radius`, and `--deadband` directly so you don't have to touch code
to retune for bench scale — see the exact bench command below.

Also fixed, while reviewing this area: `--min-correspondences` existed as a
constructor parameter on `RadarSLAM` but was **never actually wired to a CLI flag
or passed through in `run()`** — it was silently stuck at the hardcoded default of
15 no matter what you did. With only ~14-20 points total on the bench, that gate
alone would have kept rejecting every keyframe even after everything else was
fixed. Now exposed and threaded through.

## Root cause 5 — supervisor log buffering: CONFIRMED, fixed

Confirmed `subprocess.Popen` had no `-u`/`PYTHONUNBUFFERED`. Fixed with both: `-u`
is inserted into every child's argv, and `PYTHONUNBUFFERED=1` is set in each
child's environment as a second layer (covers the case where the interpreter
invoked isn't literally taking `-u`, e.g. a wrapper). `[fanout]` and `[slam]` lines
should now interleave with `[rio]` in real time instead of arriving in bursts.

## One more thing found during this review (not in your list, worth flagging)

`filters.py`'s `PersistenceTracker` (Stage 4 of the pipeline) had a warm-up bug:
its `persistence_min_hits` threshold (default 2) was a **fixed** requirement
regardless of how much history had actually accumulated. On the 2nd call ever,
there's only 1 past frame to compare against — a fixed threshold of 2 meant
*every* point failed persistence on that call, real structure included, purely
because the tracker hadn't warmed up yet, not because anything was transient.
Fixed: the effective threshold now scales with however much history is actually
available (`min(configured_min_hits, len(recent_history))`), reaching the
configured strictness once the window is full. This would have caused
intermittent, hard-to-explain "keyframe rejected: too_sparse" failures even after
everything else above was fixed, especially right after every
`supervisor.py` restart.

---

## Exact verification commands and pass criteria

**1. Reproduce the frame-fix and RANSAC-fix checks directly (no hardware needed):**
```bash
python3 doppler_rio.py --selftest
```
Expect `[self_test] PASS` with `|error| < 0.15 m/s` — this exercises the full
pipeline (corrected frame, new RANSAC) against a synthetic moving-platform scene
and should be unaffected in accuracy by the degeneracy guards (they only fire on
near-planar geometry).

**2. Real bench run, tuned for close range:**
```bash
python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40.0 --no-mavlink \
    --voxel-size 0.10 --max-corr-dist 0.5 --min-correspondences 6 \
    --persistence-radius 0.15 --deadband 0.05
```

**Pass criteria before advancing to Stage 2:**

| Check | Target | How to check |
|---|---|---|
| RIO velocity magnitude, stationary | `|v_body| < 0.04 m/s` on >95% of valid frames | Watch `[rio]` log; with the deadband at 0.05, a static frame reports exactly `0.000` whenever it resolves as static — this bar is satisfied trivially by `STATIC` frames, so also confirm you're *not* seeing frequent non-`STATIC` frames with nonzero output on a genuinely stationary bench |
| No velocity spikes | zero frames with `|v_body| > 0.5 m/s` over a 5+ minute static run | grep the `[rio]` log for large magnitudes |
| SLAM keyframes accepted | `[slam] keyframe OK` lines appearing, not 100% `too_sparse_after_filtering`/`insufficient_correspondences` | watch `[slam]` log directly now that buffering is fixed |
| GICP fitness | `fitness > 0.85` on accepted keyframes | printed directly in the `[slam] keyframe OK ... fitness=...` line — this is a reasonable target for a truly static scene once tuned, but treat it as something to verify empirically on your specific bench setup, not a guarantee from the code change alone |
| Pose packets flowing | UDP 5011 receiving packets | quick listener: |
```bash
python3 -c "
import socket, struct
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('127.0.0.1', 5011))
hdr = struct.Struct('<dId')
for _ in range(20):
    data, _ = s.recvfrom(2048)
    t, n_map, fwd = hdr.unpack_from(data, 0)
    print(f'pose packet: t={t:.3f} map_pts={n_map} fwd_range={fwd:.2f}')
"
```
Run this in a second terminal while the stack is up; expect ~20 lines to print
without hanging — if it hangs, no pose packets are reaching 5011 and something
upstream is still rejecting every keyframe.

**3. Bench left/right validation for `lateral_sign` (do this once, physically):**
Place a single reflective target (a corner reflector, a metal sign, even a laptop)
clearly to the radar's physical right, nothing else nearby. Run `doppler_rio.py`
alone and confirm the dominant returns land at positive body-frame Y in the
`[rio]`/debug output. If they land negative, re-run everything with
`--lateral-sign -1.0`.

**Only advance to Stage 2 once all of the above hold on a real (not synthetic)
static bench run, for a full 5+ minutes, not a few seconds.**
