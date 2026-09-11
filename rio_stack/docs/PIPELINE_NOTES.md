# Bench-State Point Cloud: Critique, Filtering Pipeline, Phase 2 Protocol

## 1. Why raw U300 output fails GICP as-is

What you're seeing in the Open3D viewer — arc-like range bins, ghost returns,
static clutter — is three separate problems that look like one blob of noise.
Worth separating them because each has a different fix, and conflating them
leads to over-filtering (throwing away real ground structure) or
under-filtering (feeding GICP garbage it fits with false confidence):

1. **Range-bin arcs are partly *expected*, not a bug, in a static bench
   test.** With the platform stationary, a single frame's detections at a
   given range gate all lie on the surface of a sphere (or, in the tilted
   cone's footprint, an arc of one) centered on the radar — that's the
   FMCW range resolution cell, not clutter. Your own monograph flags this
   directly: *"static testing yields FMCW range-bin artifacts... moving
   flight is required."* You cannot filter this away; it resolves itself
   once there's real ego-motion decorrelating consecutive frames (Phase 2).
   The pipeline below doesn't try to fix this — it just makes sure the
   *other* two problems don't compound it.

2. **Near-field leakage and sidelobe clutter are a hardware/geometry
   artifact.** TX/RX antenna coupling and near-field effects produce
   spurious low-range detections that have nothing to do with the scene.
   These are cheap to remove (a fixed-radius gate) and should be removed
   *first*, before any statistical filtering runs on a distribution that's
   contaminated by them.

3. **Multipath ghosts are the genuinely hard problem, and GICP is
   uniquely bad at tolerating them.** A ground-bounce or wall-bounce
   multi-reflection return reports a *geometrically self-consistent but
   physically wrong* position — it's not high-variance noise GICP can
   average out, it's a plausible-looking point in the wrong place. Worse,
   because GICP weights correspondences by local covariance (Eq. 21), a
   cluster of 3-4 ghost points from a single stable bounce geometry can
   look like a *confident, low-covariance* planar feature — exactly the
   kind of feature GICP trusts most. This is why "just run SOR and call it
   clean" is not sufficient: statistical outlier removal catches sparse,
   isolated noise, not a self-consistent phantom cluster. That's what the
   temporal-persistence stage below is actually for.

Net effect on GICP specifically: `registration_generalized_icp` either (a)
finds too few correspondences within `max_correspondence_distance` because
real structure is buried under clutter density, and the frame gets rejected
by the `min_correspondences` gate you already have — the safe failure mode —
or (b) worse, finds correspondences *between two consistent ghost clusters*
across frames and converges to a confident, wrong transform — the dangerous
failure mode, because nothing downstream flags it as an error. The pipeline
below is built to make (b) as unlikely as possible before geometry ever
reaches GICP, rather than trying to make GICP itself more robust.

## 2. Pipeline design (implemented in `filters.py`)

Six ordered stages, each a rejection filter, applied to every keyframe
before ground-plane split / GICP:

| # | Stage | Removes | Why this order |
|---|---|---|---|
| 1 | Near-field leakage gate (`r < 0.35 m`) | TX/RX coupling artifacts | Must go first — a leakage point sitting at r≈0 skews every downstream statistical estimate (neighbor density, plane fit) if left in. |
| 2 | Range gate (`0.3 m < r < 350 m`) | Out-of-envelope / range-ambiguous returns | Cheap, deterministic, do it before anything that costs O(N log N). |
| 3 | Doppler static-consistency (`|V_i − (−uᵢᵀv̂)| < ε`) | Movers, and multipath ghosts whose bounce-induced Doppler disagrees with any single rigid-body velocity | This is the same static-world model as `doppler_rio.py`'s RANSAC (Eq. 7) — reusing it here means "what RIO trusts" and "what SLAM trusts" don't quietly diverge into two different models of the scene. On the bench with near-zero ego-velocity this becomes a near-zero-Doppler gate — **note carefully: real static ground correctly reads V≈0 here and should pass; this stage removes *movers*, not statics.** |
| 4 | Temporal persistence (needs a point-neighbor within `1.0 m` in ≥2 of the last 4 frames) | Self-consistent-but-transient ghost clusters that stages 1–3 can't distinguish from real structure in a single frame | This is the direct answer to the "confident wrong GICP correspondence" failure mode above — a multipath ghost's apparent position shifts as bounce geometry changes frame-to-frame in a way real static structure doesn't. |
| 5 | Statistical outlier removal (Open3D, k=8, std_ratio=1.5) | Remaining sparse, low-density noise | Runs last among the point-level filters because it's the most expensive and most sensitive to the *input* distribution already being reasonably clean — running SOR first, on raw data, would have its neighbor-density statistics dominated by the leakage/ghost population you're about to remove anyway. |
| 6 | Voxel downsample (`1.0–1.5 m`) | Redundant density, not "bad" points | Purely a compute/uniformity step for GICP, not a rejection stage — kept last and separate for that reason. |

Every stage returns/reports a count, so `slam_node.py` now logs the full
funnel (`raw=… range=… doppler=… persist=… sor=… final=…`) on every rejected
keyframe — when you're tuning thresholds on the bench, you want to see
*which* stage is eating your points, not just "rejected."

Tuning notes specific to the U300 bench setup:

- **`doppler_eps_mps` must stay in sync with `doppler_rio.py`'s RANSAC
  `--eps`.** They don't have to be numerically identical, but if SLAM's
  gate is much tighter than RIO's, you'll see SLAM silently starving on
  frames RIO considers perfectly good — confusing to debug if you don't
  know to check this first.
- **`persistence_min_hits=2` of `persistence_window=4` is deliberately
  lenient**, not strict. At 5 Hz keyframe rate this is roughly "seen in at
  least 2 of the last ~0.8 s" — tight enough to kill single-frame ghosts,
  loose enough not to punish real structure that occasionally drops below
  SNR threshold and reappears next frame. Tighten only after you've
  confirmed on real bench data that it isn't eating real returns.
- **Widen `leakage_radius_m` and `persistence_radius_m` together if you
  see the ground plane itself getting thinned out** — that's a sign the
  gates are tuned for point density you don't have yet (still applicable
  pre-Phase-2, see below).

## 3. Phase 1 → Phase 2 transition protocol

**Do not change more than one variable between the two phases.** The
pipeline above is the only thing that should change going into Phase 2 —
keep `theta_tilt_deg`, mounting, baud rate, and MAVLink wiring exactly as
validated in Phase 1.

1. **Re-baseline the filter funnel on the bench, static, with the new
   pipeline active, before moving.** Run `slam_node.py` with the defaults
   above for 5+ minutes stationary. Record the steady-state
   `raw → final` funnel numbers per keyframe. This is your Phase 1 exit
   baseline — you need it to tell, in Phase 2, whether a drop in point
   count is "the filters are working as motion starts helping" or "the
   filters are now too aggressive for a moving platform."
2. **Confirm the ground plane survives the funnel.** With the platform
   level and stationary over real ground/pavement, `segment_plane`'s
   inlier count (already logged as `n_ground`) should be a large majority
   of `n_final`. If it isn't, loosen `persistence_radius_m` or
   `sor_std_ratio` before touching anything else — a thinned-out ground
   plane is the single most common way this pipeline silently breaks the
   vertical-axis constraint GICP needs (Sec. 3, ground-plane degeneracy).
3. **Move from bench to handheld with the visualizer still running, not
   headless.** Watch the arc artifacts from Sec. 1 point 1 above collapse
   into coherent structure as soon as real translation starts — this is
   the direct visual confirmation that the range-bin-arc problem was a
   static-test artifact and not something the filters need to solve.
4. **Enable the RIO velocity hint (`--rio-port`) only once Phase 2 walking
   starts, not before.** On the bench, `last_v_body` is `None`/zero and
   stage 3's gate is a pure zero-Doppler test; the moment you're walking,
   `v_body_hint` becomes nonzero and the *same* gate now correctly expects
   nonzero radial Doppler on static ground ahead of you. Bringing this up
   too early (e.g. testing it stationary with a stale nonzero hint from a
   previous run) will silently reject good ground returns — restart
   `doppler_rio.py`/`slam_node.py` together at the Phase 1→2 boundary
   rather than leaving Phase 1's process running.
5. **Walk straight lines and figure-eights, log the full funnel + GICP
   fitness/RMSE per keyframe, don't chase a live-tuned "looks clean"
   visual.** This is Test Stage 1/2 from `ARCHITECTURE.md` — the acceptance
   gates there (inlier ratio, correspondence count, latency) are what
   decide "ready for Stage 3," not how the point cloud looks in the
   viewer. A visually sparse-but-correct map beats a visually dense map
   built from over-permissive filters.
6. **Only after Stage 2's gates pass, revisit filter thresholds.** If
   Phase 2 walking data shows the pipeline is over-rejecting real
   structure now that motion provides good angular diversity, that's the
   time to tighten voxel size / loosen persistence — tune against real
   moving data, not bench data, since the two regimes have genuinely
   different noise characteristics (Sec. 1 point 1).
