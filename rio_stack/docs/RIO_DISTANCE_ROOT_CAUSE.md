# RIO Horizontal Distance Under-Reporting — Root Cause Analysis & Fix Plan
### Flight data: 2026-09-18, five flights | Codebase: `radar.zip` as uploaded | Full line-by-line audit

> **Analysis basis:** all 5 JSONL flight logs from 2026-09-18 (6,538 RIO frames, 36,000 IMU samples,
> 35,900 GPS fixes, 13,600 altimeter samples), cross-referenced against a line-by-line read of
> `doppler_rio.py` (858 L), `slam_node.py` (854 L), `nav_node.py` (935 L), `filters.py`,
> `supervisor.py`, `altimeter_bridge.py`, `gps_logger.py`, `radar_fanout.py`.
> Reproduce every number with `docs/rio_forensics.py` (shipped alongside this document).

---

## 0. Verdict

**The distance error is not a scale error. It is a zero-publishing error.**

Across all five flights, **50–74% of every RIO frame published is exactly `(0.0, 0.0, 0.0)`**, and
**77–81% of the distance the aircraft actually flew was covered while RIO was reporting zero
velocity.** On the frames where RIO *does* publish a non-zero velocity, it does not under-report —
it **over**-reports, with a median RIO/GPS speed ratio of **1.12 to 1.45**.

There are three defects, and they are **partially cancelling each other**, which is why the error
looks like a confusing 21–68% scatter rather than a clean bias:

| # | Defect | Contribution | Mechanism |
|---|---|---|---|
| **R1** | `doppler_ransac()` publishes a **fabricated zero** instead of rejecting the frame | **~80% of the loss** | The static hypothesis is accepted on as few as 3 near-zero-Doppler points. In flight those points are almost certainly **airframe self-returns**, not ground. |
| **R2** | The velocity solve **slides along the unobservable Vx/Vz direction** | The "hallucination". Inflates the surviving frames by 1.2–3.3× | At 45° tilt ρ(Vx,Vz) = −0.965. The solver has a near-null direction and nothing constrains it. |
| **R3** | **Signal starvation** — 8–9 usable points at 20–30 m AGL | Sets up R1 and R2 | Ground returns vanish with altitude; the self-returns do not. |

**Nyquist saturation is ruled out.** Evidence in §5.

**Critically: fixing R1 alone makes the system worse, not better.** If you simply stop publishing
zeros and hold the last good velocity, the integrated distance goes from 0.36× to **1.87×** on
flight 12:34:05 and **3.32×** on 14:48:05. The zeros were masking the explosions. **R1 and R2 must
be fixed together.**

---

## 1. What the logs actually say (including three corrections to the stated conditions)

### 1.1 Per-flight summary, computed from the logs

| Flight | Tilt | Duration | RIO rate | **Zero frames** | GPS path | RIO path | **Ratio** | Stated error |
|---|---|---|---|---|---|---|---|---|
| `12:34:05` | 45° | 163.0 s | 9.7 Hz | **72.5%** | 168.6 m | 61.4 m | **0.364** | 68.0% ✓ |
| `12:36:56` | 45° | — | — | **73.5%** | 202.5 m | 75.9 m | **0.375** | *(not listed)* |
| `12:39:57` | 45° | 123.4 s | 9.1 Hz | **50.0%** | 146.4 m | 118.5 m | **0.809** | 21.2% ✓ |
| `14:48:05` | 25° | 138.1 s | 9.6 Hz | **63.6%** | 148.5 m | 79.6 m | **0.536** | 55.0% ✓ |
| `14:50:32` | 25° | — | — | **64.8%** | 177.2 m | 142.1 m | **0.802** | *(not listed)* |

My independently computed ratios reproduce your stated errors to within 1–2 points, so we are
looking at the same phenomenon.

### 1.2 The single most important table in this document

**Where the distance went, flight by flight:**

| Flight | Distance flown **while RIO said exactly zero** | Distance lost to scale under-report on non-zero frames | Distance lost in publish gaps |
|---|---|---|---|
| `12:34:05` | **136.5 m (81.0% of path)** | **−0.6 m (i.e. over-reported)** | 3.0 m |
| `12:39:57` | **73.3 m (50.1% of path)** | **−15.2 m (over-reported by 10%)** | 25.2 m |
| `14:48:05` | **114.7 m (77.2% of path)** | **−7.1 m (over-reported)** | 13.0 m |

> The "scale" column is **negative in every flight.** There is no under-reporting of speed.
> When RIO produces a number, that number is too big, not too small. The entire deficit is
> the time spent publishing zero.

### 1.3 Corrections to the stated flight conditions

Three of the conditions in your brief do not match the logged data, and two of them change the
analysis materially:

| Flight | You stated | Logged altimeter (median) | Logged GPS-ENU up (median) | Comment |
|---|---|---|---|---|
| `12:34:05` | "20 m AGL" | **30.6 m** | **29.2 m** | 🔴 **You flew at 30 m, not 20 m.** Both sensors agree. |
| `12:39:57` | "14 m AGL" | 16.2 m | 15.8 m | ✓ close enough |
| `14:48:05` | "**1–3 m AGL**" | **20.1 m** | **19.1 m** | 🔴 **You flew at ~20 m, not 1–3 m.** Both sensors agree. |

| Flight | You stated max true speed | GPS max (median-15 filtered) | GPS max (raw) |
|---|---|---|---|
| `12:34:05` | 8.1 m/s | 8.09 m/s | 8.81 m/s | ✓ |
| `12:39:57` | **21.4 m/s** | **6.37 m/s** | **6.73 m/s** | 🔴 **The 21.4 m/s figure is not in the GPS data.** RIO's own max output for that flight was 14.89 m/s. The 21.4 is almost certainly a RIO hallucination that was mistaken for ground truth. |
| `14:48:05` | 9.5 m/s | 9.97 m/s | 15.26 m/s | ✓ |

**Why the 14:48 correction matters most:** you attributed that flight's 55% error to low-altitude
operation. It was not low altitude. It was a 20 m flight at 25° tilt — which means the slant range
to the ground was `20/sin(25°) = 47 m` at boresight and `20/sin(13°) = 89 m` at the far beam edge.
That is a much harsher range budget than 45° tilt at the same height (`20/sin(45°) = 28 m`), and it
explains why the shallower tilt did *not* help.

### 1.4 The altitude ↔ failure relationship is monotonic and clean

| Flight | Median AGL | Zero-frame % |
|---|---|---|
| `12:39:57` | 16.2 m | **50.0%** |
| `14:48:05` | 20.1 m | 63.6% |
| `14:50:32` | 23.9 m | 64.8% |
| `12:36:56` | 25.6 m | 73.5% |
| `12:34:05` | 30.6 m | **72.5%** |

And within a single flight (`12:34:05`), binned by the altimeter:

| AGL band | Frames | Mean inliers | Zero % |
|---|---|---|---|
| 3–6 m | 80 | **62.5** | **8.8%** |
| 6–10 m | 97 | 43.1 | 15.5% |
| 10–15 m | 99 | 32.6 | 12.1% |
| 15–20 m | 68 | 17.3 | 23.5% |
| 20–30 m | 175 | 10.2 | 74.3% |
| 30–60 m | 847 | **9.1** | **95.9%** |

**A 6.9× collapse in point count between 5 m and 30 m AGL, and above 30 m the system is
effectively dead — 96% of frames publish zero.** Flight 12:34:05 spent 847 of its 1,583 frames
(53%) in that dead band.

---

## 2. ROOT CAUSE R1 — The fabricated zero (≈80% of the distance loss)

### 2.1 The code path

`doppler_rio.py`, `doppler_ransac()`, lines 269–336. Two separate branches return a **confident
zero velocity** rather than rejecting the frame:

**Branch A — the low-count guard, lines 283–286:**
```python
    if n < 8:
        if (score_zero / n >= min_inlier_ratio) or score_zero >= 3:
            return mask_zero, True, 1.0     # <-- STATIC. v = (0,0,0). Published.
        return None
```
With `min_inlier_ratio = 0.25`, a frame with **7 points of which 3 read |v_radial| < eps** is
declared static. The moving hypothesis is never even attempted.

**Branch B — the static-margin fallback, lines 316–324:**
```python
    margin = max(static_margin_min, int(math.ceil(static_margin_frac * n)))
    static_ok = (n > 0) and (score_zero / n >= min_inlier_ratio)
    if best_mask is None or best_score <= score_zero + margin:
        if static_ok:
            return mask_zero, True, 1.0     # <-- STATIC. v = (0,0,0). Published.
        return None
```
With `n = 12`, `margin = max(3, ceil(0.12 × 12)) = 3`. The moving hypothesis must explain **4 more
points** than the static one. With 12 points and a narrow LOS cone, it frequently cannot.

And then in `process_frame()`, lines 582–588:
```python
        if is_static:
            v_body = np.zeros(3)
            cov = np.eye(3) * 0.25
```
This is forwarded verbatim at line 710 into `FORWARD_PKT` and consumed by `slam_node`, `nav_node`,
`mavlink_bridge`, and `gps_logger` as a **measurement**, not as a gap.

> ⚠️ Note: raising the static covariance from `1e-4` to `0.25` (which was done since the last
> review) protects the EKF from *weighting* the false zero too heavily. **It does nothing for the
> distance integration**, because `nav_node.PoseTracker.on_rio_velocity()` and every offline
> integration use `v`, not `cov`. The covariance fix and this fix are orthogonal; you need both.

### 2.2 Proof from the data — the inlier histogram

Here is the distribution of `n_inliers` **on the frames that published exact zero**:

| Flight | ≤3 | 4 | 5 | 6 | 7 | **8–9** | 10–14 | 15–24 | ≥25 |
|---|---|---|---|---|---|---|---|---|---|
| `12:34:05` | 0% | 0% | 0% | 9% | 6% | **66%** | 11% | 3% | 4% |
| `12:36:56` | 0% | 0% | 2% | 7% | 13% | **66%** | 7% | 3% | 2% |
| `12:39:57` | 0% | 0% | 0% | 7% | 5% | **42%** | 22% | 3% | 21% |
| `14:48:05` | 0% | 0% | 6% | 13% | 4% | **48%** | 13% | 9% | 8% |
| `14:50:32` | 0% | 1% | 5% | 21% | 10% | **41%** | 15% | 4% | 4% |

And the complementary view:

```
inliers <  8  ->  100.0% of those frames are exact zero   (all five flights)
inliers < 10  ->  100.0% of those frames are exact zero   (all five flights)
inliers < 15  ->   88.8% - 95.2% exact zero
```

**Every single frame with fewer than 10 inliers published (0,0,0). Without exception. In every
flight.** That is Branch A and Branch B firing, and nothing else.

### 2.3 The deeper finding: those 8–9 points are almost certainly your own airframe

This is the part that matters for the fix, and it is not obvious from the code.

In the static branch, the reported `n_inliers` is `mask_zero.sum()` — **the number of points whose
radial velocity is within `eps` of zero.** The histogram above shows that number clustering tightly
at **8–9, frame after frame, across 163 seconds of flight, across five separate flights, at two
different mount angles.**

Ask what physical scatterer produces a near-zero radial velocity *while the aircraft is moving at
5 m/s*. For a genuine ground return, `V_i = −u_i·v`, and at 45° tilt with the beam spanning 33–57°
depression and ±40° azimuth, **there is no direction in the beam for which `u·v ≈ 0`** when `v` is
predominantly forward. A real ground point cannot read zero Doppler during forward flight.

Something co-moving with the sensor can. Candidates:

- landing gear legs, battery straps, or payload inside the ±40° azimuth fan,
- the rotor **hub** (the blade tips are ±50–100 m/s and get rejected, but the hub is at ~0),
- TX/RX near-field coupling and antenna-mount reflections surviving the `leakage_radius_m = 0.35 m`
  gate,
- radome standing waves.

A fixed population of self-returns gives you a **permanently available, perfectly self-consistent
"static" hypothesis that always scores ≥3 and never goes away.** As soon as the real ground returns
thin out at altitude, that hypothesis wins the margin comparison and RIO publishes zero.

This is why the failure is so strongly altitude-dependent: the ground signal decays, the self-return
signal does not.

> 📌 **DIAG-01 (do this first, it takes 15 minutes).** Aircraft restrained on the ground, props
> spinning at 30% throttle. Log raw `Points_float` frames directly from `radar_fanout.py` (before
> any filtering). Histogram `range` and `|v|`. **Any cluster with `|v| < 0.15 m/s` at a stable
> range is a self-return.** Record its maximum range `R_self`. Then set
> `--leakage-radius = R_self + 0.2 m`. Repeat with props stopped and compare — the difference tells
> you how much of it is rotor-related.
>
> Run it a second time with the aircraft hand-carried at a brisk walk in a clear area: any point
> still reading `|v| < 0.15` is definitively a self-return, because nothing in a static world can.

### 2.4 The fix for R1

Three changes, all in `doppler_ransac()`:

**(a) A truly static world produces near-zero Doppler on *almost all* returns, not 25% of them.**
`min_inlier_ratio = 0.25` is appropriate for the *moving* hypothesis (where outliers and movers are
expected) but is far too lenient for the *static* hypothesis. If the vehicle is genuinely stationary,
essentially every static-world return should satisfy `|v_radial| < eps`. Gate static separately and
much harder.

**(b) Never accept static on a handful of points.** Require a floor on the absolute count, not just
a ratio.

**(c) While airborne, prefer a declared gap over a fabricated measurement.** A gap widens the EKF
covariance and the estimator coasts on the IMU, which is correct behaviour. A fabricated zero is a
high-confidence lie that the estimator cannot detect.

```python
# doppler_rio.py — REPLACE lines 269-336 signature + static logic

def doppler_ransac(u_body, v_radial,
                   eps=0.15, iters=80,
                   min_inlier_ratio=0.35, max_speed_mps=25.0,
                   static_margin_frac=0.12, static_margin_min=3,
                   cond_reject_threshold=30.0, rng=None, force_2d=False,
                   # ---- NEW ----
                   airborne: bool = False,
                   static_min_ratio: float = 0.70,   # a static world yields ~ALL zeros
                   static_min_points: int = 10,      # never declare static on a handful
                   min_points_moving: int = 8):
    """
    ... (existing docstring) ...

    STATIC-HYPOTHESIS HARDENING (2026-09-18 flight forensics):
      Across five flights, 50-74% of published frames were exact zeros, and
      100% of frames with <10 inliers were zeros. The inlier count on those
      frames clustered at 8-9 regardless of altitude, speed or mount angle --
      the signature of a fixed population of AIRFRAME SELF-RETURNS that always
      read zero Doppler and therefore always support a "static" hypothesis.
      min_inlier_ratio=0.25 let 3 such points out of 12 outvote the real ground
      returns. A genuinely stationary vehicle produces near-zero Doppler on
      essentially EVERY static-world return, so the static gate is now a
      separate, much stricter threshold, and while airborne a failed solve is
      reported as a GAP rather than as a confident v=0 measurement.
    """
    n = u_body.shape[0]
    if n < 3:
        return None
    rng = rng or np.random.default_rng()

    res_zero = np.abs(v_radial)
    mask_zero = res_zero < eps
    score_zero = int(mask_zero.sum())

    # A static hypothesis is only credible if it explains the OVERWHELMING
    # majority of returns AND rests on a non-trivial number of points.
    static_credible = (n >= static_min_points and
                       score_zero >= static_min_points and
                       (score_zero / n) >= static_min_ratio)
    if airborne:
        # In flight, "static" is essentially never physically true. Demand even
        # more, and prefer a declared gap.
        static_credible = static_credible and (score_zero / n) >= 0.85

    if n < min_points_moving:
        # Too few points for a 3-DOF solve with any redundancy. Do NOT
        # substitute a fabricated zero -- report the gap.
        if static_credible and not airborne:
            return mask_zero, True, 1.0
        return None
    ...
    if best_mask is None or best_score <= score_zero + margin:
        if static_credible:
            return mask_zero, True, 1.0
        return None                       # <-- gap, not a fake zero
```

And expose the knobs:
```python
# doppler_rio.py CLI, after line 822
    p.add_argument('--static-min-ratio', type=float, default=0.70,
                    help="Fraction of returns that must read near-zero Doppler before the "
                         "STATIC hypothesis is accepted. 0.25 (the old min-inlier-ratio) let "
                         "a handful of airframe self-returns outvote the real ground returns "
                         "and publish v=0 for 50-74%% of every flight on 2026-09-18.")
    p.add_argument('--static-min-points', type=int, default=10)
    p.add_argument('--min-points-moving', type=int, default=8)
```

**Wire the `airborne` flag.** `imu_bridge.py` already talks to the FC. Extend `IMU_PKT` to carry it:
```python
# imu_bridge.py
IMU_PKT = struct.Struct('<dffffffB')   # + uint8 airborne (33 bytes)
# request EXTENDED_SYS_STATE at 2 Hz after heartbeat:
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 500000, 0,0,0,0,0)
# landed_state: 1 = ON_GROUND, 2 = IN_AIR
airborne = 1 if (armed and landed_state == 2) else 0
```
Update the matching unpack in `doppler_rio.IMUListener._run()` (line ~76) and
`slam_node.run()` (line ~658). **Both listeners must change together or the struct sizes desync.**

---

## 3. ROOT CAUSE R2 — The "hallucination" is a null-direction slide, not a Nyquist fold

### 3.1 The geometric argument

For a nose radar pitched down by θ, the mean line of sight in body-FRD is `(cos θ, 0, sin θ)`.
The direction **orthogonal to it within the elevation plane** is:

```
n̂(θ) = ( sin θ , 0 , −cos θ )
```

A velocity along `n̂` produces **zero Doppler on boresight** and only
`|v| · sin(ε)` on a ray at elevation offset `ε`. With the U300's **±12° elevation FOV**, a 16 m/s
velocity along `n̂` produces only `16 × sin(12°) = 3.3 m/s` of residual even at the extreme beam
edge, and essentially nothing near the centre. The least-squares cost surface is almost perfectly
flat along `n̂`. In my earlier geometric analysis this appears as **ρ(Vx,Vz) = −0.965 at 45° tilt**.

Nothing in the current code constrains this direction. `weighted_refit()` (line 350) uses
`lstsq(rcond=1e-3)` and `pinv(rcond=1e-3)` — which retains singular values down to 0.001 × σ_max,
i.e. **admits condition numbers up to 1000.** `irls_refit()` (lines 453, 459) does the same.

### 3.2 Proof from the data — the alignment test

For each flight I took every frame where `|v_RIO| > 5 m/s` and computed
`|cos| = |v̂ · n̂(θ)|` — how closely the reported velocity lies along the tilt-specific
null direction. The control group is frames with `|v_RIO| < 3 m/s`.

| Flight | Tilt | `n̂` | Spike frames | **mean \|cos\| (spikes)** | median | **mean \|cos\| (control)** | GPS true speed on spike frames |
|---|---|---|---|---|---|---|---|
| `12:34:05` | 45° | (+0.707, 0, −0.707) | 19 | **0.872** | **0.975** | 0.565 | mean 2.5, max 5.9 m/s |
| `12:36:56` | 45° | (+0.707, 0, −0.707) | 32 | **0.837** | **0.962** | 0.515 | mean 3.8, max 8.5 m/s |
| `12:39:57` | 45° | (+0.707, 0, −0.707) | 47 | **0.853** | **0.975** | 0.520 | mean 3.7, max 6.3 m/s |
| `14:48:05` | **25°** | **(+0.423, 0, −0.906)** | 23 | **0.784** | 0.808 | 0.439 | mean 3.3, max 8.1 m/s |
| `14:50:32` | **25°** | **(+0.423, 0, −0.906)** | 67 | **0.793** | 0.855 | 0.419 | mean 3.0, max 9.4 m/s |

**This is conclusive.** The spikes align with the null direction at 0.78–0.87 mean (median up to
0.975), against a control of 0.42–0.57. And critically — **the null direction is different for the
two mount angles, and the alignment tracks it.** A 45° mount produces spikes along (1,0,−1); a 25°
mount produces spikes along (0.42,0,−0.91). That is a geometric signature, not a numerical accident.

Look at the raw vectors from the top overshoot frames:

```
14:48:05  t= 58.17s  RIO=17.06  GPS=0.00  inl=16  v = [-16.14, -5.54, +15.40]   |Vx|≈|Vz|, opposite
14:48:05  t= 39.18s  RIO=15.18  GPS=0.00  inl=10  v = [-14.59, -4.19, +13.25]
12:39:57  t= 80.87s  RIO=12.81  GPS=0.09  inl=11  v = [-12.47, -2.92, +11.69]
12:39:57  t= 58.07s  RIO=14.89  GPS=5.68  inl=14  v = [-14.27, -4.27, +15.35]
12:34:05  t=134.60s  RIO= 4.07  GPS=0.00  inl=24  v = [ +3.97,  +0.93,  -3.56]
```

Every one has `|Vx| ≈ |Vz|` with opposite sign — the 45°/25° null direction — and **the worst ones
occur when GPS ground truth says the aircraft was doing 0.00 m/s.** The solver is not saturating
on a real fast motion; it is free-running along a direction it cannot see.

### 3.3 Why the existing condition-number gate does not catch it

`cond_reject_threshold = 12.0` is applied at line 333, on `cond(-u_body[best_mask])`. Two holes:

1. **RANSAC maximises inlier *count*, not solution quality.** A null-direction hypothesis that
   retains only the beam-centre points can tie or beat the true solution's inlier count, because
   the excluded edge points are exactly the ones that would have disambiguated it. The winning
   mask is then a narrow central patch whose condition number can sit just under the threshold.
2. **The gate is never re-applied after the refits.** `weighted_refit()` reweights by `1/R²` and
   `irls_refit()` applies polar + Huber weights and hard-excludes points at
   `gross_outlier_mult × huber_delta`. Both can strip the very points that were holding the
   conditioning together, and neither re-checks.

**A condition number is the wrong gate anyway.** It is a unitless ratio that tells you nothing
about whether the answer is good to 0.1 m/s or 10 m/s. The right gate is the posterior velocity
uncertainty, which the code already computes and then throws away.

### 3.4 The fix for R2 — three layers

#### Layer 1 (the principled fix): pin the null direction with a vertical-velocity prior

The degeneracy lives entirely in the **Vx–Vz plane**. You now have two independent sources of
vertical velocity that the radar solve is not using:

- the **U200A belly altimeter** at 20 Hz (differentiable, though noisy: ±0.2 m over 100 ms is
  ±2 m/s — too noisy alone),
- the **FC's own EKF vertical velocity**, available from `LOCAL_POSITION_NED.vz` or `VFR_HUD.climb`,
  which on a Cube Orange with a working barometer is good to roughly ±0.3 m/s.

Add the FC's `vz` as a **soft pseudo-measurement row** in the least-squares. This costs one extra
row, breaks the degeneracy exactly where it lives, and — crucially — does **not** bias `Vx` when
the geometry is good, because the weight is finite:

```python
# doppler_rio.py -- add to weighted_refit() and irls_refit()

def _augment_with_vz_prior(A_w, b_w, vz_prior, sigma_vz):
    """Append the pseudo-measurement  [0,0,1] . v = vz_prior  with weight 1/sigma_vz.

    WHY: at the mount tilts we fly (25-45 deg), rho(Vx,Vz) is -0.90 to -0.965.
    The LOS cone has a near-null direction n = (sin(theta), 0, -cos(theta)), and
    the solver slides along it freely -- 2026-09-18 forensics showed the >5 m/s
    output spikes aligned with n at |cos| = 0.78-0.87 while GPS ground truth
    read 0.00 m/s. A finite-weight vertical prior removes that freedom without
    clamping Vz to a fixed value (which is what --force-2d did, and which leaks
    0.81 m/s of phantom Vx per 1 m/s of true climb at 40 deg tilt).

    sigma_vz should reflect the FC EKF's own vertical velocity accuracy
    (~0.3 m/s on a Cube Orange with a healthy baro + rangefinder). Do NOT set
    it below 0.2 -- that starts fighting genuine radar information.
    """
    if vz_prior is None or not math.isfinite(vz_prior):
        return A_w, b_w
    w = 1.0 / max(sigma_vz, 0.2)
    A_w = np.vstack([A_w, np.array([[0.0, 0.0, 1.0]]) * w])
    b_w = np.concatenate([b_w, [vz_prior * w]])
    return A_w, b_w
```

Thread `vz_prior` from a new field in `IMU_PKT` (the same packet change as §2.4 — do both at once):

```python
IMU_PKT = struct.Struct('<dffffffBf')   # + uint8 airborne + float32 vz_ned (37 bytes)
```

> **Sign discipline.** `LOCAL_POSITION_NED.vz` is **NED, +down**. The radar body frame from
> `TiltMount` is **FRD, +down**. They agree — pass it straight through. Do **not** negate it.
> Verify on the bench by climbing at 1 m/s and confirming `vz_prior < 0`.

#### Layer 2 (cheap and highly effective): an acceleration gate

A multirotor cannot change horizontal velocity by 3 g. At the logged 10 Hz radar rate, any RIO
solution more than ~3 m/s from the previously accepted one implies >30 m/s². Every single spike in
the table above violates this by a wide margin.

```python
# doppler_rio.py -- DopplerRIO.__init__, add:
        self.v_prev = None
        self.t_prev = None
        self.max_accel_mps2 = max_accel_mps2       # default 15.0 (~1.5 g, generous)
        self.n_accel_rejected = 0

# in process_frame(), immediately before the deadband block at line 609:
        if self.v_prev is not None and self.t_prev is not None:
            dt = t_frame - self.t_prev
            if 0.0 < dt < 0.5:
                accel = np.linalg.norm(v_body - self.v_prev) / dt
                if accel > self.max_accel_mps2:
                    # Physically impossible for this airframe. This is the
                    # null-direction slide (2026-09-18 forensics, Sec 3.2),
                    # not a real manoeuvre. Report a GAP.
                    self.n_accel_rejected += 1
                    logging.warning(
                        f"RIO rejected: implied accel {accel:.1f} m/s^2 "
                        f"(v_prev={self.v_prev.round(2)} -> v={v_body.round(2)}, dt={dt:.3f})")
                    return {'t': t_frame, 'valid': False, 'n_total': int(keep.sum()),
                            'reason': 'accel_gate'}
        self.v_prev, self.t_prev = v_body.copy(), t_frame
```

> ⚠️ **Place this AFTER the solve and BEFORE publishing, and make sure `v_prev` is only updated on
> accepted frames.** If you update it on rejected frames the gate walks along behind the spike and
> stops working.

#### Layer 3: gate on posterior uncertainty, not condition number

`weighted_refit` and `irls_refit` already compute `cov`. Use it.

```python
# doppler_rio.py, process_frame(), after irls_refit returns (line ~601):
            sigma = np.sqrt(np.clip(np.diag(cov), 0.0, None)) * self.sigma_v_mps
            if np.any(sigma > self.max_sigma_v_mps):        # default 0.60 m/s
                logging.info(f"RIO rejected: posterior sigma_v = {sigma.round(2)} m/s "
                             f"exceeds {self.max_sigma_v_mps} -- the solve is not "
                             f"constrained along at least one axis.")
                return {'t': t_frame, 'valid': False, 'n_total': int(keep.sum()),
                        'reason': 'sigma_gate'}
```

And tighten the pseudo-inverse conditioning:
```python
# lines 350, 356, 453, 459: rcond=1e-3  ->  rcond=1e-2
# 1e-3 admits cond(A) up to 1000. At our tilt the physically meaningful
# ceiling is ~30; 1e-2 caps it at 100, which is still generous.
```

---

## 4. ROOT CAUSE R3 — Signal starvation, the physical layer

### 4.1 The measured range budget

Slant range to flat ground, `R = h / sin(depression)`, for the two mount angles flown, with the
U300's **±12° elevation FOV**:

| Tilt | AGL | Near edge (θ+12°) | Boresight | Far edge (θ−12°) | Observed mean inliers |
|---|---|---|---|---|---|
| 45° | 5 m | 6 m | 7 m | 9 m | **62** |
| 45° | 15 m | 18 m | 21 m | 28 m | **28** |
| 45° | 20 m | 24 m | 28 m | 37 m | **15** |
| 45° | 30 m | 36 m | 42 m | 55 m | **9** |
| 25° | 20 m | 31 m | 47 m | 89 m | **12** |
| 25° | 30 m | 47 m | 71 m | 133 m | **8** |

**The empirical effective ground-detection range is `R_gnd ≈ 40–50 m`, not the datasheet's 350 m.**
Point count halves roughly every 25–30 m of slant range, and falls below 10 by 40–55 m.

This is expected and I flagged it in the previous plan, but the measured number is **far worse than
my 120 m estimate.** The datasheet's 350 m is a high-RCS point-target figure (`Pr ∝ σ/R⁴` with
`σ ≈ 10–100 m²`). Diffuse ground gives `Pr ∝ R⁻³` with `σ⁰ ≈ −15 to −25 dB`, and at the 13–33°
grazing angles the 25° mount produces, `σ⁰` collapses further.

### 4.2 The 25° mount was the wrong direction

Your 14:48 / 14:50 flights moved from 45° to 25°, presumably to improve forward-velocity
observability. The data says it made things worse in the way that matters:

| | 45° @ 20–30 m AGL | 25° @ 20–30 m AGL |
|---|---|---|
| Boresight slant range | 28–42 m | **47–71 m** |
| Far-edge slant range | 37–55 m | **89–133 m** |
| Far-edge grazing angle | 33° | **13°** (σ⁰ collapse region) |
| Mean inliers observed | 10.2 | 11.7 |
| Zero-frame % | 74.3% | 78.8% |

The theoretical DOP_Vx advantage of a shallow tilt (3.3 vs 5.8) is real but **completely swamped by
the range budget** — you cannot exploit better geometry on points that do not return. This is the
same conclusion the tilt analysis reached, now confirmed in flight.

### 4.3 The fix for R3

**(a) Fly lower until the point count is healthy.** With `R_gnd ≈ 45 m` measured:

| Tilt | Max AGL for ≥20 inliers (est.) | Max AGL for ≥10 inliers |
|---|---|---|
| 25° | ~9 m | ~15 m |
| 45° | ~17 m | ~28 m |
| **55°** | **~22 m** | **~34 m** |
| 60° | ~24 m | ~37 m |

**Go back to 45°, and consider 55° if you need 20 m+ AGL.** Do not go shallower.

**(b) Kill the self-returns** (DIAG-01, §2.3) — raise `--leakage-radius` above `R_self`. Every
self-return you remove is one fewer vote for the false static hypothesis.

**(c) Adaptive frame accumulation for RIO.** The radar runs at **10 Hz (confirmed: `dt` median =
100.0 ms across all five flights)**. When the point count is low, accumulate 2–3 frames before
solving. At 5 m/s, 300 ms is 1.5 m of travel — acceptable if you deskew with the previous velocity
estimate, and vastly better than solving on 8 points.

```python
# doppler_rio.py -- DopplerRIO, new adaptive accumulator
class FrameAccumulator:
    """When the ground return thins out at altitude, a single 10 Hz frame carries
    8-9 points -- not enough for a conditioned 3-DOF solve (2026-09-18 forensics:
    100% of frames with <10 inliers published a fabricated zero). Accumulating
    2-3 frames triples the point count and the angular diversity.

    Cost: the velocity is an average over the window. At 10 Hz and 3 frames that
    is 300 ms; during a 1 g acceleration the within-window velocity change is
    ~3 m/s, so ONLY accumulate when the single-frame count is inadequate, and
    deskew each frame by the last accepted velocity before merging."""
    def __init__(self, min_points=20, max_frames=3, max_window_s=0.35):
        ...
```

**(d) Ask Linpowave for the detection-layer settings** (§8 Q3). The number of reported detections
is CFAR-threshold dependent. If there is a sensitivity or "max targets" parameter, raising it is
free signal.

---

## 5. Nyquist saturation — ruled out

Your hypothesis was a hardware velocity ceiling around 15 m/s. The data does not support it.

**Evidence 1 — the spikes occur at zero ground speed.** A Doppler fold requires a real velocity
exceeding `v_max`. The worst spikes (17.06, 15.18, 13.45, 12.81, 12.01 m/s) all occurred at
**GPS ground truth = 0.00 m/s**. There was nothing to alias.

**Evidence 2 — no ceiling, no fold.** On non-zero frames in flight `14:50:32`, binned by true speed:

| GPS true speed | n | RIO mean \|v_xy\| | ratio |
|---|---|---|---|
| 4–5 m/s | 7 | 5.33 | 1.21 |
| 5–6 m/s | 8 | 7.56 | 1.39 |
| 6–8 m/s | 8 | 7.84 | 1.09 |
| **8–12 m/s** | 9 | **8.02** | **0.96** |

RIO **tracks correctly right up to 12 m/s** with 14.9 mean inliers. A Nyquist fold would produce a
hard ceiling and sign inversions; there are neither. The excursions are at *low* true speed.

**Evidence 3 — the physics has margin.** The Linpowave U300 is published at **±45 m/s**
unambiguous velocity. At the highest logged ground speed (10 m/s at 25° tilt), the maximum
radial component is `10 × cos(13°) = 9.7 m/s` — 4.6× inside the limit.

**Evidence 4 — the 21.4 m/s "true speed" was not real.** GPS for flight `12:39:57` shows a maximum
of 6.37 m/s (median-filtered) / 6.73 m/s (raw). The 21.4 figure appears to be RIO's own output being
read as ground truth, which is exactly the confusion R2 creates.

> **Still worth confirming.** Ask Linpowave for `v_max` **at your configured range profile** (§8 Q3).
> If they ship a long-chirp 350 m mode, `v_max = λ/(4·T_c)` can drop to ±8–12 m/s, and that *would*
> matter at 20 m/s cruise. It did not matter on 2026-09-18.

---

## 6. Why the two bugs were masking each other

This is the reason the error looked like unexplainable scatter (21%, 55%, 68%) rather than a clean
bias, and it dictates the order you must fix things in.

**Counterfactual:** if you changed nothing except suppressing the fabricated zeros — i.e. holding
the last good velocity across the gaps instead of integrating zero:

| Flight | GPS path | As-flown RIO | ratio | **Hold-last-good** | **ratio** |
|---|---|---|---|---|---|
| `12:34:05` | 168.6 m | 61.4 m | 0.364 | **315.1 m** | **1.869** |
| `12:36:56` | 202.5 m | 75.9 m | 0.375 | **490.5 m** | **2.423** |
| `12:39:57` | 146.4 m | 118.5 m | 0.809 | **315.0 m** | **2.151** |
| `14:48:05` | 148.5 m | 79.6 m | 0.536 | **492.5 m** | **3.317** |
| `14:50:32` | 177.2 m | 142.1 m | 0.802 | **421.8 m** | **2.380** |

**Fixing R1 alone turns a 64% under-report into a 232% over-report.** The zeros were diluting the
inflated non-zero frames. The apparent accuracy of flight `12:39:57` (0.809, your "best" result) is
not a better solution — it is a luckier cancellation, and the counterfactual shows its underlying
frames were the second-worst of the five.

> ### 🚨 Do not deploy R1's fix without R2's fix.
> If you only stop publishing zeros, the EKF will receive uncontested null-direction spikes at up to
> 17 m/s. In `nav_node`'s dead-reckoning that is instant divergence; in a GUIDED flight it is a
> flyaway. **R2 Layer 2 (the acceleration gate) is 12 lines of code and stops every spike in this
> dataset. Implement it first, then R1.**

---

## 7. Everything else the line-by-line audit turned up

### 7.1 Instrumentation gap — you cannot currently diagnose this from the logs

`FORWARD_PKT` (`doppler_rio.py:42`) is `'<dfffIfff'` — timestamp, vx, vy, vz, **n_inliers**, cxx,
cyy, czz. It does **not** carry:

- `n_total` (how many points reached the solver),
- `is_static` (was this a real solve or the static fallback?),
- `cond` (the conditioning of the accepted inlier set),
- the rejection reason for frames that produced nothing.

Consequently `gps_logger.py:244` records only `inliers`, and **rejected frames are not logged at
all.** I had to infer the static-branch signature from the shape of the inlier histogram. Fix this
before the next flight or you will be guessing again.

```python
# doppler_rio.py:42
FORWARD_PKT = struct.Struct('<dfffIfffIBf')
# t, vx, vy, vz, n_inliers, cxx, cyy, czz, n_total, flags, cond
#   flags bit0 = is_static, bit1 = airborne, bit2 = vz_prior_used,
#         bit3 = accel_gate_armed, bit4 = accumulated_frames_used

# doppler_rio.py:710
                flags = (int(result.get('is_static', False))
                         | (int(self.airborne) << 1)
                         | (int(vz_prior is not None) << 2))
                pkt = FORWARD_PKT.pack(t_frame, vx, vy, vz, result['n_inliers'],
                                       cov[0,0], cov[1,1], cov[2,2],
                                       result['n_total'], flags,
                                       float(result.get('cond', 0.0)))
```

**And log the rejections.** Add a second small packet (or reuse the same one with
`n_inliers = 0` and a reason code in `flags`) so `gps_logger` can record *why* a frame produced
nothing. Right now a rejected frame is indistinguishable from a dropped UDP datagram.

> **Update all four consumers together** — `slam_node.py` (RIO_PKT at line ~50),
> `nav_node.py` (RIO_PKT at line ~46), `mavlink_bridge.py` (RIO_PKT at line ~52),
> `gps_logger.py` (RIO_PKT at line 55). A struct change that misses one consumer produces a silent
> `struct.error` inside a `try/except` and that stream just goes quiet.

### 7.2 The deadband is not the problem, but it is mislabelled

`--deadband 0.05` (line 831) snaps `|v| < 0.05` to exactly zero at line 609. This contributes a
negligible fraction of the zeros (the histogram shows them clustering at the static-branch
signature, not at genuine small velocities). **Leave it, but tag it in the flags** so the two kinds
of zero are distinguishable in the log.

### 7.3 Confirmed-good: the fixes applied since the last review

Diffing against the previous upload, these landed correctly and I found no regressions:

| Change | Status |
|---|---|
| `force_2d` decoupled from `imu_level_points`, now an explicit flag with a tilt guard (line 654) | ✅ correct |
| Lever-arm rotation compensation replaced with `u·(ω × r_lever)` (lines 550–566) | ✅ correct — the old `u·(ω × p)` was identically zero |
| `cov[2,2] = 1e6` in 2D mode | ✅ correct |
| Static covariance `1e-4 → 0.25` (line 588) | ✅ helps the EKF, **does nothing for distance integration** |
| `_gravity_correct()` altimeter Z blend, `alpha = 0.15` (lines 308–322) | ✅ correct — and it is why your SLAM Z error is 0.63 m |
| `_gravity_correct()` now gated on `imu_level_points` (lines 511, 553) | ✅ correct |
| `altimeter_bridge.py` created, wired to ports 5030–5033 | ✅ correct |
| `--tilt-deg` now `required=True` (supervisor line 224) | ✅ correct |
| `--max-range` default 350 → 150 (line 814) | ✅ correct |
| `t_frame` now monotonic (logs confirm `t_mono − t_frame` = 12–16 ms, stable) | ✅ correct |

**Your SLAM Z accuracy (0.63 m) is direct evidence that the altimeter integration works.** The Z
axis has an absolute reference and behaves; the horizontal axes do not and do not. That asymmetry
is the whole story of this investigation.

### 7.4 One real latency anomaly

Flight `12:36:56` shows `t_mono − t_frame` p95 = **230 ms** (all other flights: 21–26 ms). Something
stalled the pipeline for ~0.2 s repeatedly in that flight — most likely GICP blocking in
`slam_node` (it warns above 150 ms at line 503) back-pressuring through the shared process tree, or
Jetson thermal throttling. Check `tegrastats` next flight, and run
`sudo nvpmodel -m 0 && sudo jetson_clocks`.

### 7.5 Publish gaps are already non-trivial

| Flight | Gaps > 0.25 s | Total gap time | Distance lost in gaps |
|---|---|---|---|
| `12:34:05` | 5 | 1.8 s | 3.0 m |
| `12:39:57` | **21** | **7.8 s** | **25.2 m (17% of the path)** |
| `14:48:05` | 7 | 3.2 s | 13.0 m |

After the R1 fix these will grow substantially, because frames that currently publish a fake zero
will correctly publish nothing. **That is the intended behaviour**, but it means the downstream
consumer must handle gaps properly:

- `mavlink_bridge` → nothing sent, EKF3 coasts on IMU with widening covariance. Correct.
- `nav_node.PoseTracker.on_rio_velocity` → dead-reckoning simply stops advancing, and
  `stale_for()` eventually trips the failsafe ladder. Correct.
- **`slam_node`'s constant-velocity prior (line ~474) will use a stale `last_v_body`.** Add an age
  check — if the last RIO velocity is older than 0.3 s, use zero translation in `T_pred` rather
  than extrapolating a stale velocity.

---

## 8. Verification flight plan

### 8.1 Ground diagnostics (before flying again)

| ID | Test | What it settles |
|---|---|---|
| **DIAG-01** | Restrained, props at 30%, log raw `Points_float`. Histogram range vs \|v\|. Repeat props-off and hand-carried at walking pace. | **The self-return population.** Sets `--leakage-radius`. This is the highest-value 15 minutes in the whole plan. |
| **DIAG-02** | Replay the five 2026-09-18 logs through the patched `doppler_rio` offline. | Confirms the fixes reproduce GPS distance **on data you already have**, with zero flight risk. |
| **DIAG-03** | Walk test: carry the rig 20 m at ~1 m/s, levelled, over the same terrain. | RIO scale and sign at a point count you know is healthy. |
| **DIAG-04** | Raise/lower the rig 2 m while walking forward. | That the vz-prior fix has not reintroduced Vx/Vz leakage. |

> **DIAG-02 is the gate.** You have 6,538 frames of real flight data with GPS ground truth. Do not
> fly the patched code until it reproduces those five flights offline to within 10% of GPS path
> length. Write the replay harness as `tools/replay_rio.py` — it needs only the raw point stream,
> which means **you must also start logging raw points** (see §7.1).

### 8.2 Flight ladder

| Gate | Flight | Pass criterion |
|---|---|---|
| **F1** | Hover 60 s at 5 m AGL, 45° tilt | zero-frame rate < 10%; mean inliers > 40; no frame with \|v\| > 1 m/s |
| **F2** | Altitude ladder: 30 s hover at 5, 10, 15, 20, 25, 30, 40 m | **Record mean inliers and zero-rate at each step. This gives you the real ceiling.** Stop climbing when inliers < 20. |
| **F3** | 100 m out-and-back at 3 m/s, at the F2 ceiling | closure error < 5% of path; zero-frame rate < 20%; **no frame with \|cos\| to `n̂` > 0.9** |
| **F4** | Repeat F3 at 6 m/s | ratio RIO/GPS within 0.95–1.05 |
| **F5** | Repeat F3 with deliberate ±15° pitch oscillation | accel-gate rejection rate < 5%; no spikes |
| **F6** | The original 12:34-style profile, 45° tilt, at the F2 ceiling | **RIO/GPS distance ratio 0.95–1.05** |

### 8.3 The metrics to compute on every flight from now on

Add to `tools/analyze_run.py`:

```python
def rio_health(log):
    """The five numbers that would have found this bug in one flight instead of five."""
    return {
        'zero_frame_pct':      100 * (v_norm == 0).mean(),
        'static_branch_pct':   100 * (flags & 1).mean(),        # needs the FORWARD_PKT change
        'reject_pct':          100 * n_rejected / n_attempted,   # needs rejection logging
        'null_alignment_p95':  np.percentile(np.abs(v_hat @ n_hat), 95),
        'dist_ratio':          dist_rio / dist_gps,
        'scale_on_nonzero':    np.median(sp_rio[nz] / sp_gps[nz]),
        'inliers_p10':         np.percentile(inliers, 10),
    }
```

`null_alignment_p95` is the one to watch. **Above 0.85 you have a degeneracy problem regardless of
what the distance ratio says.**

---

## 9. Implementation order

**Do these in order. Each is independently testable against the existing logs via DIAG-02.**

1. **§7.1 — extend `FORWARD_PKT`, log `n_total`/`is_static`/`cond`, log rejections.** Also start
   logging raw points so DIAG-02 is possible. *Nothing else is verifiable until this is done.*
2. **§3.4 Layer 2 — the acceleration gate.** 12 lines. Stops every spike in the dataset.
3. **§2.4 — static-hypothesis hardening** (`static_min_ratio = 0.70`, `static_min_points = 10`,
   gap-not-zero while airborne). Requires the `airborne` flag from `imu_bridge`.
4. **§3.4 Layer 3 — posterior-σ gate, `rcond` 1e-3 → 1e-2.**
5. **DIAG-02 offline replay against all five 2026-09-18 logs.** Iterate on thresholds here, not
   in the air.
6. **§3.4 Layer 1 — the vz prior.** Bigger change (packet format + FC message). Do it after the
   cheap fixes are proven, and measure how much it adds.
7. **DIAG-01 — self-return survey → set `--leakage-radius`.**
8. **§4.3(c) — adaptive frame accumulation.** Only if F2 shows you still need altitude you cannot
   reach.
9. **Return the mount to 45°** (or 55° if you need 20 m+). **Do not fly 25° again.**

---

## 10. What I need from you

### 🔴 Blocking

**Q1 — Raw point logs.** The single biggest gap in this investigation is that I could infer the
self-return population only from the shape of the inlier histogram. Run **DIAG-01** and send me
the raw `Points_float` dump (ground, props spinning; and hand-carried at walking pace). That
converts a strong inference into a measurement and tells me exactly what `--leakage-radius` to set.

**Q2 — The exact `supervisor.py` command line used for the 2026-09-18 flights.** Specifically
`--eps`, `--min-inlier-ratio`, `--cond-reject-threshold`, `--deadband`, `--leakage-radius`,
`--imu-level-points` / `--no-imu-level-points`, and the `--lever-x/y/z` values. The supervisor
defaults (`--eps 0.40`) differ from `doppler_rio`'s own (`--eps 0.20`), and which one was active
changes the static-branch arithmetic substantially.

**Q3 — From Linpowave, for your firmware build:**
- **`v_max`** (max unambiguous velocity) **at your configured range profile** — not the marketing
  ±45 m/s (§5).
- The **CFAR / detection-sensitivity** parameters and the **max detections per frame**. §4 shows you
  are point-starved; if there is free sensitivity, take it.
- Whether the unit can run at **20 Hz** rather than the 10 Hz measured in these logs. Doubling the
  frame rate halves the accumulation window needed in §4.3(c).
- Confirmed **azimuth and elevation FOV** for your build. All geometry here assumes ±40° × ±12°.

**Q4 — Which mount angle do you want to standardise on?** My recommendation from this data is
**45°**, or **55°** if you need to operate above 20 m AGL. Confirm before I write the
patch, because `n̂(θ)` and the accumulation thresholds depend on it.

### 🟠 Needed soon

**Q5 — Does the FC publish `LOCAL_POSITION_NED` or `VFR_HUD` on the link `imu_bridge.py` uses,
and at what rate?** The vz prior (§3.4 Layer 1) needs it at ≥10 Hz. If not, I will derive the prior
from the U200A instead, with a wider σ.

**Q6 — Terrain at the test site.** Grass, soil, crops, asphalt, or mixed? `R_gnd ≈ 45 m` is very low
even for diffuse terrain, and a specular surface (asphalt, smooth concrete, water) would explain it.
This determines whether §4's ceilings are the sensor's limit or the site's.

**Q7 — Confirm the altitude discrepancy in §1.3.** Two independent sensors say flight `14:48:05`
was at ~20 m, not 1–3 m. If your notes are right and both sensors are wrong, that is a far more
serious problem than anything else in this document and we should stop and investigate that first.

---

## 11. One-paragraph summary

RIO is not under-reporting speed — it is reporting **exactly zero for 50–74% of every flight**, and
that accounts for **77–81% of the missing distance**. The cause is `doppler_ransac()` accepting the
static hypothesis on as few as 3 near-zero-Doppler points, which in flight are almost certainly
**airframe self-returns** (they cluster at a constant 8–9 across all altitudes, speeds and mount
angles — no genuine ground return can read zero Doppler during forward flight). The "23 m/s
hallucination" is a separate defect: the velocity solve **slides along the unobservable Vx/Vz
direction** `n̂(θ) = (sin θ, 0, −cos θ)`, proven by the spikes aligning with `n̂` at |cos| = 0.78–0.87
versus a 0.42–0.57 control, with the alignment **tracking the mount angle** between the 45° and 25°
flights, and with the worst spikes occurring when GPS says the aircraft was doing **0.00 m/s**.
Nyquist saturation is ruled out — RIO tracks correctly to 12 m/s. The two defects partially cancel,
so **fixing the zeros alone turns a 64% under-report into a 232% over-report**; the acceleration
gate and the vertical-velocity prior must land with it. Finally, the 25° mount made things worse,
not better: measured `R_gnd ≈ 45 m` means shallow tilt buys geometry you cannot use on points that
never return.

*Generated 2026-09-18 from all five flight logs and a full read of the uploaded tree.
Reproduce with `docs/rio_forensics.py`.*
