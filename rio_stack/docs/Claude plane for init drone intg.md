# Master Implementation Plan: Active GPS-Denied Navigation
### Linpowave U300 (nose, tilted) + U200A (belly altimeter) + Cube Orange / ArduPilot + Jetson

> **Target path:** `/home/alok/radar/rio_stack/docs/implementation_plan.md`
> **Supersedes:** the previous `implementation_plan.md` ("Drone Migration v2 — PASSIVE ONLY").
> That document was correct for its scope. This one moves the stack from *passive logging* to
> *active closed-loop control*, which is a completely different trust level.
> **Audit basis:** full read of `doppler_rio.py` (822 L), `slam_node.py` (809 L), `nav_node.py` (645 L),
> `mavlink_bridge.py`, `radar_fanout.py`, `imu_bridge.py`, `filters.py`, `supervisor.py`, `gps_logger.py`.

---

## 0. Verdict up front

**You cannot fly this today.** Not because the architecture is wrong — the architecture is
genuinely good — but because there are **eleven defects** between the radar and the motors,
of which **four will cause a crash or a flyaway on the first GUIDED flight**, and **one whole
sensor (the U200A belly altimeter) is not wired into the stack at all.**

| # | Defect | Severity | File |
|---|--------|----------|------|
| **B1** | `type_mask` bits inverted → vehicle commanded to **yaw to magnetic North** on every setpoint | 🔴 **CRASH** | `nav_node.py:296` |
| **B2** | `fwd_range = -1.0` sentinel read as a real 1 m obstacle → **permanent hard-stop whenever the path is clear** | 🔴 **MISSION-KILL** | `slam_node.py:713` + `nav_node.py:364` |
| **B3** | `on_slam_pose()` called without `attitude` → **`yaw_offset` is permanently 0.0**; SLAM frame and NED frame silently disagree unless you take off facing exactly North | 🔴 **FLYAWAY** | `nav_node.py:425` |
| **B4** | `is_static` publishes **v=0 with covariance 1e-4** to the EKF. In flight, a degenerate frame becomes a confident "we are not moving" | 🔴 **FLYAWAY** | `doppler_rio.py:572` |
| **B5** | `force_2d` silently zeroes Vz whenever IMU levelling is on; leaks **0.81 m/s of phantom forward velocity per 1 m/s of climb** at 40° tilt | 🔴 **ALTITUDE/POSITION DIVERGENCE** | `doppler_rio.py:560,574,585` |
| **B6** | **U200A altimeter has no driver.** `mavlink_bridge.py --alt-port` exists, nothing feeds it, supervisor never sets it | 🔴 **MISSING SUBSYSTEM** | new file |
| **B7** | IMU rotation compensation `u·(ω×p)` is **mathematically identically zero** — it has never done anything | 🟠 | `doppler_rio.py:549-551`, `filters.py:112-114` |
| **B8** | `_gravity_correct()` hard-clamps roll/pitch to 0 **and is unconditionally applied**; `slam_node.py:682` hardcodes `imu_level_points=True`, ignoring the supervisor flag | 🟠 | `slam_node.py:279,682` |
| **B9** | No EKF-health monitoring; `FAILSAFE_RTL` hands off to an autopilot RTL that has **no trustworthy position source** | 🟠 | `nav_node.py:493` |
| **B10** | MAVLink vision timestamps use an **arbitrary epoch** (`time.time() - t0_wall`), so `EK3_VIS_DELAY` cannot be calibrated | 🟠 | `mavlink_bridge.py:114,120` |
| **B11** | `--theta-tilt-deg` defaults to **90.0** in all three entry points; a forgotten flag silently reinterprets a 40° nose radar as a nadir radar | 🟡 | `doppler_rio.py:781`, `slam_node.py:754`, `supervisor.py` |

Fix all of these and the system is flyable. Sections 4 and 5 give exact line-level diffs.

---

## 1. Confirmed hardware facts (and one discrepancy you must resolve)

### 1.1 Linpowave U300 — nose / navigation radar

| Parameter | Manufacturer value | Source |
|---|---|---|
| Band | 76–81 GHz FMCW MIMO | Linpowave product page |
| Range | 0.2 – 350 m | Linpowave product page |
| Range accuracy | ±0.23 m | Linpowave U300 blog |
| **Azimuth FOV** | **±40°** | Linpowave U300 blog |
| **Elevation FOV** | **±12°** | Linpowave U300 blog |
| **Max unambiguous velocity** | **±45 m/s** | Linpowave U300 blog |
| Frame rate | 20 Hz | Linpowave U300 blog |
| Output | Points_float TLV type 1: `(x, y, z, v)` float32, no SNR | your `Points_float_PySDK_UART.py` |

> ⚠️ **DISCREPANCY — resolve before mounting.**
> Your brief states **100° azimuth × 40° elevation**. Linpowave publishes **±40° × ±12°**
> (= 80° × 24°). Your own code independently agrees with the datasheet: `doppler_rio.py`'s
> `self_test()` at line 732-733 samples `az ∈ ±0.6 rad (±34°)` and `el ∈ 40°±12°`.
> **Every number in Section 2 is computed with ±40°/±12°, the conservative case.**
> If Linpowave confirms ±50°/±20° for your firmware build, re-run `docs/tilt_analysis.py`
> (Section 2.7) — the optimal tilt shifts by roughly −3° and all DOP figures improve ~35%.

### 1.2 Linpowave U200A — belly radar altimeter

| Parameter | Value | Source |
|---|---|---|
| Band | 77–81 GHz FMCW | Linpowave U200A blog |
| Range | 0.2 – 200 m | Linpowave U200A blog |
| Accuracy | ±0.2 m | Linpowave U200A blog |
| Resolution | 0.05 m | Linpowave UAV altitude module page |
| Rate | 20 Hz | Linpowave U200A blog |
| Interface | TTL / CAN / RS422 | Linpowave U200A blog |

**This sensor is the single most important component in the whole GPS-denied stack** and it is
currently not connected to anything. Section 2.4 explains why.

### 1.3 The 350 m claim

350 m is a **point-target** figure — a high-RCS object (vehicle, drone, corner reflector),
where received power falls as `Pr ∝ σ/R⁴` with `σ ≈ 1–100 m²`.

Ground clutter is different. For a distributed surface the illuminated cell area grows with
range, so `σ_clutter = σ⁰ · A_cell` and:

```
A_cell ≈ R·θ_az × (c·Δτ/2)/sin(γ)        (range-resolution-limited cell)
  ⇒  σ_clutter ∝ R
  ⇒  Pr ∝ σ_clutter / R⁴  ∝  R⁻³         (not R⁻⁴)
```

but `σ⁰` for terrain collapses at shallow grazing angle `γ`:
`σ⁰ ≈ −10 dB` at γ=45°, `≈ −20 dB` at γ=20°, `≈ −30 dB or worse` below γ=10°
(grass/soil at W-band; asphalt and water are far worse — see §12.3).

**Working assumption used throughout this plan: `R_gnd ≈ 120 m` for diffuse terrain at
γ > 25°.** This is an estimate, not a fact.

> 📌 **TEST-01 (must run before any nav flight).** Hover the aircraft at 10, 20, 30, 40, 60, 80 m AGL
> for 30 s each over your actual operating terrain. Log `n_total` from `doppler_rio.py`. Record the
> altitude at which `n_total` drops below 15 points/frame. **That number is your real `R_gnd`, and it
> sets your operational ceiling.** Everything in Section 2.5 must be recomputed with it.

---

## 2. Mathematical angle & altitude analysis

### 2.1 Geometry and frame definition

Radar native frame (confirmed from the Points_float protocol PDF and `TiltMount` docstring at
`doppler_rio.py:115-139`): **X_R = lateral, Y_R = forward/boresight, Z_R = up.**

The permutation `P` (line 155-159) maps this to pre-tilt body FRD:
`X_B0 = Y_R` (forward), `Y_B0 = ±X_R` (right), `Z_B0 = −Z_R` (down).
Then `R_tilt` (line 146-150) pitches down by θ about body +Y:

```
R_R→B(θ) = R_tilt(θ) · P
```

For a ray at radar azimuth α and elevation ε, the body-frame LOS unit vector is:

```
u_fwd  =  cos θ · cos ε cos α  +  sin θ · sin ε
u_right=  cos ε · sin α
u_down =  sin θ · cos ε cos α  −  cos θ · sin ε
```

Boresight depression = θ. Beam edges = θ ± 12°.

### 2.2 TABLE A — Slant range to flat ground, `R = h / sin(depression)`

Boresight slant range in metres. **Bold = beyond the assumed 120 m diffuse-ground range.**

| tilt θ | near edge (θ+12°) | far edge (θ−12°) | 10 m | 30 m | 50 m | 80 m | 100 m | 150 m | 200 m |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10° | 22° | −2° (**sky**) | 58 | **173** | **288** | **461** | **576** | **864** | **1152** |
| 20° | 32° | 8° | 29 | 88 | **146** | **234** | **292** | **439** | **585** |
| 30° | 42° | 18° | 20 | 60 | 100 | **160** | **200** | **300** | **400** |
| **35°** | 47° | 23° | 17 | 52 | 87 | **139** | **174** | **262** | **349** |
| **40°** | 52° | 28° | 16 | 47 | 78 | **124** | **156** | **233** | **311** |
| **45°** | 57° | 33° | 14 | 42 | 71 | 113 | **141** | **212** | **283** |
| 50° | 62° | 38° | 13 | 39 | 65 | 104 | **131** | **196** | **261** |
| 60° | 72° | 48° | 12 | 35 | 58 | 92 | 115 | **173** | **231** |
| 75° | 87° | 63° | 10 | 31 | 52 | 83 | 104 | **155** | **207** |
| 90° | 102°¹ | 78° | 10 | 30 | 50 | 80 | 100 | **150** | **200** |

¹ depression > 90° = the beam has passed nadir and is looking *behind* the aircraft.

**Immediate consequences:**
- **θ ≤ 12° is structurally invalid.** The far edge of the beam points at or above the horizon —
  you get sky, not ground, and any returns there are aircraft/terrain at unknown range.
- At **10° tilt**, a 50 m hover needs a 288 m ground return. That will not happen.
- The classic "shallow tilt gives the best forward-velocity observability" instinct is
  **defeated by the range budget**, not by the geometry.

### 2.3 TABLE B — Velocity observability (Dilution of Precision)

For `V_i = −uᵢᵀv`, the WLS covariance is `Cov(v) = σ_v² (Σ uᵢuᵢᵀ)⁻¹`. Define
`M = (1/N)Σ uᵢuᵢᵀ`, then `DOP_k = √[M⁻¹]_kk` and:

```
σ_v,k  =  σ_doppler · DOP_k / √N
```

Uniform sampling over the full ±40°/±12° FOV:

| tilt θ | DOP_Vx (fwd) | DOP_Vy (lat) | DOP_Vz (vert) | ρ(Vx,Vz) |
|---:|---:|---:|---:|---:|
| 10° | **1.78** | 2.59 | 8.05 | −0.782 |
| 20° | 2.98 | 2.59 | 7.69 | −0.921 |
| 30° | 4.19 | 2.59 | 7.10 | −0.954 |
| 35° | 4.77 | 2.59 | 6.72 | −0.960 |
| 40° | 5.32 | 2.59 | 6.30 | −0.964 |
| 45° | 5.83 | 2.59 | 5.83 | **−0.965** |
| 50° | 6.30 | 2.59 | 5.32 | −0.964 |
| 60° | 7.10 | 2.59 | 4.19 | −0.954 |
| 75° | 7.90 | 2.59 | 2.36 | −0.878 |
| 90° | 8.17 | 2.59 | **1.09** | 0.000 |

Three structural facts fall out of this table:

1. **DOP_Vy is constant at 2.59 for every tilt.** Lateral velocity comes from the wide ±40°
   azimuth fan, which the pitch rotation does not compress. Lateral velocity is your *best*
   observed axis and it is tilt-independent. Useful: it means tilt selection is purely a
   forward-vs-vertical trade.
2. **DOP_Vx and DOP_Vz trade off exactly.** `DOP_Vx(θ) = DOP_Vz(90°−θ)`. They cross at 45°.
3. **ρ(Vx,Vz) peaks at −0.965 around 45°.** Forward and vertical velocity are almost
   perfectly confounded in the 30–60° band. This is the mathematical root of `force_2d`
   being so destructive (§2.6) and the mathematical justification for the belly altimeter (§2.4).

### 2.4 Why the U200A belly altimeter is not optional

At 40° tilt, ρ(Vx,Vz) = −0.964. The nose radar **cannot separate "flying forward" from
"descending"** from a single frame to better than that correlation. Two independent errors then
compound:

- A vertical velocity error leaks into horizontal position at ~0.8:1 (Table E).
- A vertical **position** error is unbounded: nothing in the RIO/SLAM loop measures absolute height.
  `slam_node.py`'s own comment at line 283 records the symptom: **191 m of Z drift in 96 s.**

The U200A breaks the loop by supplying an **absolute, drift-free, bias-free height measurement**
at 20 Hz with ±0.2 m accuracy. Feeding it to EKF3 as a rangefinder (`EK3_SRC1_POSZ = 2`):

- pins the Z state absolutely, so Z drift → 0;
- via the EKF's cross-covariance between Z and the horizontal states, **partially de-confounds
  Vx from Vz**, recovering a large fraction of the DOP_Vx penalty you paid to tilt steeply;
- gives you terrain-following and a safe automatic landing;
- gives `nav_node.py` a real AGL for the geofence instead of `−p[2]` off a drifting SLAM pose.

**Conclusion: the belly radar is a hard prerequisite for active navigation, not an accessory.**

### 2.5 TABLE C — Fraction of the beam that actually returns, vs altitude

A ray only produces a ground detection if its slant range `h/sin(depression) ≤ R_gnd`.
Percentage of the ±40°×±12° FOV that returns, at `R_gnd = 120 m`:

| tilt | 10 m | 30 m | 50 m | 80 m | 100 m | 150 m | 200 m |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 20° | 99% | 66% | 24% | 0% | 0% | 0% | 0% |
| 30° | 100% | 98% | 61% | 0% | 0% | 0% | 0% |
| 35° | 100% | 100% | 80% | 10% | 0% | 0% | 0% |
| **40°** | 100% | 100% | 94% | 26% | 0% | 0% | 0% |
| **45°** | 100% | 100% | 99% | 44% | 0% | 0% | 0% |
| 50° | 100% | 100% | 100% | 62% | 8% | 0% | 0% |
| 60° | 100% | 100% | 100% | 90% | 34% | 0% | 0% |
| 90° | 100% | 100% | 100% | 100% | 81% | 0% | 0% |

And at the optimistic `R_gnd = 200 m`:

| tilt | 50 m | 80 m | 100 m | 150 m | 200 m |
|---:|---:|---:|---:|---:|---:|
| 40° | 100% | 96% | 76% | 5% | **0%** |
| 60° | 100% | 100% | 100% | 67% | **0%** |
| 90° | 100% | 100% | 100% | 100% | **0%** |

### 2.6 **Proof: 200 m AGL is impossible, at any tilt angle**

This is not a tuning problem. It is a hard physical bound with three independent proofs.

**Proof 1 — the nadir bound.** The shortest possible path to the ground is straight down.
Therefore for *any* mounting angle, ground detection at altitude `h` requires
`R_gnd ≥ h`. At 200 m AGL you need a **200 m diffuse-ground detection range**.
With `R_gnd ≈ 120 m`, the whole beam returns nothing — the last column of Table C
is `0%` at **every tilt including 90°**. At 150 m AGL, likewise. The aircraft is
navigating on IMU integration alone, which diverges as t³.

**Proof 2 — the grazing-angle bound.** Suppose `R_gnd = 350 m` (the optimistic
point-target number). At 200 m AGL and 40° tilt, only 55% of the beam returns —
but those returns come from slant ranges of 250–330 m at grazing angles of 28–52°, where
the illuminated cell is ~10× larger and the SNR per cell is 3 log-units down. In practice
you get a handful of the brightest scatterers (a road sign, a metal roof), not a ground
surface. The Doppler solve then fits **a few specular point targets**, which is exactly the
regime `doppler_ransac`'s condition-number gate (line 333) is designed to reject.

**Proof 3 — the conditioning bound.** Even in the altitude band where *some* returns survive,
only the steepest slice of the beam gets through, so the surviving LOS set collapses toward a
single direction. Conditioning of the **surviving** set, `R_gnd = 120 m`:

| tilt | 10 m | 30 m | 50 m | 80 m | 100 m |
|---:|---:|---:|---:|---:|---:|
| 35° | 56 | 56 | 79 | **1345** | — |
| 40° | 56 | 56 | 61 | **364** | — |
| 45° | 56 | 56 | 57 | **169** | — |
| 50° | 56 | 56 | 56 | 99 | **1308** |
| 60° | 56 | 56 | 56 | 61 | **174** |
| 90° | 56 | 56 | 56 | 56 | 60 |

Your `--cond-reject-threshold` is **12.0** (supervisor default). Your own gate throws
these frames away long before the physics does. At 80 m with a 40° tilt, condition number 364
is 30× your reject threshold: **RIO publishes nothing, the EKF coasts, and you drift.**

> ### 🔒 **HARD ALTITUDE CEILING**
> | Measured `R_gnd` | Max safe AGL (40° tilt) | Max safe AGL (60° tilt) |
> |---|---|---|
> | 80 m | **35 m** | 45 m |
> | 120 m | **50 m** | 70 m |
> | 200 m | **80 m** | 110 m |
> | 350 m | **130 m** | 180 m |
>
> **Enforce this in software** (`nav_node.py` `max_altitude_agl_m`, currently 200.0 — see B-12).
> The 100–200 m mission profile in your memory notes is **not achievable** with this sensor pair
> unless `R_gnd` measures ≥ 250 m, which it will not over diffuse terrain.

### 2.7 Optimal tilt angle — the decision

Minimise worst-case `DOP_Vx` across the operating altitude band, subject to ≥30% of the beam
returning at every altitude in the band:

| `R_gnd` | Ceiling 30 m | Ceiling 50 m | Ceiling 80 m |
|---|---|---|---|
| 80 m | 35° (DOP 5.19) | 53° (DOP 7.06) | **no feasible tilt** |
| 120 m | 27° (DOP 4.17) | **38° (DOP 5.52)** | 56° (DOP 7.32) |
| 200 m | 21° (DOP 3.34) | 27° (DOP 4.17) | 37° (DOP 5.39) |

Cross-checking against DOP of the surviving set (`R_gnd = 120 m`), `DOP_Vx / DOP_Vy`:

| tilt | 10 m | 30 m | 50 m | 80 m |
|---:|---:|---:|---:|---:|
| 30° | 4.19 / 2.59 | 4.30 / 2.63 | 7.05 / 2.78 | — |
| 35° | 4.77 / 2.59 | 4.77 / 2.59 | 5.90 / 2.75 | 26.7 / 4.67 |
| **40°** | **5.32 / 2.59** | **5.32 / 2.59** | **5.57 / 2.68** | **14.9 / 3.64** |
| 45° | 5.83 / 2.59 | 5.83 / 2.59 | 5.88 / 2.60 | 10.7 / 3.22 |
| 50° | 6.30 / 2.59 | 6.30 / 2.59 | 6.30 / 2.59 | 8.59 / 3.03 |

Note that **40° dominates 35° at every altitude** — at 50 m the 35° beam has already started
clipping (5.90 vs 5.57), and at 80 m it is nearly twice as bad. This is the non-obvious result:
the shallower angle is *not* better even for forward velocity, once the range budget is applied.

> ### ✅ **DECISION: mount the U300 at 40° nose-down (40.0° ± 0.5°).**
>
> **Why 40° and not something else:**
> - 40° is flat across the whole 0–50 m band (DOP_Vx 5.32→5.57, a 5% variation), so performance
>   does not change as you climb. 30° varies by 68% over the same band.
> - It keeps a real reserve: 94% beam return at 50 m, 26% at 80 m. A 30° mount is at 61% by 50 m.
> - The far beam edge sits at 28° depression, comfortably above the ~20° grazing angle where σ⁰
>   starts to collapse. A 30° mount puts its far edge at 18° — into the bad-σ⁰ region.
> - It matches the constants already baked into your codebase (`TiltMount.theta_tilt_deg = 40.0`
>   at line 113, `self_test()` at line 726), so the existing self-tests stay meaningful.
> - 40° is trivially fabricable and measurable with a digital inclinometer.
>
> **Revisit only if TEST-01 says otherwise:**
> - measured `R_gnd < 90 m` and ceiling ≤ 30 m → **use 35°**
> - measured `R_gnd < 90 m` and ceiling 50 m → **use 50°**
> - you need 80 m+ AGL → **use 55–60°**, and accept DOP_Vx ≈ 7.3 (37% worse forward velocity)

### 2.8 TABLE E — The `force_2d` bias leak (this is why B5 is red)

If you constrain `Vz = 0` while the true vertical velocity is `w`, least-squares dumps the
unmodelled signal into the remaining axes with gain `β = (A₂ᵀA₂)⁻¹A₂ᵀa₃`:

| tilt | β_x | phantom Vx during a **2 m/s climb** | phantom position error over a 30 s climb |
|---:|---:|---:|---:|
| 20° | 0.357 | 0.71 m/s | 21 m |
| 30° | 0.564 | 1.13 m/s | 34 m |
| 35° | 0.682 | 1.36 m/s | 41 m |
| **40°** | **0.814** | **1.63 m/s** | **49 m** |
| 45° | 0.965 | 1.93 m/s | 58 m |
| 60° | 1.614 | 3.23 m/s | 97 m |
| 90° | 0.000 | 0.00 m/s | 0 m |

At your chosen 40° tilt, a routine 2 m/s climb to altitude injects **1.63 m/s of completely
fictitious forward velocity**, which the EKF integrates into **49 m of position error in 30
seconds**. The vehicle will believe it has flown 49 m forward while hovering in place, and will
fly 49 m backwards to "correct." `force_2d` must be removed from the flight path. Full stop.

(Note β_x = 0 at 90°: nadir is the only tilt where forcing 2D is harmless — because at nadir
Vx and Vz are orthogonal, ρ = 0. This is a nadir-only optimisation that was applied to a tilted
sensor.)

### 2.9 TABLE G/H/I — Error budget: what actually causes drift

`σ_vx = σ_doppler · DOP_Vx / √N`. With `σ_doppler = 0.05 m/s`, `DOP_Vx = 5.3`, `N = 120 points`:

```
σ_vx = 0.05 × 5.3 / √120 = 0.024 m/s
random-walk position error over 60 s at 20 Hz = 0.024 × √(60 × 0.05) = 0.04 m
```

**Random noise is irrelevant.** 4 cm in a minute. Drift is entirely a *bias* problem:

**Yaw error — the dominant term:**

| yaw error | 100 m leg | 250 m leg | 500 m leg | 1000 m leg |
|---:|---:|---:|---:|---:|
| 0.5° | 0.9 m | 2.2 m | 4.4 m | 8.7 m |
| 1.0° | 1.7 m | 4.4 m | 8.7 m | 17.5 m |
| **2.0°** | 3.5 m | 8.7 m | **17.5 m** | **34.9 m** |
| 5.0° | 8.7 m | 21.9 m | 43.7 m | 87.5 m |

**Tilt-misalignment error:**

| Δθ mount error | Vz leak at 10 m/s fwd | false altitude change over 60 s |
|---:|---:|---:|
| 0.5° | 0.09 m/s | 5.2 m |
| 1.0° | 0.17 m/s | 10.5 m |
| **2.0°** | **0.35 m/s** | **20.9 m** |
| 5.0° | 0.87 m/s | 52.3 m |

> **Two engineering conclusions that drive the whole build:**
> 1. **Measure the mount angle to better than 0.5°** with a digital inclinometer, on the actual
>    airframe, with the actual landing gear, at the actual level attitude. Then feed the measured
>    value, not "40", to `--tilt-deg`. A 2° error is 21 m of phantom altitude per minute.
> 2. **Yaw is your #1 error source, not velocity.** Budget 4–8 m of cross-track per 500 m leg
>    even with an excellent compass. This is why RTH must return along the outbound breadcrumb
>    (re-observing mapped terrain so GICP can correct yaw) rather than flying a straight line home.

---

## 3. Physical mounting specification

### 3.1 U300 nose radar

```
                    ┌─────────────┐
                    │ Cube Orange │  ← body origin (FC IMU), levelled airframe
                    └──────┬──────┘
        X_body (fwd) ──────┼──────────────────────►
                           │                    ╱
                           │  lever arm   ╔════╗  ← U300, boresight 40° below
                           │  [Lx,Ly,Lz]  ║U300║     the body X axis
                           ▼ Z_body(down) ╚════╝
                                              ╲ 40°
                                               ╲
                                                ▼ boresight
```

| Requirement | Spec | Why |
|---|---|---|
| **Tilt** | 40.0° nose-down from the body X axis, measured with the airframe levelled | §2.7 |
| **Tilt tolerance** | ±0.5° mechanical; then **measure** and pass the measured value | §2.9 Table I |
| **Roll/yaw misalignment** | < 0.5°; boresight must lie in the body XZ plane | yaw error → cross-track (Table H) |
| **Clear FOV** | No prop disc, landing gear, antenna, or payload inside ±45° az × ±17° el of boresight (5° margin on the ±40°/±12° FOV) | any blockage biases the LOS distribution and skews the velocity solve |
| **Prop separation** | ≥ 0.5 m from any rotor tip in the beam; raise `--leakage-radius` to 0.8 m if not achievable | §12.2 |
| **Radome** | ABS / PC / PTFE only. **No carbon fibre anywhere in the beam** — CF is conductive and opaque at 77 GHz | |
| **Radome standoff** | λ/2 multiple at 79 GHz = **1.9 mm steps**; a 3.8 mm or 5.7 mm air gap. Avoid arbitrary spacing (standing waves → phantom near-field returns) | |
| **Vibration** | Soft-mount on 3M VHB or silicone grommets. FMCW chirp integration over 50 ms smears under >50 Hz vibration | |
| **EMI** | ≥ 100 mm from ESCs and power leads; route the UART away from motor phases; ferrite on the UART if the ESCs are >30 A | |
| **Thermal** | −40 to +85 °C rated; but do not bury it behind a hot ESC stack — FMCW frequency stability drifts with die temp | |

### 3.2 U200A belly altimeter

| Requirement | Spec |
|---|---|
| **Orientation** | Straight down (0° from body −Z), ±2° |
| **Placement** | As close to the FC as the airframe allows; log the lever arm (§14) |
| **Clear FOV** | Nothing below it: no landing gear leg, no battery strap, no payload. A leg in the beam gives a constant false 0.3 m reading |
| **Min-range** | ≥ 0.2 m from the lowest point of the landing gear, so the gear does not clip the near gate at touchdown |
| **Interface** | TTL/UART → Jetson (recommended, keeps one clock domain). CAN → Cube Orange is an option but then you lose the Jetson-side log |

### 3.3 Cable and port map

| Device | Port | Baud |
|---|---|---|
| U300 nose radar | `/dev/ttyUSB0` (pin with a udev rule → `/dev/radar_nose`) | 921600 |
| U200A altimeter | `/dev/ttyUSB1` (→ `/dev/radar_alt`) | per Linpowave (likely 115200) |
| Cube Orange | `/dev/ttyACM0` (→ `/dev/cube`) — **USB for bench, TELEM2 serial for flight** | 115200 / 921600 |
| GPS (ground truth) | Cube's own GPS, read via MAVLink `GPS_RAW_INT` — **or** a separate NEO-M8N on `/dev/ttyUSB2` |

> **Pin the device names.** Two USB-serial adapters will swap enumeration order between boots and
> you will send altimeter bytes to the radar parser. Write `/etc/udev/rules.d/99-rio.rules`:
> ```
> SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{serial}=="AAAA", SYMLINK+="radar_nose"
> SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{serial}=="BBBB", SYMLINK+="radar_alt"
> SUBSYSTEM=="tty", ATTRS{idVendor}=="2dae", SYMLINK+="cube"
> ```

---

## 4. Frame conventions contract — read this before touching any code

Half the defects in Section 5 exist because there is no single written statement of what frame
each number is in. Here it is. **Paste this as a docstring at the top of `nav_node.py`.**

| Frame | Axes | Where it appears |
|---|---|---|
| **Radar native** | X=lateral(right), Y=forward, Z=up | raw Points_float TLV |
| **Body (FRD)** | X=forward, Y=right, Z=**down** | output of `TiltMount.to_body()`, `v_body` in every RIO packet |
| **Map / nav** | X=forward-at-init, Y=right-at-init, Z=**down** | `slam_node.T_world`, `PoseState.position_nav` |
| **NED** | X=North, Y=East, Z=**down** | `MAV_FRAME_LOCAL_NED` setpoints, ArduPilot EKF local frame |

**The map frame is NOT ENU.** `nav_node.py`'s module docstring (lines 25–30) claims
"right-handed ENU (x=East, y=North, z=Up)". That is **wrong**, and the rest of the same file
contradicts it: `_check_geofence` computes `alt_agl = -p[2]` (Z-down) at line 452, and
`parse_waypoints` negates Z at line 593 with the comment "internal map frame is NED (Z-Down)".
The code is right; the docstring is wrong. Fix the docstring (change **N-01**) or someone will
eventually "fix" the code to match it and invert your altitude limit.

**Map → NED requires a yaw rotation.** The map frame's X axis points wherever the nose was
pointing at SLAM bootstrap. NED's X axis points North. The rotation between them is
`yaw_offset = attitude.yaw − slam_yaw`, computed in `on_slam_pose` (line 217) — **and
currently never executed**, because the call site at line 425 omits the `attitude` argument.
See defect **B3 / change N-03**.

---

## 5. Exact code changes

Line numbers are from the archive you uploaded. Apply in the order given; later line numbers
shift as you insert, so work **bottom-up within each file** or re-grep before each edit.

---

### 5.1 `src/nav_node.py`

#### **N-01 — line 296 — 🔴 `type_mask` bits inverted (defect B1)**

The comment states the intent correctly; the binary literal does the opposite.

```python
# CURRENT (line 296):
        type_mask = 0b0000_100_111_000_111
```

Decoding `0b0000100111000111` = `0x09C7` = 2503:

| bit | field | value | effect |
|---|---|---|---|
| 0–2 | x,y,z | 1,1,1 | ignore position ✅ |
| 3–5 | vx,vy,vz | 0,0,0 | use velocity ✅ |
| 6–8 | afx,afy,afz | 1,1,1 | ignore accel ✅ |
| 9 | force | 0 | — |
| **10** | **yaw** | **0** | **USE yaw** ❌ |
| **11** | **yaw_rate** | **1** | **IGNORE yaw_rate** ❌ |

The call then passes `yaw = 0` as the 13th positional argument (line 303). **Every setpoint,
at 10 Hz, commands the aircraft to yaw to heading 0 (North).** With a nose-mounted radar this
means: the moment you enter GUIDED, the vehicle spins to North, the SLAM map is invalidated by
an uncommanded yaw slew, `yaw_offset` (already broken by B3) goes further out, and the forward
obstacle cone points somewhere unrelated to the direction of travel.

```python
# NEW (line 296):
        # POSITION_TARGET_TYPEMASK: 1 = IGNORE that field.
        #   bits 0-2  position   -> 1,1,1 ignore
        #   bits 3-5  velocity   -> 0,0,0 USE
        #   bits 6-8  accel      -> 1,1,1 ignore
        #   bit  9    force flag -> 0
        #   bit 10    yaw        -> 1 ignore   (we do NOT command absolute heading)
        #   bit 11    yaw_rate   -> 0 USE
        # = 0b0000_0101_1100_0111 = 0x05C7 = 1479
        type_mask = 0b0000_0101_1100_0111   # 1479: velocity + yaw_rate
```

> **Regression guard — add to `tests/sitl_test.py`:**
> ```python
> assert type_mask & (1 << 10), "yaw must be IGNORED"
> assert not (type_mask & (1 << 11)), "yaw_rate must be USED"
> assert not (type_mask & 0b111000), "velocity bits must be USED"
> assert (type_mask & 0b111) == 0b111, "position bits must be IGNORED"
> ```
> If you ever want velocity-only with no heading control at all, the value is
> `0b0000_1101_1100_0111` = 3527 (ignore both yaw and yaw_rate).

---

#### **N-02 — lines 364–366 — 🔴 `-1.0` obstacle sentinel (defect B2, half 1)**

```python
# CURRENT (lines 364-366):
    if not math.isfinite(fwd_range_m):
        return v_cmd_nav, False
    if fwd_range_m <= cfg.obstacle_hard_stop_m:
```

`slam_node.py:713` converts "nothing in the cone" (`+inf`) into the wire value `-1.0`.
`math.isfinite(-1.0)` is `True`, so the guard does not fire, and `-1.0 <= 5.0` is `True`.
**Result: the vehicle hard-stops whenever the path ahead is completely clear**, and only
moves when something is 5–15 m in front of it. The mission will never progress past waypoint 0.

```python
# NEW (lines 364-366):
    # slam_node.py sends -1.0 as the "nothing detected in the forward cone"
    # sentinel (it cannot put +inf on the wire). Treat any non-positive value
    # as "clear", NOT as a 1-metre obstacle.
    if (not math.isfinite(fwd_range_m)) or fwd_range_m <= 0.0:
        return v_cmd_nav, False
    if fwd_range_m <= cfg.obstacle_hard_stop_m:
```

#### **N-02b — `slam_node.py` line 713 — 🔴 (defect B2, half 2)**

Belt and braces: make the sentinel unambiguous on the transmit side too.

```python
# CURRENT (slam_node.py:713):
                    fwd_send = fwd if math.isfinite(fwd) else -1.0  # -1.0 = "nothing in cone"

# NEW:
                    # Sentinel must be unmistakably "not an obstacle". -1.0 was
                    # being read downstream as a 1 m range. Use a large positive
                    # value that is beyond any braking threshold.
                    fwd_send = fwd if math.isfinite(fwd) else 1.0e6  # 1e6 m = "nothing in cone"
```

> Also apply the same treatment in `nav_node.py:547` (`if fwd_range > self.cfg.obstacle_brake_start_m`)
> — with the fix above it now behaves correctly, but add an explicit comment so nobody
> re-introduces the sentinel confusion.

---

#### **N-03 — line 425 — 🔴 `yaw_offset` never computed (defect B3)**

```python
# CURRENT (line 425):
                self.tracker.on_slam_pose(t, T, fwd_range)
```

`on_slam_pose`'s signature (line 211) takes an optional `attitude`; without it, the
`yaw_offset` update at line 217 never runs and **`self.yaw_offset` stays 0.0 for the entire
flight.** Consequences:

- Line 535–541 rotates the map-frame velocity command into NED using `yaw_offset = 0`, i.e.
  **identity**. This is only correct if the aircraft's SLAM bootstrap heading was exactly North.
- Line 244 computes `nav_yaw = attitude.yaw - 0 = attitude.yaw`, so RIO dead-reckoning
  integrates in **true NED**, while SLAM writes `position_nav` in the **map frame**.
  Two different frames are being written into the same variable, 20 times a second.
- Take off facing East and command "fly 20 m along map +X": the vehicle flies 20 m **North**.
  That is a flyaway.

(This is a half-applied patch: `patch_nav_node.py` added the `attitude` parameter to the
function but never updated the caller.)

```python
# NEW (lines 418-441, restructure _drain_sockets):
    def _drain_sockets(self):
        # Fetch attitude FIRST so both the pose and the RIO handlers see the
        # same sample. This is the source of roll/pitch/yaw used to (a) resolve
        # the map->NED yaw offset and (b) rotate RIO body velocity into nav.
        att = self.attitude_listener.get() if self.attitude_listener else None

        try:
            while True:
                data, _ = self.pose_sock.recvfrom(2048)
                if len(data) < POSE_PKT_HDR.size + 128:
                    continue                      # short/truncated packet, drop it
                t, n_map, fwd_range = POSE_PKT_HDR.unpack_from(data, 0)
                T = np.frombuffer(data, dtype='<f8', count=16,
                                   offset=POSE_PKT_HDR.size).reshape(4, 4)
                self.tracker.on_slam_pose(t, T, fwd_range, att)   # <-- att was missing
        except BlockingIOError:
            pass
        except struct.error:
            pass                                   # malformed packet, drop it

        try:
            while True:
                data, _ = self.rio_sock.recvfrom(64)
                if len(data) < RIO_PKT.size:
                    continue
                t, vx, vy, vz, _n, _cxx, _cyy, _czz = RIO_PKT.unpack(data[:RIO_PKT.size])
                self.tracker.on_rio_velocity(t, np.array([vx, vy, vz]), att)
        except BlockingIOError:
            pass
        except struct.error:
            pass
```

Add `import struct` is already present (line 36). ✅

> **Additional hardening — `PoseTracker.on_slam_pose` (line 216).**
> Latch `yaw_offset` **once**, at SLAM bootstrap, instead of re-estimating it on every keyframe.
> If GICP's yaw wanders, a continuously-updated offset lets the map↔NED relationship rotate
> under the controller. Change line 216–217 to:
> ```python
>         if attitude is not None and attitude.valid and not self._yaw_offset_locked:
>             self.yaw_offset = attitude.yaw - slam_yaw
>             self._yaw_offset_locked = True
>             print(f"[nav_node] yaw_offset LATCHED at {math.degrees(self.yaw_offset):+.1f} deg")
> ```
> and initialise `self._yaw_offset_locked = False` in `__init__` (line 206).

---

#### **N-04 — lines 452–458 — 🔴 altitude limit reads a drifting SLAM Z**

```python
# CURRENT (line 452):
        alt_agl = -p[2]
```

`p` is `position_nav`, whose Z comes from GICP + `_gravity_correct`, which has no absolute
reference and is the source of the documented **191 m of Z drift in 96 s**. Your geofence, your
`min_altitude_agl_m = 2.0` floor, and your `max_altitude_agl_m` ceiling are all being evaluated
against a number that can be 100 m wrong. **Once the U200A is wired in, AGL must come from the
altimeter, not from SLAM.**

```python
# NEW: add to NavConfig (after line 166)
    alt_source_stale_timeout_s: float = 0.5   # altimeter staleness gate

# NEW: add to PoseState (after line 186)
    agl_m: float = float('nan')          # absolute AGL from U200A belly radar
    t_agl_local: float = 0.0

# NEW: add to PoseTracker (after line 253)
    def on_altimeter(self, agl_m: float, attitude: AttitudeState | None):
        """Absolute AGL from the U200A belly radar, tilt-compensated.
        The belly radar measures SLANT range along body -Z; at a roll/pitch
        attitude the true vertical height is range * cos(roll) * cos(pitch).
        Over ~15 deg of tilt this is a 3.4% correction -- 1.7 m at 50 m AGL."""
        if attitude is not None and attitude.valid:
            agl_m = agl_m * math.cos(attitude.roll) * math.cos(attitude.pitch)
        self.pose.agl_m = agl_m
        self.pose.t_agl_local = time.monotonic()

    def agl(self) -> float:
        """Returns tilt-compensated AGL, or NaN if the altimeter is stale."""
        if not math.isfinite(self.pose.agl_m):
            return float('nan')
        if time.monotonic() - self.pose.t_agl_local > self.cfg.alt_source_stale_timeout_s:
            return float('nan')
        return self.pose.agl_m

# NEW (replacing lines 452-458 of _check_geofence):
        alt_agl = self.tracker.agl()
        if not math.isfinite(alt_agl):
            # No trustworthy absolute height. SLAM Z is NOT an acceptable
            # fallback for a safety limit (documented 191 m drift in 96 s).
            print("[nav_node] ALTIMETER STALE -- no trustworthy AGL, failing geofence")
            return False
        if alt_agl > self.cfg.max_altitude_agl_m:
            print(f"[nav_node] ALTITUDE limit breach: agl={alt_agl:.1f}m > "
                  f"{self.cfg.max_altitude_agl_m}m")
            return False
        if self.state == NavState.MISSION and alt_agl < self.cfg.min_altitude_agl_m:
            print(f"[nav_node] ALTITUDE limit breach: agl={alt_agl:.1f}m < "
                  f"{self.cfg.min_altitude_agl_m}m")
            return False
        return True
```

Then bind the altimeter socket in `NavNode.__init__` (after line 411) and drain it in
`_drain_sockets`:

```python
# NavNode.__init__, after line 411:
        self.alt_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.alt_sock.bind((alt_ip, alt_port))       # default 127.0.0.1:5031
        self.alt_sock.setblocking(False)

# _drain_sockets, after the RIO block:
        try:
            while True:
                data, _ = self.alt_sock.recvfrom(32)
                if len(data) >= 4:
                    (agl,) = struct.unpack('<f', data[:4])
                    self.tracker.on_altimeter(agl, att)
        except BlockingIOError:
            pass
```

---

#### **N-05 — line 620–622 — 🔴 altitude ceiling far above the physical limit**

```python
# CURRENT (NavConfig line 165):
    max_altitude_agl_m: float = 200.0
# CURRENT (CLI line 616):
    p.add_argument('--max-altitude', type=float, default=50.0)
```

Section 2.6 proves 200 m is physically unreachable. Two different defaults (200 in the
dataclass, 50 on the CLI) is itself a trap.

```python
# NEW (NavConfig line 165):
    # HARD CEILING. See implementation_plan.md Section 2.6: ground returns
    # vanish above R_gnd, and R_gnd for diffuse terrain is ~120 m, not the
    # datasheet's 350 m (which is a high-RCS point-target figure).
    # 50 m is the safe value for a 40 deg tilt with R_gnd = 120 m.
    # DO NOT raise this without re-running TEST-01 at the new altitude.
    max_altitude_agl_m: float = 50.0

# NEW (CLI line 616):
    p.add_argument('--max-altitude', type=float, default=50.0,
                    help="HARD AGL ceiling. Raising this above the value justified by "
                         "your measured R_gnd (see implementation_plan.md Sec 2.6) will "
                         "silently put the aircraft in a zone where RIO publishes nothing.")
```

---

#### **N-06 — line 348–356 — 🟠 no separate vertical speed limit**

```python
# CURRENT (line 355):
    speed = min(max_speed, max_speed * min(1.0, dist / 3.0))
    return (err / dist) * speed
```

The 3-vector is capped by a single `max_speed`, so a waypoint 20 m above the vehicle produces
a 2 m/s climb command. At 40° tilt a 2 m/s climb is exactly the case that Table E shows
poisons Vx — and it also punches the beam out of range faster. Split the limits.

```python
# NEW:
def velocity_toward(current: np.ndarray, target: np.ndarray, max_speed: float,
                    max_climb: float = 1.0, max_descend: float = 0.7) -> np.ndarray:
    """Proportional guidance with SEPARATE horizontal and vertical speed caps.
    Vertical is capped much harder than horizontal because (a) at a 40 deg
    tilt the Vx/Vz correlation is -0.964, so fast vertical motion degrades the
    horizontal solution, and (b) climbing pushes the ground out of the radar's
    usable range (Sec 2.5). Descend is capped tighter still -- vortex ring
    state plus the altimeter's 0.2 m min range at touchdown."""
    err = target - current
    err_h = err[:2]
    dist_h = float(np.linalg.norm(err_h))
    v = np.zeros(3)
    if dist_h > 1e-6:
        speed_h = min(max_speed, max_speed * min(1.0, dist_h / 3.0))
        v[:2] = (err_h / dist_h) * speed_h
    # Z is DOWN-positive in the map frame; err[2] > 0 means the target is below us.
    dz = err[2]
    vz_cap = max_descend if dz > 0 else max_climb
    v[2] = float(np.clip(dz * 0.5, -vz_cap, vz_cap))   # P-controller, capped
    return v

# and in NavConfig, after line 153:
    max_climb_mps: float = 1.0
    max_descend_mps: float = 0.7

# and at the call site (line 529):
            v_target = velocity_toward(pos, target, self.cfg.max_speed_mps,
                                       self.cfg.max_climb_mps, self.cfg.max_descend_mps)
```

---

#### **N-07 — lines 493–497 — 🟠 RTL is delegated to an autopilot with no position source**

```python
# CURRENT (lines 493-497):
        if self.state == NavState.FAILSAFE_RTL:
            if not self._rtl_sent:
                self.ap.request_rtl()
                self._rtl_sent = True
            return  # stop sending our own setpoints; autopilot owns it now
```

ArduPilot's RTL flies to `home` using the EKF's position estimate. In a GPS-denied
configuration with `EK3_SRC1_VELXY = 6` (ExternalNav velocity) and no position source, the
EKF's horizontal position is **pure integration of your velocity** — the very thing that is
drifting. Handing over to RTL at the moment your estimator has become untrustworthy is the
worst possible time to trust it.

Worse: `FAILSAFE_RTL` is entered when **pose is lost** (line 484). So the trigger is "we no
longer know where we are" and the response is "fly home using where we think we are."

**Replace with a three-tier failsafe ladder.** Add to `NavState`:

```python
class NavState(Enum):
    INIT = auto()
    WAIT_FOR_POSE = auto()
    HOLD = auto()
    MISSION = auto()
    OBSTACLE_STOP = auto()
    RTH = auto()            # NEW: our own breadcrumb return, we still own the setpoints
    FAILSAFE_HOLD = auto()  # NEW: brief pose loss -> zero velocity, wait for recovery
    FAILSAFE_LAND = auto()  # NEW: pose unrecoverable -> descend on altimeter alone
    LANDING = auto()
    DISARMED = auto()
```

Replace lines 483–500 with:

```python
        # ---- Failsafe ladder (see implementation_plan.md Sec 8.3) ----
        # Tier 1: pose briefly stale -> stop moving, hold, keep trying.
        # Tier 2: pose lost longer than pose_lost_rth_timeout_s but the
        #         estimator is still healthy -> fly the breadcrumb home.
        # Tier 3: EKF unhealthy OR pose unrecoverable -> descend vertically
        #         on the altimeter. This is the ONLY action that does not
        #         require a trustworthy horizontal position.
        ekf_ok = self.ap.ekf_healthy()
        flying = self.state in (NavState.HOLD, NavState.MISSION,
                                NavState.OBSTACLE_STOP, NavState.RTH)

        if flying and not ekf_ok:
            print("[nav_node] EKF UNHEALTHY -- descending in place on altimeter")
            self._enter(NavState.FAILSAFE_LAND)

        if flying and stale > self.cfg.pose_lost_land_timeout_s:
            print(f"[nav_node] POSE LOST {stale:.1f}s -- descending in place")
            self._enter(NavState.FAILSAFE_LAND)
        elif flying and stale > self.cfg.pose_lost_rth_timeout_s \
                and self.state != NavState.RTH:
            print(f"[nav_node] POSE STALE {stale:.1f}s -- starting breadcrumb RTH")
            self._begin_rth()

        if self.state == NavState.FAILSAFE_LAND:
            # Pure vertical descent using the belly altimeter. No horizontal
            # command at all -- we have explicitly decided we do not trust our
            # horizontal estimate, so we must not act on it.
            agl = self.tracker.agl()
            if math.isfinite(agl) and agl < 0.4:
                self.ap.send_velocity_setpoint(0, 0, 0)
                self.ap.arm(False)
                self._enter(NavState.DISARMED)
                return
            vz = self.cfg.failsafe_descend_mps     # +down in NED
            if math.isfinite(agl) and agl < 5.0:
                vz = 0.3                            # slow final
            self.ap.send_velocity_setpoint(0.0, 0.0, vz)
            return
```

and add to `NavConfig`:

```python
    pose_lost_rth_timeout_s: float = 2.0
    pose_lost_land_timeout_s: float = 6.0
    failsafe_descend_mps: float = 0.8
```

#### **N-07b — EKF health monitor (new)**

`nav_node.py` never reads `EKF_STATUS_REPORT`. Add to `AttitudeListener.run()` (line 110) so
one thread reads both messages:

```python
    def run(self):
        while not self._stop.is_set():
            msg = self._conn.recv_match(type=['ATTITUDE', 'EKF_STATUS_REPORT'],
                                        blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == 'ATTITUDE':
                with self._lock:
                    self.state.roll, self.state.pitch, self.state.yaw = \
                        msg.roll, msg.pitch, msg.yaw
                    self.state.t_local = time.monotonic()
                    self.state.valid = True
            else:  # EKF_STATUS_REPORT
                with self._lock:
                    # ArduPilot: variances are normalised; >1.0 means the EKF
                    # is rejecting/struggling with that measurement class.
                    self.ekf_vel_var = msg.velocity_variance
                    self.ekf_pos_var = msg.pos_horiz_variance
                    self.ekf_hgt_var = msg.pos_vert_variance
                    self.ekf_flags = msg.flags
                    self.ekf_t = time.monotonic()

    def ekf_healthy(self, var_limit: float = 0.9) -> bool:
        with self._lock:
            if time.monotonic() - getattr(self, 'ekf_t', 0.0) > 3.0:
                return False                      # no report = not healthy
            return (self.ekf_vel_var < var_limit and
                    self.ekf_pos_var < var_limit and
                    self.ekf_hgt_var < var_limit)
```

and expose it via `Autopilot.ekf_healthy()` → delegate to the listener.

---

#### **N-08 — line 552–558 — 🟠 `start_mission()` arms from the ground**

```python
# CURRENT (lines 552-558):
    def start_mission(self):
        if self.state != NavState.HOLD:
            ...
        self.ap.offboard_or_guided_mode()
        self.ap.arm(True)
        self._enter(NavState.MISSION)
```

Arming in GUIDED and immediately sending velocity setpoints is the highest-risk possible entry.
There is no takeoff, no altitude check, and no confirmation the EKF has converged.

**Do not automate takeoff for first flights.** The correct sequence is: safety pilot takes off
manually in `LOITER`/`ALT_HOLD`, climbs to the test altitude, confirms a stable radar-only
hover, *then* flips to `GUIDED` and the companion takes over.

```python
# NEW:
    def start_mission(self):
        if self.state != NavState.HOLD:
            print(f"[nav_node] refusing to start mission from state {self.state.name}")
            return
        if not self.ap.is_armed():
            print("[nav_node] REFUSING: vehicle is not armed. Take off manually in "
                  "LOITER/ALT_HOLD first, then issue 'go'. This script does not arm.")
            return
        agl = self.tracker.agl()
        if not math.isfinite(agl) or agl < self.cfg.min_mission_start_agl_m:
            print(f"[nav_node] REFUSING: AGL={agl} is not a valid airborne altitude "
                  f"(need > {self.cfg.min_mission_start_agl_m} m from the belly radar).")
            return
        if not self.ap.ekf_healthy():
            print("[nav_node] REFUSING: EKF not healthy.")
            return
        if self.tracker.stale_for() > self.cfg.pose_stale_timeout_s:
            print("[nav_node] REFUSING: SLAM pose is stale.")
            return
        self.ap.offboard_or_guided_mode()      # mode change only. NO arm().
        self.home_map = self.tracker.pose.position_nav.copy()
        self.breadcrumbs = [self.home_map.copy()]
        self._enter(NavState.MISSION)
```

Add `min_mission_start_agl_m: float = 3.0` to `NavConfig`, and implement
`Autopilot.is_armed()` by latching `HEARTBEAT.base_mode & MAV_MODE_FLAG_SAFETY_ARMED`.

**Delete `self.ap.arm(True)` entirely from this file.** Arming is the safety pilot's job.

---

#### **N-09 — module docstring lines 25–30 — 🟡 wrong frame documented**

```python
# CURRENT (lines 25-30):
Local navigation frame: right-handed ENU (x=East, y=North, z=Up), all
waypoints and the position estimate expressed relative to the pose the
vehicle had when nav_node started ...
```

Replace with the contract from Section 4 of this plan. This is a documentation-only change but
it is the one that prevents the next person from "fixing" `alt_agl = -p[2]`.

---

#### **N-10 — line 584–594 — 🟡 waypoint sign convention is a trap**

```python
# CURRENT (line 593):
        wps.append(Waypoint(x, y, -z))  # Z is UP in user input, but internal map frame is NED
```

Correct, but silent. Print the interpretation at startup so a sign error is caught on the
ground and not at 40 m:

```python
# NEW (after line 593):
    print("[nav_node] waypoints parsed (user input is X-fwd, Y-right, Z-UP metres "
          "relative to SLAM bootstrap pose):")
    for i, w in enumerate(wps):
        print(f"           WP{i}: fwd={w.x:+7.1f}  right={w.y:+7.1f}  "
              f"alt={-w.z:+7.1f} (map Z={w.z:+7.1f}, down-positive)")
```

---

#### **N-11 — new — 🟠 breadcrumb RTH**

`request_rtl()` is not a return-to-home for this system (N-07). Implement RTH as a reverse
traverse of the outbound path. This is not just a safety measure — flying back over terrain
the SLAM map already contains gives GICP real correspondences, which **corrects accumulated
yaw error** and is the single best drift mitigation available to you.

```python
# In NavNode.__init__:
        self.home_map = None
        self.breadcrumbs = []          # list of map-frame positions
        self.rth_index = 0

# In tick(), inside the MISSION branch, after the position is read:
            # Drop a breadcrumb every crumb_spacing_m of travel.
            if not self.breadcrumbs or \
               np.linalg.norm(pos - self.breadcrumbs[-1]) > self.cfg.crumb_spacing_m:
                self.breadcrumbs.append(pos.copy())

# New method:
    def _begin_rth(self):
        """Return home by REVERSING the outbound breadcrumb trail, not by
        flying a straight line. Two reasons:
          1. The outbound path is known-flyable and known-obstacle-free.
          2. It re-observes terrain already in the SLAM map, which lets GICP
             find correspondences and correct the yaw drift that accumulated
             on the way out. A straight-line return flies over unmapped
             ground with a heading error you cannot detect."""
        if not self.breadcrumbs:
            print("[nav_node] RTH requested with no breadcrumbs -- landing in place.")
            self._enter(NavState.FAILSAFE_LAND)
            return
        self.rth_index = len(self.breadcrumbs) - 1
        print(f"[nav_node] RTH: retracing {len(self.breadcrumbs)} breadcrumbs "
              f"back to origin.")
        self._enter(NavState.RTH)

# New tick() branch:
        if self.state == NavState.RTH:
            if self.rth_index < 0:
                print("[nav_node] RTH complete -- over home, handing to LAND.")
                self._enter(NavState.FAILSAFE_LAND)
                return
            target = self.breadcrumbs[self.rth_index]
            if np.linalg.norm(target - pos) < self.cfg.waypoint_accept_radius_m:
                self.rth_index -= 1
                return
            v_target = velocity_toward(pos, target, self.cfg.rth_speed_mps,
                                       self.cfg.max_climb_mps, self.cfg.max_descend_mps)
            v_target, hard_stop = apply_obstacle_scaling(v_target, target - pos,
                                                          fwd_range, self.cfg)
            v_cmd = self.slew.step(v_target, time.monotonic())
            self._send_map_velocity_as_ned(v_cmd, target - pos)
            return
```

Add `crumb_spacing_m: float = 5.0` and `rth_speed_mps: float = 1.5` to `NavConfig`.

---

#### **N-12 — lines 535–542 — 🟠 factor out the map→NED rotation and add yaw-rate**

The rotation is inlined once and will drift out of sync with the RTH branch. Extract it, and
add the yaw-rate command that N-01 just enabled.

```python
    def _send_map_velocity_as_ned(self, v_cmd_map: np.ndarray,
                                   heading_map: np.ndarray | None = None):
        """Rotate a map-frame velocity into NED and send it, plus a yaw-rate
        command that points the NOSE along the direction of travel.

        Pointing the nose along track is not cosmetic. The U300 is nose-mounted:
        the forward obstacle cone, and the best-conditioned part of the ground
        patch, are both ahead of the aircraft. Flying sideways or backwards
        means braking on a cone that is not looking where you are going."""
        cy, sy = math.cos(self.tracker.yaw_offset), math.sin(self.tracker.yaw_offset)
        v_ned = np.array([
            v_cmd_map[0] * cy - v_cmd_map[1] * sy,
            v_cmd_map[0] * sy + v_cmd_map[1] * cy,
            v_cmd_map[2],
        ])

        yaw_rate = 0.0
        if heading_map is not None and np.linalg.norm(heading_map[:2]) > 1.0:
            desired_map_yaw = math.atan2(heading_map[1], heading_map[0])
            att = self.attitude_listener.get() if self.attitude_listener else None
            if att is not None and att.valid:
                current_map_yaw = att.yaw - self.tracker.yaw_offset
                err = math.atan2(math.sin(desired_map_yaw - current_map_yaw),
                                  math.cos(desired_map_yaw - current_map_yaw))
                yaw_rate = float(np.clip(
                    err * 0.8,
                    -math.radians(self.cfg.yaw_rate_max_dps),
                     math.radians(self.cfg.yaw_rate_max_dps)))
        self.ap.send_velocity_setpoint(*v_ned, yaw_rate_rad_s=yaw_rate)
```

Then replace lines 535–542 with `self._send_map_velocity_as_ned(v_cmd, target - pos)`.

> ⚠️ **Yaw-rate limit.** `yaw_rate_max_dps = 30.0` is aggressive for a radar SLAM front end.
> A fast yaw slews the whole point cloud between keyframes and breaks GICP correspondence.
> **Reduce to 15 °/s** and, if you see keyframe rejections during turns, to 10 °/s.

---

### 5.2 `src/doppler_rio.py`

#### **D-01 — lines 560, 574, 585 — 🔴 remove `force_2d` from the flight path (defect B5)**

```python
# CURRENT (line 560):
            force_2d=(attitude is not None and self.imu_level_points))
# CURRENT (line 574):
            v_seed, _ = weighted_refit(u_body, v_adjusted, ranges, mask,
                                       force_2d=(attitude is not None and self.imu_level_points))
# CURRENT (line 585):
                force_2d=(attitude is not None and self.imu_level_points))
```

Three problems, in increasing order of severity:

1. **`force_2d` is not a flag, it is a side effect.** There is no `--force-2d` option. Turning
   on IMU levelling silently turns on 2D mode. Nothing in the log says so.
2. **It zeroes an observable state.** At 40° tilt, `DOP_Vz = 6.30` — vertical velocity is
   perfectly observable, just less well than lateral. There is no observability argument for
   discarding it.
3. **It corrupts the states it keeps.** §2.8: `β_x = 0.814` at 40°. A 2 m/s climb becomes
   1.63 m/s of phantom forward velocity, integrating to **49 m in 30 s**.

Additionally, `weighted_refit` line 360 and `irls_refit` line 463 set `cov[2,2] = 1.0` in 2D
mode. `mavlink_bridge.py` forwards that as the Z-velocity covariance. **The result is a
`VISION_SPEED_ESTIMATE` message telling EKF3 "vz = 0.0 ± 1.0 m/s" at 20 Hz during every climb
and descent.** The EKF will fight the real accelerometer-derived climb rate and the height
state will oscillate.

**Fix: hard-disable 2D mode for flight.**

```python
# NEW (line 560):
            # force_2d is a NADIR-ONLY optimisation and is disabled for flight.
            # At the 40 deg nose tilt, rho(Vx,Vz) = -0.964, so constraining
            # Vz = 0 leaks 0.814 m/s of phantom Vx per 1 m/s of true climb
            # (implementation_plan.md Sec 2.8). Vz is fully observable at this
            # tilt (DOP_Vz = 6.3) and MUST be solved for. See defect B5.
            force_2d=self.force_2d)
# NEW (line 574):
            v_seed, _ = weighted_refit(u_body, v_adjusted, ranges, mask,
                                       force_2d=self.force_2d)
# NEW (line 585):
                force_2d=self.force_2d)

# NEW in DopplerRIO.__init__ (add parameter, default False):
                 force_2d: bool = False,
        ...
        self.force_2d = force_2d
        if force_2d:
            logging.warning("force_2d ENABLED: Vz is being clamped to zero. "
                            "This is only valid for a NADIR mount. At any tilt "
                            "between 10 and 80 deg it injects large phantom Vx. "
                            "DO NOT FLY WITH THIS ON.")

# NEW CLI flag (after line 819):
    p.add_argument('--force-2d', action='store_true',
                    help="DEBUG/NADIR-ONLY. Clamp Vz to zero in the velocity solve. "
                         "Refuses to combine with a tilt angle between 10 and 80 deg.")

# NEW guard in run_udp_loop (after line 630):
    if getattr(args, 'force_2d', False) and 10.0 < args.theta_tilt_deg < 80.0:
        raise SystemExit(
            f"REFUSING: --force-2d with --theta-tilt-deg {args.theta_tilt_deg} is "
            f"unsafe. At this tilt the Vx/Vz correlation exceeds 0.9 and clamping "
            f"Vz injects phantom forward velocity. See implementation_plan.md Sec 2.8.")
```

Also decouple `cov[2,2]`:

```python
# CURRENT (weighted_refit line 360, irls_refit line 463):
            cov[2, 2] = 1.0
# NEW:
            # 1e6, not 1.0: in 2D mode Vz is NOT measured. Publishing 1.0
            # tells the EKF "vz = 0 +/- 1 m/s", which is a confident,
            # wrong measurement. 1e6 means "no information", which is true.
            cov[2, 2] = 1.0e6
```

---

#### **D-02 — line 567–572 — 🔴 the static hypothesis is over-confident (defect B4)**

```python
# CURRENT (lines 567-572):
        if is_static:
            v_body = np.zeros(3)
            cov = np.eye(3) * 1e-4
```

`σ = 0.01 m/s` on all three axes. `mavlink_bridge.py` puts that straight into
`VISION_SPEED_ESTIMATE`, so EKF3 receives **"the vehicle's velocity is exactly zero, and I am
certain to within 1 cm/s."**

On the bench this is correct and desirable. **In flight it is a flyaway mechanism.**
`doppler_ransac` falls back to the static hypothesis whenever no moving hypothesis clears the
margin (line 319–324) — which is exactly what happens in the degenerate-geometry regime that
Table D shows you enter at altitude. So the failure mode is:

> The aircraft climbs → the beam clips → the surviving LOS set becomes near-parallel →
> no moving hypothesis clears the margin → RIO publishes **v = 0 ± 0.01 m/s** →
> EKF3 believes it with high weight → the controller sees zero velocity and commands
> more throttle/pitch to reach the waypoint → the aircraft accelerates away while
> the estimator insists it is stationary.

**Fix: gate the static hypothesis on an armed/airborne signal, and inflate its covariance.**

```python
# NEW (lines 567-572):
        if is_static:
            v_body = np.zeros(3)
            if self.airborne:
                # In flight, "static" is almost never physically true -- it is
                # far more often the signature of degenerate geometry (see
                # implementation_plan.md Sec 2.6 / defect B4). Publish it, but
                # with a covariance that says "I am guessing", so the EKF
                # coasts on the IMU rather than latching onto a false zero.
                cov = np.eye(3) * (self.static_airborne_sigma ** 2)   # default 1.0 m/s
                self.static_airborne_count += 1
                if self.static_airborne_count % 20 == 0:
                    logging.warning(
                        f"RIO has fallen back to the STATIC hypothesis "
                        f"{self.static_airborne_count} times while airborne. This "
                        f"usually means the ground has left the beam -- DESCEND.")
            else:
                cov = np.eye(3) * 1e-4      # on the ground: a real ZUPT, trust it
```

Wire `self.airborne` from the FC. The cleanest route: extend `IMU_PKT` in `imu_bridge.py` to
carry an armed/landed flag, and have `IMUListener` expose it.

```python
# imu_bridge.py -- extend the packet (breaking change, update BOTH listeners):
IMU_PKT = struct.Struct('<dffffffB')   # ... + uint8 airborne flag (33 bytes)

# in the main loop, track HEARTBEAT and EXTENDED_SYS_STATE:
    msg = master.recv_match(type=['ATTITUDE', 'HEARTBEAT', 'EXTENDED_SYS_STATE'],
                            blocking=True, timeout=1.0)
    ...
    # EXTENDED_SYS_STATE.landed_state: 1 = ON_GROUND, 2 = IN_AIR
    airborne = 1 if (armed and landed_state == 2) else 0
```

Request the stream once after heartbeat:
```python
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 500000, 0,0,0,0,0)  # 2 Hz
```

> **If you do not want the packet-format change yet**, the minimum acceptable interim fix is to
> unconditionally raise the static covariance to `np.eye(3) * 0.25` (σ = 0.5 m/s). That still
> lets the bench ZUPT work while removing the flyaway mechanism. Do **not** ship `1e-4`.

---

#### **D-03 — lines 539–553 — 🟠 IMU rotation compensation is identically zero (defect B7)**

```python
# CURRENT (lines 549-551):
            omega_cross_p = np.cross(omega_used, xyz_b)          # (N, 3)
            v_rot_comp = np.sum(u_body * omega_cross_p, axis=1)  # (N,)
            v_adjusted = v_meas + v_rot_comp
```

`u_body = xyz_b / |xyz_b|`, so `u_body ∥ xyz_b`. And `ω × p ⊥ p` by definition of the cross
product. Therefore `u·(ω×p) ≡ 0` for every point, every frame, always. Verified numerically:

```
>>> np.sum(u * np.cross(omega, p), axis=1)
array([0.0, 4.4e-16, 0.0, -1.2e-15, 0.0, -8.9e-16])
```

"Stage 3 IMU rotation compensation" has never applied any compensation. `filters.py:112-114`
contains the identical no-op in `gate_doppler_consistency`.

**Why the physics says it must be zero, and what the real correction is.**
Doppler measures range rate. For a static target, `ṙ = u·(v_target − v_sensor) = −u·v_sensor`.
The rotational term enters **only through the lever arm**, because that is what moves the
*sensor*:

```
v_sensor = v_body + ω × r_lever
  ⇒  V_i = −uᵢ·(v_body + ω × r_lever)
  ⇒  V_corrected,i = V_measured,i + uᵢ·(ω × r_lever)      ← r_lever, NOT p_i
```

The magnitude is small but not negligible: with `r_lever = 0.3 m` and a 60 °/s yaw
(`ω = 1.05 rad/s`), the correction is up to **0.31 m/s** — larger than your
`--eps 0.20` RANSAC tolerance. During aggressive turns this is what is throwing away frames.

```python
# NEW (replacing lines 542-553):
        if omega is not None:
            # Correct for the sensor's own velocity due to body rotation about
            # the lever arm:  v_sensor = v_body + omega x r_lever.
            #
            # NOTE: the previous implementation used (omega x p_i) where p_i is
            # the POINT position. That term is identically zero, because
            # (omega x p) is perpendicular to p and u = p/|p|. It compensated
            # nothing. The rotational term enters ONLY through the lever arm,
            # because the lever arm is what actually moves the sensor.
            # See implementation_plan.md defect B7.
            if attitude is not None and self.imu_level_points:
                omega_used = self.mount.get_R(attitude) @ omega
                lever_used = self.mount.get_R(attitude) @ self.mount.lever_arm
            else:
                omega_used = omega
                lever_used = self.mount.lever_arm
            v_lever = np.cross(omega_used, lever_used)        # (3,) m/s
            v_adjusted = v_meas + (u_body @ v_lever)          # (N,)
        else:
            v_adjusted = v_meas
```

Apply the identical correction in `filters.py` `gate_doppler_consistency` (lines 110–116),
adding a `lever_arm` argument threaded through `preprocess_frame` from `FilterConfig`.

> **This makes the lever arm load-bearing.** Until this change, `--lever-x/y/z` only shifted
> point *positions* (a sub-metre geometric offset, negligible against 50 m ranges) and had
> essentially no effect on the velocity solve. After this change it directly scales a
> velocity correction. **Measure it properly** (§14 Q2).

---

#### **D-04 — line 781 / `slam_node.py:754` / `supervisor.py` — 🟡 tilt default of 90° (defect B11)**

```python
# CURRENT (doppler_rio.py:781, slam_node.py:754, supervisor.py):
    p.add_argument('--theta-tilt-deg', type=float, default=90.0, ...)
```

A default of 90° means "forget the flag and the system silently models your 40° nose radar as a
nadir radar." The rotation error is 50°, which rotates the entire velocity vector — a 10 m/s
forward flight is reported as 6.4 m/s forward and 7.7 m/s descent.

```python
# NEW -- make it mandatory, everywhere:
    p.add_argument('--theta-tilt-deg', type=float, required=True,
                    help="Physical mount pitch-down angle in degrees, MEASURED with a digital "
                         "inclinometer on the levelled airframe (implementation_plan.md Sec 3.1). "
                         "No default: a wrong tilt rotates the entire velocity vector.")
```

Do the same for `--lever-x/-y/-z` in both `doppler_rio.py` (lines 787-789) and `slam_node.py`
(lines 765-767): remove the `0.12 / 0.0 / 0.05` defaults and make them `required=True`.

---

#### **D-05 — line 790 — 🟡 `--max-range 350` admits ghosts**

```python
# CURRENT (line 790):
    p.add_argument('--max-range', type=float, default=350.0)
```

350 m is a point-target figure (§1.3). At a 40° tilt and 50 m AGL the furthest real ground
return is ~107 m (near edge at 52°: 63 m; far edge at 28°: 107 m). Anything read out beyond
~150 m is a multipath ghost or a range alias, and it enters the LOS set with a nearly
horizontal `u` — precisely the direction that most corrupts the Vx/Vz split.

```python
# NEW (line 790):
    p.add_argument('--max-range', type=float, default=150.0,
                    help="Reject returns beyond this range. 350 m is the datasheet's "
                         "HIGH-RCS POINT-TARGET figure, not a diffuse-ground figure. "
                         "Beyond ~1.5x the far-beam-edge ground range, returns are "
                         "multipath ghosts with near-horizontal LOS vectors that "
                         "corrupt the Vx/Vz split. See implementation_plan.md Sec 1.3.")
```

Set the same on `slam_node.py:781` and `filters.py` `FilterConfig.max_range_m`.

> **Better: make it altitude-adaptive.** `max_range = 2.0 * agl / sin(tilt - el_half)`.
> At 40° tilt and 10 m AGL that is 21 m; at 50 m it is 107 m. Gate on the belly altimeter.
> Implement after first flight — a fixed 150 m is safe to start with.

---

#### **D-06 — line 673 — 🟡 wall-clock timestamps**

```python
# CURRENT (line 673):
        t_frame = time.time()
```

`radar_fanout.py` stamps frames with `time.monotonic()`; `slam_node.py` uses
`time.monotonic()`; `nav_node.py` measures staleness with `time.monotonic()` but computes
RIO `dt` from this wall-clock value. An NTP step (very likely on a Jetson that just acquired
network) makes `dt` jump, and `PoseTracker.on_rio_velocity` line 250 silently drops the sample
(`0 < dt < 0.5`) or, worse, integrates a huge `dt`.

```python
# NEW (line 673):
        t_frame = time.monotonic()   # monotonic everywhere; NTP steps must not
                                     # perturb dt in the dead-reckoning integrator
```

Audit every `time.time()` in `src/` and replace with `time.monotonic()` except where a
wall-clock is genuinely wanted (log filenames).

---

#### **D-07 — line 224 — 🟡 `max_speed_mps` vs Doppler ambiguity**

`doppler_ransac(..., max_speed_mps: float = 25.0)` is hardcoded and not exposed. The U300's
unambiguous velocity is **±45 m/s**, so 25 m/s is safe — but it is worth surfacing, and worth
adding a hard flight-speed cap derived from it:

```python
# NEW: expose as a CLI flag and clamp NavConfig.max_speed_mps
    p.add_argument('--max-speed-mps', type=float, default=25.0,
                    help="Reject any velocity hypothesis above this. Must be well below "
                         "the U300's +/-45 m/s unambiguous Doppler limit -- beyond it, "
                         "velocity aliases and RIO reports a confident wrong answer.")
```

> **The real operational limit is lower.** Doppler aliasing is only one bound. The others:
> at 20 Hz, 15 m/s is 0.75 m of travel per frame, which is fine; but the SLAM keyframe window
> (`--window-s 0.3-0.5`) accumulates 4.5–7.5 m of motion, smearing the cloud unless deskew is
> exact. **Cap `nav_node --max-speed` at 5 m/s for the first flights, 8 m/s thereafter.**

---

### 5.3 `src/slam_node.py`

#### **S-01 — lines 679–682 — 🔴 hardcoded `imu_level_points=True` (defect B8)**

```python
# CURRENT (lines 679-682):
                # SLAM always needs leveled points for _gravity_correct() to work.
                # This is independent of the RIO solver's imu_level_points setting.
                xyz_body = mount.to_body(pts_radar[:, :3], latest_attitude,
                                         imu_level_points=True)
```

Three separate problems:

1. `args.imu_level_points` is parsed (line 759), passed to `RadarSLAM.__init__` (line 600),
   and stored as `self.imu_level_points` (line 254) — **and then never read anywhere**. It is a
   dead field. The supervisor's `--imu-level-points` / `--no-imu-level-points` flag does nothing
   for SLAM.
2. RIO and SLAM therefore operate in **different frames by default** (supervisor default is
   `--no-imu-level-points`, so RIO is body-FRD, SLAM is gravity-levelled). Then
   `slam_node.py:474` does `t_shift = self.T_world[:3,:3] @ (self.last_v_body * dt)` —
   rotating a **body-frame** RIO velocity by a **levelled-world** rotation. During any
   non-trivial pitch/roll the prediction is wrong by `|v|·sin(tilt)`.
3. `to_body(..., imu_level_points=True)` with `latest_attitude = None` silently falls through
   to the static path (line 199). So the behaviour flips based on whether IMU packets happen to
   be arriving — an invisible mode switch.

**On a drone with a properly trimmed Cube Orange AHRS, levelling is correct and desirable.
The fix is to make it explicit, consistent, and mandatory — not hardcoded.**

```python
# NEW (lines 679-682):
                # Levelling is REQUIRED for flight: _gravity_correct() assumes
                # the incoming cloud is already gravity-aligned, and RIO must
                # operate in the SAME frame or the constant-velocity prediction
                # at line 474 mixes frames. Driven by the supervisor flag, and
                # fatal if attitude is missing -- silently falling back to the
                # unlevelled path is how the two nodes end up disagreeing.
                if slam.imu_level_points and latest_attitude is None:
                    logging.error("imu_level_points requested but no IMU attitude "
                                   "available -- dropping frame rather than silently "
                                   "processing it unlevelled.")
                    continue
                xyz_body = mount.to_body(pts_radar[:, :3], latest_attitude,
                                         imu_level_points=slam.imu_level_points)
```

**And enforce the pairing at the supervisor level** — add to `supervisor.py` `main()`:

```python
    if not args.imu_port:
        raise SystemExit("REFUSING: flight configuration requires --imu-port. RIO and SLAM "
                          "must share a levelled frame, which requires FC attitude.")
    # For flight, levelling is ON for both nodes or OFF for both. Never mixed.
    args.imu_level_points = True
```

---

#### **S-02 — lines 279–303 — 🟠 `_gravity_correct()` behaviour with a flat IMU (defect B8)**

You asked specifically what this does when the IMU is actually flat. Walking it through:

```python
def _gravity_correct(self, T):
    R_gicp  = T[:3, :3]
    yaw_gicp = atan2(R_gicp[1,0], R_gicp[0,0])          # ZYX yaw extraction
    if self.trust_imu_yaw and self.last_attitude is not None:
        yaw_gicp = self.last_attitude[2]                # magnetometer yaw
    R_corrected = Rz(yaw_gicp)                           # roll = pitch = 0, exactly
    T_out = T.copy(); T_out[:3,:3] = R_corrected
    return T_out                                         # translation UNTOUCHED
```

**With a flat IMU and levelled input, this is close to a no-op on rotation** — `R_gicp` is
already near-`Rz(ψ)`, so the clamp removes only the small residual. That is the intended and
correct behaviour, and it does suppress the roll/pitch numerical creep that caused the
191 m Z drift.

**But four things are wrong with it for flight:**

1. **It does not touch translation.** The comment claims it prevents "catastrophic Z-drift",
   but `T_out[2,3]` is passed through unchanged. The rotation clamp only stops *rotational*
   error feeding *into* translation on the next iteration; it does not bound `Z` itself.
   **`Z` is still unbounded, and the altimeter is the only thing that can bound it.**
2. **The `atan2(R[1,0], R[0,0])` yaw extraction is not gimbal-safe.** With a levelled cloud
   pitch is near zero so it is fine in practice — but add an assertion.
3. **When `trust_imu_yaw` is False** (the supervisor default), yaw comes purely from GICP,
   which over featureless terrain has no yaw observability at all — this is the "frozen yaw"
   problem that the gyro integration at lines 477-487 patches. For flight you want
   **`--trust-imu-yaw` ON** with a properly calibrated compass, because Table H shows yaw is
   your dominant error term.
4. **It is applied twice per keyframe** (line 492 on `T_pred`, line 534 on `T_world`) but only
   `if self.last_attitude is not None`. Another invisible mode switch on IMU availability.

```python
# NEW -- add a hard Z constraint from the altimeter, and assert the levelling assumption:
    def _gravity_correct(self, T: np.ndarray, agl_m: float | None = None) -> np.ndarray:
        R_gicp = T[:3, :3]
        # Levelling assumption check: if GICP has drifted more than ~10 deg off
        # level, the yaw extraction below is no longer valid and something
        # upstream (levelling, deskew, correspondence) has failed.
        if abs(R_gicp[2, 0]) > 0.17:            # |sin(pitch)| > 10 deg
            logging.warning(f"[slam] GICP pose is {math.degrees(math.asin(abs(R_gicp[2,0]))):.0f} "
                             f"deg off level -- levelling or deskew is broken.")
        yaw = math.atan2(R_gicp[1, 0], R_gicp[0, 0])
        if self.trust_imu_yaw and self.last_attitude is not None:
            yaw = self.last_attitude[2]
        cy, sy = math.cos(yaw), math.sin(yaw)
        T_out = T.copy()
        T_out[:3, :3] = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        # HARD Z CONSTRAINT. The rotation clamp alone does not bound Z (the
        # translation column was previously passed through untouched, despite
        # the docstring's claim). The belly altimeter is the only absolute
        # height reference in the system; without it Z is a free-running
        # integrator. Map frame is Z-DOWN, so world Z = -(AGL) + takeoff offset.
        if agl_m is not None and math.isfinite(agl_m):
            z_meas = -(agl_m - self.agl_at_bootstrap)
            # Complementary blend, not a hard snap -- a hard snap injects a step
            # into the pose chain that GICP then tries to undo on the next frame.
            alpha = self.alt_blend_alpha            # 0.15 is a good starting point
            T_out[2, 3] = (1.0 - alpha) * T_out[2, 3] + alpha * z_meas
        return T_out
```

Wire `agl_m` from a new altimeter UDP listener in `slam_node.run()` (same pattern as the IMU
listener), and latch `self.agl_at_bootstrap` on the first keyframe.

---

#### **S-03 — line 466 & 474 — 🟡 `dt` and the constant-velocity prediction**

```python
# CURRENT (line 466):
        dt = (t - self.last_kf_t) if self.last_kf_t is not None else (1.0 / 5.0)
```

No clamp. If a GICP call blocks for 400 ms (line 503 already warns above 150 ms) or a keyframe
is dropped, `dt` can be large and `T_pred` is extrapolated metres away from truth, putting GICP
outside its `max_corr_dist` basin. Add:

```python
# NEW (line 466):
        dt = (t - self.last_kf_t) if self.last_kf_t is not None else (1.0 / 5.0)
        if not (0.0 < dt < 0.6):
            logging.warning(f"[slam] keyframe dt = {dt:.3f}s out of range; clamping. "
                             f"A large dt extrapolates T_pred outside GICP's "
                             f"correspondence basin.")
            dt = min(max(dt, 0.0), 0.6)
```

---

#### **S-04 — line 789–791 — 🟡 `--plane-threshold` must scale with altitude**

```python
    p.add_argument('--plane-threshold', type=float, default=0.6, ...)
```

The help text already notes "increase for flight altitude (e.g. 1.0 at 50 m AGL)". At 50 m
with ±0.23 m range accuracy projected through a 40° incidence, the ground plane thickness is
~0.6–0.9 m from noise alone, plus real terrain roughness. A 0.6 m threshold at altitude
splits the real ground into "ground" and "off-plane", inflating the off-plane set with
noise and degrading GICP.

**Set `--plane-threshold 1.2` for flight**, and consider driving it from AGL once the
altimeter feed exists.

---

### 5.4 `src/mavlink_bridge.py`

#### **M-01 — lines 114, 120 — 🟠 arbitrary timestamp epoch (defect B10)**

```python
# CURRENT (line 114):
    t0_wall = time.time()
# CURRENT (line 120):
            t_usec = int((time.time() - t0_wall) * 1e6)
```

`t_usec` counts from "whenever this process started". ArduPilot uses the vision timestamp
together with `EK3_VIS_DELAY` to place the measurement correctly in its fusion buffer. With an
arbitrary epoch you cannot calibrate the delay, and ArduPilot may treat the measurement as
wildly stale or from the future.

```python
# NEW:
    # Sync to the autopilot's boot clock so EK3_VIS_DELAY_MS is meaningful.
    # Ask for SYSTEM_TIME, then hold the (fc_boot_ms - local_mono) offset.
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME, 1000000, 0, 0, 0, 0, 0)
    t_offset_us = None
    ...
    # in the loop, opportunistically refresh:
    st = conn.recv_match(type='SYSTEM_TIME', blocking=False)
    if st is not None:
        t_offset_us = st.time_boot_ms * 1000 - int(time.monotonic() * 1e6)
    ...
    # and when sending, use the RADAR FRAME time, not "now" -- the pipeline
    # latency (UART + parse + RANSAC + IRLS + UDP) is what EK3_VIS_DELAY
    # must compensate, and it is measured from t_frame.
    t_usec = int(t_frame * 1e6) + (t_offset_us or 0)
```

> 📌 **TEST-02 — measure the pipeline latency.** Log `time.monotonic() - t_frame` at the
> moment `mavlink_bridge` sends. Take the median over 60 s. That value, in milliseconds,
> is your `EK3_VIS_DELAY_MS`. Expect 50–120 ms. Guessing this wrong causes the classic
> "EKF oscillates in position hold" symptom.

#### **M-02 — line 90 — 🟡 `DISTANCE_SENSOR` range limits are wrong for the U200A**

```python
# CURRENT (line 90):
def send_distance_sensor(conn, t_boot_ms, range_m, min_range_m=0.1, max_range_m=30.0):
```

The U200A is **0.2–200 m**. Reporting `max_range = 30 m` makes ArduPilot discard every reading
above 30 m and, worse, treat 30 m as "out of range" — exactly when you need it at 40–50 m.

```python
# NEW:
def send_distance_sensor(conn, t_boot_ms, range_m,
                         min_range_m=0.2, max_range_m=200.0):
    """Linpowave U200A: 0.2-200 m, +/-0.2 m, 20 Hz. These limits MUST match
    RNGFND1_MIN_CM / RNGFND1_MAX_CM on the autopilot or ArduPilot will
    silently discard in-range readings."""
```

And add a signal-quality field so ArduPilot can reject bad returns:

```python
        conn.mav.distance_sensor_send(
            t_boot_ms,
            int(min_range_m * 100), int(max_range_m * 100),
            int(max(range_m, 0.0) * 100),
            mavutil.mavlink.MAV_DISTANCE_SENSOR_RADAR,
            id=1,
            orientation=mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,
            covariance=int(20),          # 0.2 m -> 20 cm, per the message spec
        )
```

#### **M-03 — line 117–127 — 🟠 the RIO socket timeout starves the altimeter**

```python
# CURRENT:
    rio_sock.settimeout(0.02)
    ...
    alt_sock.settimeout(0.001)
```

The loop blocks up to 20 ms on RIO, then polls the altimeter for 1 ms. At 20 Hz both streams
arrive every 50 ms, so this mostly works — but under load the altimeter is the one that gets
dropped, and the altimeter is now a flight-critical Z source. Use `select()` on both:

```python
    import select
    socks = [rio_sock] + ([alt_sock] if alt_sock else [])
    for s in socks:
        s.setblocking(False)
    while True:
        ready, _, _ = select.select(socks, [], [], 0.05)
        for s in ready:
            ...
```

---

### 5.5 NEW FILE — `src/altimeter_bridge.py` (defect B6)

**Nothing currently reads the U200A.** This is the missing subsystem. The file must:

1. own the U200A serial port exclusively (same pattern as `radar_fanout.py`);
2. parse its frame format (**you must send me the U200A protocol PDF — see §14 Q7**);
3. publish `struct '<f'` range packets to a fan-out list of UDP ports:
   - `5030` → `mavlink_bridge.py` (→ `DISTANCE_SENSOR` → EKF3 `POSZ`)
   - `5031` → `nav_node.py` (→ absolute AGL for the geofence and failsafe descent)
   - `5032` → `slam_node.py` (→ Z constraint in `_gravity_correct`, change S-02)
   - `5033` → `gps_logger.py` (→ ground-truth comparison against GPS altitude)
4. apply the **lever-arm Z offset** (`agl_fc = agl_measured + lever_z_alt`) so every consumer
   receives AGL referenced to the **FC**, not to the radar;
5. reject out-of-envelope readings (`< 0.2 m` or `> 200 m`) and emit nothing rather than a
   clamped value — a clamped 200 m reading is indistinguishable from a real one;
6. implement a **median-of-5 + rate limiter**: a single spurious return from a bird or a
   passing vehicle must not step the EKF height. Reject any sample more than
   `max_climb_mps × dt × 3` from the previous accepted sample;
7. auto-reconnect on USB dropout, exactly like `SerialReaderThread._reconnect()`.

Skeleton:

```python
#!/usr/bin/env python3
"""altimeter_bridge.py -- Linpowave U200A belly radar -> UDP fan-out.

The U200A is the ONLY absolute height reference in this GPS-denied stack.
SLAM Z is a free-running integrator (documented: 191 m drift in 96 s) and the
barometer drifts with weather and prop wash. Everything vertical -- the EKF
height state, the nav geofence, the failsafe descent, terrain following --
depends on this process staying alive.
"""
ALT_PKT = struct.Struct('<f')       # matches mavlink_bridge.py

class AltimeterReader(threading.Thread):
    def __init__(self, port, baud, lever_z_m, dests):
        ...
        self.window = collections.deque(maxlen=5)
        self.last_accepted = None
        self.last_t = None

    def _accept(self, r_raw, t):
        if not (0.2 <= r_raw <= 200.0):
            self.n_out_of_envelope += 1
            return None
        self.window.append(r_raw)
        r = float(np.median(self.window))         # spike rejection
        if self.last_accepted is not None:
            dt = max(t - self.last_t, 1e-3)
            if abs(r - self.last_accepted) > 3.0 * MAX_CLIMB_MPS * dt:
                self.n_rate_rejected += 1
                return None                        # bird / vehicle / roof edge
        self.last_accepted, self.last_t = r, t
        return r + self.lever_z_m                  # referenced to the FC
```

Register it in `supervisor.py` `build_children()` as **critical=True**:

```python
    if args.alt_port:
        children.insert(1, Child(
            "alt", [py, _p("altimeter_bridge.py"),
                    "--port", args.alt_port,
                    "--baud", str(args.alt_baud),
                    "--lever-z", str(args.alt_lever_z),
                    "--dest-ports", "5030,5031,5032,5033"],
            critical=True, start_delay_s=0.5))
    elif args.enable_nav:
        raise SystemExit("REFUSING: --enable-nav without --alt-port. The belly "
                          "altimeter is the only absolute height reference in "
                          "this stack. See implementation_plan.md Sec 2.4.")
```

and pass `--alt-port 5030` to `mavlink_bridge.py` (currently never set — line 183 default is 0).

---

## 6. Integration architecture

### 6.1 Process and port map (post-fix)

```
   ┌──────────────┐  UART 921600            ┌──────────────┐  UART
   │ U300  NOSE   │─────────────┐           │ U200A  BELLY │──────┐
   │ 40° down     │             │           │ nadir        │      │
   └──────────────┘             ▼           └──────────────┘      ▼
                        ┌───────────────┐                 ┌──────────────────┐
                        │ radar_fanout  │                 │ altimeter_bridge │  ★NEW
                        │  (UART owner) │                 │  (UART owner)    │
                        └───┬───────┬───┘                 └──┬───┬───┬───┬───┘
                     20 Hz  │       │  5 Hz            5030  │   │   │   │ 5033
                      5005  │       │  5010,5012            ▼   │5031  │
                            ▼       ▼                  mavlink  │   │   ▼
                    ┌────────────┐ ┌────────────┐      _bridge  │   │  gps
                    │ doppler_rio│ │ slam_node  │◄─5032─────────┘   │  _logger
                    │  20 Hz     │ │  ~5 Hz     │                   │
                    └──────┬─────┘ └──────┬─────┘               nav_node
        5006 (→slam) 5007 (→mav) 5008 (→nav) 5009 (→vis) 5013 (→gps)
                           │              │ pose: 5011 (→nav) 5014 (→gps)
                           ▼              ▼
              ┌─────────────────────┐  ┌─────────────────────┐
              │  mavlink_bridge     │  │     nav_node        │
              │  VISION_SPEED_EST   │  │  SET_POSITION_      │
              │  DISTANCE_SENSOR    │  │  TARGET_LOCAL_NED   │
              │  (AIDING ONLY)      │  │  (COMMANDS!)        │
              └──────────┬──────────┘  └──────────┬──────────┘
                         │      MAVLink TELEM2    │
                         └───────────┬────────────┘
                                     ▼
                         ┌───────────────────────┐
                         │   CUBE ORANGE / EKF3  │
                         │  AHRS_EKF_TYPE=3      │
                         │  EK3_SRC1_VELXY = 6   │  ← RIO velocity
                         │  EK3_SRC1_POSZ  = 2   │  ← U200A rangefinder
                         │  EK3_SRC1_VELZ  = 6   │
                         │  EK3_SRC1_YAW   = 1   │  ← compass
                         │  GPS: LOGGING ONLY    │
                         └───────────┬───────────┘
                                     │ ATTITUDE 50 Hz, EKF_STATUS, GPS_RAW_INT
                                     ▼
                              ┌─────────────┐
                              │ imu_bridge  │→ 5020 rio, 5021 slam, 5022 gps
                              └─────────────┘
```

### 6.2 Complete port allocation

| Port | Producer | Consumer | Payload |
|---|---|---|---|
| 5005 | `radar_fanout` | `doppler_rio` | raw Points_float, 20 Hz |
| 5006 | `doppler_rio` | `slam_node` | RIO velocity (deskew/prior) |
| 5007 | `doppler_rio` | `mavlink_bridge` | RIO velocity → EKF3 |
| 5008 | `doppler_rio` | `nav_node` | RIO velocity → dead reckoning |
| 5009 | `doppler_rio` | `visualizer_3d` | RIO velocity |
| 5010 | `radar_fanout` | `slam_node` | decimated points, ~5–10 Hz |
| 5011 | `slam_node` | `nav_node` | pose `T_world` + `fwd_range` |
| 5012 | `radar_fanout` | `visualizer_3d` | decimated points |
| 5013 | `doppler_rio` | `gps_logger` | RIO velocity |
| 5014 | `slam_node` | `gps_logger` | pose |
| 5020 | `imu_bridge` | `doppler_rio` | attitude + ω + **airborne flag** |
| 5021 | `imu_bridge` | `slam_node` | attitude + ω |
| 5022 | `imu_bridge` | `gps_logger` | attitude + ω |
| **5030** ★ | `altimeter_bridge` | `mavlink_bridge` | AGL float32 → `DISTANCE_SENSOR` |
| **5031** ★ | `altimeter_bridge` | `nav_node` | AGL → geofence / failsafe descent |
| **5032** ★ | `altimeter_bridge` | `slam_node` | AGL → Z constraint |
| **5033** ★ | `altimeter_bridge` | `gps_logger` | AGL → ground truth compare |

### 6.3 The two MAVLink connections — keep them separate

`mavlink_bridge.py` and `nav_node.py` have fundamentally different trust levels:

| | `mavlink_bridge.py` | `nav_node.py` |
|---|---|---|
| Sends | `VISION_SPEED_ESTIMATE`, `DISTANCE_SENSOR` | `SET_POSITION_TARGET_LOCAL_NED`, mode changes |
| Worst case if it misbehaves | EKF rejects bad measurements; vehicle degrades gracefully | **vehicle flies into something** |
| Can be tested with props off | Yes | No |
| Should arm the vehicle | Never | **Never** (change N-08) |

They must use **different `source_component` IDs** (already: 197 vs 191 — good) and, on
hardware, ideally different physical links (`TELEM1` for aiding, `TELEM2` for control) so a
UART stall in one does not block the other. Set `SERIAL1_PROTOCOL = 2`, `SERIAL2_PROTOCOL = 2`,
`SERIAL1_BAUD = 921`, `SERIAL2_BAUD = 921`.

### 6.4 GPS is logging-only — make it structurally true

"GPS only for cross-verification" must be enforced by the **autopilot parameters**, not by
convention:

```
GPS_TYPE          1      # GPS enabled and LOGGING
EK3_SRC1_POSXY    0      # <- GPS is NOT a position source
EK3_SRC1_VELXY    6      # <- ExternalNav (your RIO)
EK3_SRC1_POSZ     2      # <- RangeFinder (your U200A)
EK3_SRC1_VELZ     6
EK3_SRC1_YAW      1      # <- Compass
```

With `EK3_SRC1_POSXY = 0` and `VELXY = 6`, EKF3 holds horizontal position by integrating your
radar velocity. GPS still appears in the dataflash log (`GPS` messages) and in `gps_logger.py`'s
JSONL, so post-flight comparison works — but **the estimator never sees it.** That is the only
honest way to prove the radar solution works.

> **Keep GPS as an armed safety net.** Configure `EK3_SRC2_*` as a full GPS source set
> (`POSXY=3, VELXY=3, POSZ=1, VELZ=3, YAW=1`) and bind an RC switch to
> `RCx_OPTION = 90` (EKF Source Set). The safety pilot can then fall back to GPS navigation
> **with one switch flip** if the radar solution diverges. This costs nothing and does not
> contaminate the experiment, because the switch position is recorded in the log.

---

## 7. ArduPilot parameter set (Cube Orange)

Load with `param load rio_gpsdenied.parm` in MAVProxy, or paste into Mission Planner's
Full Parameter Tree. **Reboot after loading.**

```ini
# ── EKF3 core ──────────────────────────────────────────────────────────────
AHRS_EKF_TYPE        3
EK3_ENABLE           1
EK2_ENABLE           0

# ── Source set 1: RADAR (the flight configuration) ─────────────────────────
EK3_SRC1_POSXY       0     # no absolute horizontal position (velocity-only DR)
EK3_SRC1_VELXY       6     # ExternalNav  <- doppler_rio via VISION_SPEED_ESTIMATE
EK3_SRC1_POSZ        2     # RangeFinder  <- U200A via DISTANCE_SENSOR
EK3_SRC1_VELZ        6     # ExternalNav
EK3_SRC1_YAW         1     # Compass

# ── Source set 2: GPS (safety fallback on an RC switch) ────────────────────
EK3_SRC2_POSXY       3
EK3_SRC2_VELXY       3
EK3_SRC2_POSZ        1
EK3_SRC2_VELZ        3
EK3_SRC2_YAW         1
RC8_OPTION           90    # EKF Source Set: low=SRC1(radar) high=SRC2(GPS)

# ── Visual odometry input ──────────────────────────────────────────────────
VISO_TYPE            1     # MAVLink vision messages
VISO_POS_X           <Lx>  # ★ nose radar lever arm, metres, FRD from the FC
VISO_POS_Y           <Ly>  # ★  (§14 Q2 -- I need these numbers)
VISO_POS_Z           <Lz>  # ★
VISO_DELAY_MS        <T>   # ★ from TEST-02. Expect 50-120.
VISO_VEL_M_NSE       0.15  # start conservative; tighten after TEST-04
EK3_VIS_VERR_MIN     0.10
EK3_VIS_VERR_MAX     1.50

# ── Rangefinder (U200A) ────────────────────────────────────────────────────
RNGFND1_TYPE         10    # MAVLink DISTANCE_SENSOR
RNGFND1_ORIENT       25    # down
RNGFND1_MIN_CM       20    # 0.2 m  -- U200A spec
RNGFND1_MAX_CM       20000 # 200 m  -- U200A spec
RNGFND1_POS_X        <Ax>  # ★ belly altimeter lever arm (§14 Q3)
RNGFND1_POS_Y        <Ay>  # ★
RNGFND1_POS_Z        <Az>  # ★ (positive DOWN from the FC)
RNGFND_LANDING       1
EK3_RNG_USE_HGT      70    # use rangefinder for height below 70% of RNGFND_MAX
EK3_RNG_USE_SPD      8.0   # ...and below 8 m/s ground speed
EK3_RNG_M_NSE        0.30  # U200A is +/-0.2 m; 0.3 gives headroom for terrain

# ── GPS: logging only ──────────────────────────────────────────────────────
GPS_TYPE             1
# (deliberately NOT referenced by EK3_SRC1_*)

# ── Compass: yaw is your dominant error term (Table H) ─────────────────────
COMPASS_USE          1
COMPASS_LEARN        0     # OFF in flight -- learning corrupts yaw mid-mission
COMPASS_AUTODEC      1

# ── GUIDED / OFFBOARD behaviour ────────────────────────────────────────────
GUID_OPTIONS         0
WPNAV_SPEED          500   # 5 m/s -- cap for first flights (Sec 5.2 D-07)
WPNAV_SPEED_UP       100   # 1 m/s climb (Sec 5.1 N-06)
WPNAV_SPEED_DN       70    # 0.7 m/s descent
PILOT_SPEED_UP       150
ANGLE_MAX            2000  # 20 deg -- limits the pitch that swings the radar beam

# ── Failsafes (PRIMARY layer; nav_node's are secondary) ────────────────────
FS_EKF_ACTION        1     # LAND on EKF failsafe -- NOT RTL (Sec 5.1 N-07)
FS_EKF_THRESH        0.8
FENCE_ENABLE         1
FENCE_TYPE           7     # alt + circle + polygon
FENCE_ALT_MAX        50    # ★ MUST match your measured R_gnd ceiling (Sec 2.6)
FENCE_RADIUS         150
FENCE_ACTION         1     # RTL_or_LAND -- verify behaviour in SITL first
BRD_SAFETY_DEFLT     1

# ── Logging: you need these to verify anything ─────────────────────────────
LOG_BITMASK          958   # include XKF*, RFND, VISO, GPS
LOG_DISARMED         1
```

> ### ⚠️ `SET_GPS_GLOBAL_ORIGIN` — the step everyone forgets
> Without a GPS fix, EKF3 has **no origin**, and `GUIDED` mode, `LOITER`, and any local-frame
> setpoint will be refused with "Need Position Estimate". You must send **MAVLink #48
> `SET_GPS_GLOBAL_ORIGIN`** once after boot (any plausible lat/lon works — it only anchors the
> local frame), plus **#243 `SET_HOME_POSITION`**.
>
> Add this to `nav_node.py`'s `Autopilot.__init__`:
> ```python
>     def set_origin(self, lat_deg, lon_deg, alt_m):
>         """EKF3 refuses GUIDED without an origin. With GPS excluded as a
>         source (EK3_SRC1_POSXY=0) nothing else will set one."""
>         self.conn.mav.set_gps_global_origin_send(
>             self.conn.target_system,
>             int(lat_deg * 1e7), int(lon_deg * 1e7), int(alt_m * 1000))
>         self.conn.mav.set_home_position_send(
>             self.conn.target_system,
>             int(lat_deg * 1e7), int(lon_deg * 1e7), int(alt_m * 1000),
>             0, 0, 0, [1, 0, 0, 0], 0, 0, 1, 0)
> ```
> Call it with the takeoff-site coordinates (which `gps_logger.py` already has) or a constant.
> **Verify in SITL that GUIDED accepts setpoints after this call before you fly.**

---

## 8. How navigation, waypoints, and RTH actually work

### 8.1 The two estimation loops

| | **Fast loop (RIO)** | **Slow loop (SLAM)** |
|---|---|---|
| Rate | 20 Hz | 3–5 Hz |
| Produces | body-frame velocity + 3×3 covariance | absolute pose `T_world` |
| Goes to | EKF3 via `VISION_SPEED_ESTIMATE` | `nav_node` position + `_gravity_correct` |
| Error behaviour | unbiased, low noise; **position error grows linearly** | bounded while terrain is re-observed |
| Fails when | ground leaves the beam (Sec 2.6) | terrain is featureless / GICP degenerate |

RIO is the flight-critical path. SLAM is the drift-corrector. **If SLAM dies, RIO + altimeter
still gives you a controllable aircraft with a drifting position** — that is why the failsafe
ladder (N-07) degrades to "descend in place on the altimeter" and not to "RTL on a drifting
position."

### 8.2 Waypoint navigation — the actual control chain

```
 waypoint (user, fwd/right/UP metres from bootstrap)
   │  parse_waypoints(): Z negated -> map frame (fwd, right, DOWN)
   ▼
 velocity_toward(pos_map, target_map, max_speed, max_climb, max_descend)     [N-06]
   │  P-guidance, horizontal and vertical capped separately
   ▼
 apply_obstacle_scaling(v, heading, fwd_range)                               [N-02]
   │  scale the along-track component by the SLAM forward-cone range
   ▼
 VelocitySlewLimiter.step(v)                                                  accel limit
   ▼
 _send_map_velocity_as_ned(v, heading)                                       [N-12]
   │  Rz(yaw_offset) rotation  +  yaw-rate to point the nose along track
   ▼
 SET_POSITION_TARGET_LOCAL_NED  (type_mask = 1479)                           [N-01]
   ▼
 ArduPilot GUIDED  ->  position controller  ->  attitude  ->  motors
```

**Position feedback comes from `PoseTracker`, which is the weak link.** It is *not* a Kalman
filter — it snaps to the SLAM pose on each keyframe and dead-reckons with RIO in between
(lines 226–253). Two consequences:

1. **Every SLAM keyframe produces a position step.** At 5 Hz with a 10 cm correction, the
   controller sees a 0.5 m/s velocity transient. `VelocitySlewLimiter` absorbs most of it, but
   under GICP jitter you will see the aircraft "twitch" at the keyframe rate.
2. **There is no consistency check between the two sources.** If SLAM jumps 20 m (a bad GICP
   match — entirely possible over repetitive terrain like a ploughed field), `PoseTracker`
   accepts it silently and the controller commands a 20 m correction.

```python
# ADD to PoseTracker.on_slam_pose, before accepting the pose:
        if self.pose.have_pose:
            jump = float(np.linalg.norm(p_nav - self.pose.position_nav))
            dt_since = time.monotonic() - self.pose.t_local
            max_plausible = self.cfg.max_speed_mps * max(dt_since, 0.05) * 3.0 + 2.0
            if jump > max_plausible:
                # A SLAM pose that implies impossible motion is a bad GICP
                # match, not a real correction. Reject it and keep dead
                # reckoning -- a drifting estimate beats a teleporting one.
                print(f"[nav_node] REJECTED SLAM pose: {jump:.1f} m jump in "
                      f"{dt_since:.2f} s (max plausible {max_plausible:.1f} m)")
                self.n_slam_rejects += 1
                if self.n_slam_rejects > 10:
                    print("[nav_node] SLAM repeatedly implausible -- treating pose as LOST")
                    self.pose.have_pose = False
                return
            self.n_slam_rejects = 0
```

**Long term (do this after first flights):** replace `PoseTracker` with a proper 3-state
position EKF, or — simpler and better — **stop maintaining a separate position estimate in
`nav_node` entirely** and read `LOCAL_POSITION_NED` back from the autopilot. ArduPilot's EKF3
is already fusing your velocity and rangefinder; duplicating that fusion badly in Python is the
single largest remaining architectural weakness. Feed SLAM pose in as
`VISION_POSITION_ESTIMATE` (#102) with `EK3_SRC1_POSXY = 6`, and let one estimator own the
state.

### 8.3 RTH — three different things called "return home"

| Trigger | Mechanism | Trustworthiness |
|---|---|---|
| **Operator types `rth`** | breadcrumb reverse traverse (N-11), full stack alive | Best. Re-observes mapped terrain, corrects yaw. |
| **Pose stale 2–6 s** | breadcrumb reverse traverse, degraded | Acceptable. The last known position is recent. |
| **Pose lost > 6 s, or EKF unhealthy** | **vertical descent in place** (N-07) | The only safe action. Do not fly horizontally on an estimate you have declared untrustworthy. |
| ArduPilot `FS_EKF_ACTION` | `LAND` (param set §7), **not RTL** | Matches the above. |

**Why breadcrumb-reverse rather than a straight line home:**

1. The outbound path is known flyable and known clear of obstacles.
2. It re-traverses ground already in the SLAM map, so GICP finds correspondences and **corrects
   the accumulated yaw error**. A straight line goes over unmapped terrain with a heading error
   you have no way to detect (Table H: 2° yaw × 500 m = 17.5 m).
3. It naturally handles the case where the mission went around an obstacle.

**Final descent** must be on the altimeter, not on SLAM Z and not on the barometer.
`RNGFND_LANDING = 1` gives you this on the ArduPilot side; the `FAILSAFE_LAND` branch (N-07)
gives you the same thing companion-side.

### 8.4 What "pose hold" means here

Commanding zero velocity (`HOLD` state, line 513) does **not** hold position — it holds
*velocity*, and the aircraft drifts with the wind at whatever the estimator's bias is. For a
real position hold you need the position error fed back. Since EKF3 is already doing this, the
correct implementation of pose hold is:

```python
        if self.state == NavState.HOLD:
            if self.hold_target_map is None:
                self.hold_target_map = self.tracker.pose.position_nav.copy()
            v = velocity_toward(pos, self.hold_target_map, 1.0,
                                 self.cfg.max_climb_mps, self.cfg.max_descend_mps)
            self._send_map_velocity_as_ned(self.slew.step(v, time.monotonic()), None)
            return
```

(Clear `hold_target_map` on every state exit.) Alternatively — and better — send
`MAV_CMD_DO_SET_MODE` to `LOITER`/`BRAKE` and let ArduPilot's position controller do it,
which it does far better than a Python loop at 10 Hz.

---

## 9. Execution commands

### 9.1 One-time setup

```bash
cd /home/alok/radar/rio_stack
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
pip install pymavlink pynmea2 pyserial open3d numpy scipy

# Serial permissions + device pinning
sudo usermod -aG dialout $USER            # log out / back in
sudo cp docs/99-rio.rules /etc/udev/rules.d/ && sudo udevadm control --reload

# Verify the self-tests still pass AFTER you apply Section 5
python3 src/doppler_rio.py --selftest
python3 src/mavlink_bridge.py --selftest
python3 tests/sitl_test.py
```

### 9.2 Bench / SITL validation (NO PROPS, NO AIRCRAFT)

```bash
# Terminal A -- ArduPilot SITL
sim_vehicle.py -v ArduCopter -f quad --console --map \
    --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551

# Terminal B -- the stack against SITL, aiding only, no nav
python3 src/supervisor.py \
    --port /dev/radar_nose --alt-port /dev/radar_alt \
    --tilt-deg 40.0 --lateral-sign 1.0 \
    --lever-x <Lx> --lever-y <Ly> --lever-z <Lz> --alt-lever-z <Az> \
    --imu-port /dev/cube --imu-baud 115200 \
    --platform ardupilot --mavlink-dest udp:127.0.0.1:14550 \
    --imu-level-points --trust-imu-yaw \
    --max-range 150 --plane-threshold 1.2 \
    --voxel-size 1.0 --max-corr-dist 4.0 --min-correspondences 12 \
    --eps 0.20 --cond-reject-threshold 12.0 \
    --save-pcd maps/ --visualizer
```

### 9.3 Field deployment with `screen`

Put this in `scripts/fly.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
cd /home/alok/radar/rio_stack
source .venv/bin/activate

# ── EDIT THESE THREE BLOCKS BEFORE EVERY FLIGHT ────────────────────────────
TILT=40.0                    # MEASURED tilt, not nominal (Sec 3.1)
LEVER_X=0.00; LEVER_Y=0.00; LEVER_Z=0.00     # ★ nose radar, FRD from FC
ALT_LEVER_Z=0.00                             # ★ belly altimeter, +down from FC
MAX_ALT=50                   # from your measured R_gnd (Sec 2.6)
MAX_SPEED=3.0                # start slow
WAYPOINTS="0,0,15; 30,0,15; 30,30,15; 0,30,15; 0,0,15"   # fwd,right,UP metres
# ───────────────────────────────────────────────────────────────────────────

screen -wipe >/dev/null 2>&1 || true
screen -dmS rio -t stack
screen -S rio -p stack -X stuff \
"python3 src/supervisor.py \
  --port /dev/radar_nose --alt-port /dev/radar_alt \
  --tilt-deg ${TILT} --lateral-sign 1.0 \
  --lever-x ${LEVER_X} --lever-y ${LEVER_Y} --lever-z ${LEVER_Z} \
  --alt-lever-z ${ALT_LEVER_Z} \
  --imu-port /dev/cube --imu-baud 115200 \
  --gps-port /dev/cube --log-dir logs \
  --platform ardupilot --mavlink-dest /dev/cube \
  --imu-level-points --trust-imu-yaw \
  --max-range 150 --plane-threshold 1.2 \
  --save-pcd maps/ 2>&1 | tee logs/stack_\$(date +%Y%m%d_%H%M%S).log
"$'\n'

sleep 8
screen -S rio -X screen -t nav
screen -S rio -p nav -X stuff \
"python3 src/nav_node.py \
  --mavlink-dest /dev/cube --baud 921600 --platform ardupilot \
  --pose-port 5011 --rio-port 5008 --alt-port 5031 \
  --waypoints '${WAYPOINTS}' \
  --max-speed ${MAX_SPEED} --max-altitude ${MAX_ALT} --max-radius 100 \
  2>&1 | tee logs/nav_\$(date +%Y%m%d_%H%M%S).log
"$'\n'

echo "Stack running in screen session 'rio'."
echo "  screen -r rio            attach"
echo "  Ctrl-a n / Ctrl-a p      next / previous window"
echo "  Ctrl-a d                 detach (leaves everything running)"
echo "  screen -S rio -X quit    kill everything"
```

**Operating it:**

```bash
chmod +x scripts/fly.sh && ./scripts/fly.sh
screen -r rio                       # attach
# Ctrl-a 1  -> the nav window
# Wait for:  [nav_node] state INIT -> WAIT_FOR_POSE -> HOLD
#            [nav_node] yaw_offset LATCHED at +XX.X deg
# Safety pilot takes off manually in LOITER, climbs to 15 m, confirms stable hover.
# THEN, in the nav window, type:
go                                  # arms nothing; switches to GUIDED and starts
rth                                 # breadcrumb return home
land                                # hand off to ArduPilot LAND
# Ctrl-a d                          # detach, keep flying
```

**Emergency:**
```bash
screen -S rio -p nav -X stuff $'\003'     # SIGINT nav_node only -> setpoints stop
                                          # ArduPilot GUIDED times out in ~3 s and holds
screen -S rio -X quit                     # kill the entire stack
```
> The safety pilot's mode switch on the transmitter is **always** faster and more reliable
> than any of the above. Flipping to `LOITER` or `STABILIZE` preempts every GUIDED setpoint
> instantly. That is the real abort.

### 9.4 Post-flight verification

```bash
python3 tools/analyze_run.py logs/run_YYYYMMDD_HHMMSS.jsonl --save-npz results.npz
```

---

## 10. Test progression — gates, not suggestions

Each gate must pass before the next is attempted. **No gate may be skipped because the previous
one "looked fine."**

| Gate | Test | Pass criterion | What it catches |
|---|---|---|---|
| **G0** | Bench: radar on a table, `--selftest` on all modules | all self-tests green | regressions from Section 5 |
| **G1** | **Axis validation.** Corner reflector 3 m ahead on boresight; then 3 m to the RIGHT; then a clear ground patch | ahead → `body X ≈ +3`; right → `body Y ≈ +3`; ground → `body Z` positive (down) | `lateral_sign` polarity, the `P` permutation, tilt sign |
| **G2** | **Walk test.** Carry the rig 20 m forward at ~1 m/s, levelled | `v_body ≈ [+1.0, 0, 0]`, `‖v‖` error < 10%, `is_static` < 5% of frames | RIO scale, sign, and the 40° tilt value |
| **G3** | **Vertical test.** Raise/lower the rig 2 m while walking forward | Vz tracks; Vx does **not** spike | that `force_2d` is truly gone (D-01) |
| **G4** | **Rotation test.** Yaw the rig ±60°/s while stationary | `v_body` stays < 0.2 m/s | lever-arm compensation (D-03) |
| **G5** | **Altimeter bench.** Known heights 1, 3, 5, 10 m; tilt the rig ±15° | reported AGL within ±0.3 m, tilt-compensated | `altimeter_bridge`, cos-correction (N-04) |
| **G6** | **SITL, aiding only.** `--no-enable-nav`, fly SITL manually | `EKF_STATUS_REPORT` variances < 0.5; `XKF` log shows vision velocity fused | EKF params, `VISO_*`, `EK3_VIS_DELAY` |
| **G7** | **SITL, full nav.** Square mission in SITL | every waypoint reached, no phantom obstacle stop, no yaw-to-North | N-01, N-02, N-03 |
| **G8** | **TEST-01: altitude ladder, tethered or manual.** Hover 10/20/30/40/50/60/80 m, 30 s each | record `n_total`, `n_inliers`, `cond`, reject rate vs altitude | **your real `R_gnd` and hence your ceiling** |
| **G9** | **TEST-02: latency.** Log `monotonic() − t_frame` at MAVLink send | median latency → set `EK3_VIS_DELAY_MS` / `VISO_DELAY_MS` | M-01 |
| **G10** | **Manual flight, radar aiding only, GPS excluded, props on.** 5 min hover at 15 m | position drift vs GPS < 5 m over 5 min | the whole aiding chain against ground truth |
| **G11** | **Manual translation flight.** 100 m out-and-back at 3 m/s, GPS excluded | closure error < 3% of path length | scale + yaw error |
| **G12** | **First GUIDED flight.** 10 m square at 15 m AGL, 2 m/s, safety pilot on the switch, wide open field | mission completes, RTH works, no failsafe | everything |

> ### 🚨 **G10 is the hard gate.** Until you have flown a 5-minute hover with
> `EK3_SRC1_POSXY = 0` and `VELXY = 6` and compared the EKF position to logged GPS,
> **you do not know whether the radar solution is good enough to fly on.** Everything
> before G10 is bench work. Do not jump from G7 (SITL) to G12 (GUIDED).

---

## 11. Verification against GPS

`gps_logger.py` already records GPS→ENU, RIO velocity, SLAM pose, and IMU into one time-aligned
JSONL. Compute these four metrics after every flight:

| Metric | Definition | Target (G11) |
|---|---|---|
| **Velocity RMSE** | `RMS(v_RIO_nav − v_GPS)` per axis, after Umeyama time alignment | < 0.25 m/s horizontal, < 0.35 m/s vertical |
| **Velocity scale** | slope of `\|v_RIO\|` vs `\|v_GPS\|` regression | 1.00 ± 0.02 (a scale error is a tilt error, §2.9) |
| **ATE** | absolute trajectory error, SLAM vs GPS, after SE(2) alignment | < 3% of path length |
| **Closure error** | ‖end − start‖ on an out-and-back | < 2% of path length |
| **Altitude RMSE** | U200A AGL vs GPS altitude minus takeoff | < 0.5 m |
| **Yaw drift** | SLAM yaw minus GPS course-over-ground during straight legs | < 2°/minute |

Add to `tools/analyze_run.py`:

```python
def velocity_scale_and_bias(v_rio_nav, v_gps):
    """A scale != 1.0 means the mount tilt is wrong. Solve for the implied
    tilt error: v_apparent = v_true * cos(dtheta), so
        dtheta = acos(scale)  -- then RE-MEASURE the mount (Sec 2.9 Table I).
    A per-axis BIAS means the lever arm or the AHRS trim is wrong."""
    mask = np.linalg.norm(v_gps, axis=1) > 0.5
    A = np.c_[np.linalg.norm(v_gps[mask], axis=1), np.ones(mask.sum())]
    scale, bias = np.linalg.lstsq(A, np.linalg.norm(v_rio_nav[mask], axis=1), rcond=None)[0]
    implied_tilt_err = math.degrees(math.acos(np.clip(scale, -1, 1)))
    return scale, bias, implied_tilt_err
```

**This is the single most valuable post-flight number.** If `scale = 0.985`, your real tilt is
off by 10° and you should re-measure the mount before doing anything else.

---

## 12. Cases you may have missed

You asked me to look for these. These are real, and several are flight-stoppers.

### 12.1 Doppler velocity aliasing
The U300's unambiguous velocity is ±45 m/s, so you have margin — **but confirm it for your
firmware's chirp configuration.** If Linpowave ships a config with a longer chirp for the 350 m
mode, `v_max` drops as `λ/(4·T_c)` and could be ±8–12 m/s. Above `v_max` the Doppler wraps and
RIO returns a confident, completely wrong velocity with a good inlier count. **Ask Linpowave for
`v_max` at your configured range profile** (§14 Q6), then set
`nav_node --max-speed ≤ 0.4 × v_max`.

### 12.2 Rotor-blade returns
A nose radar at a 40° down-tilt on a multirotor will see the front rotor discs in its azimuth
FOV at 0.3–0.8 m range. Blade-tip Doppler is ±50–100 m/s — well outside any static-world
hypothesis — so RANSAC will reject them, **but they consume the point budget** (the U300 reports
a limited number of detections per frame) and can dominate the CFAR threshold, suppressing weak
ground returns.

- Raise `--leakage-radius` from 0.35 m to **0.8 m**.
- Better: mount the radar **forward of and below the rotor plane**, so the discs are outside
  the ±40° azimuth fan.
- Verify on the bench with motors spinning at 30% throttle, props on, aircraft restrained:
  compare `n_total` and the range histogram with motors on vs off.

### 12.3 Terrain type — the silent killer
Everything in Section 2 assumes diffuse terrain (grass, crops, soil, gravel) with
`σ⁰ ≈ −15 to −25 dB`. Three surfaces break it:

| Surface | Behaviour | Consequence |
|---|---|---|
| **Calm water** | specular — all energy forward-scattered away from the radar | **near-zero returns**. RIO reports "static", B4 fires. **Do not fly over water.** |
| **Smooth asphalt / concrete** | strongly specular at 28–52° incidence | `R_gnd` drops by 2–3× |
| **Fresh snow** | low σ⁰ at W-band | `R_gnd` drops; also causes false altimeter returns from sub-surface layers |
| **Dense forest canopy** | good returns, but from the canopy top, not the ground | the altimeter reads *height above canopy*; a sudden clearing is a 20 m step |

**TEST-01 must be run over the actual mission terrain.** A ceiling measured over a grass field
is not valid over a paved apron.

### 12.4 Terrain slope and the altimeter
The U200A reports the nearest return in its beam, not the vertical distance. Over a 20° slope,
flying downhill, it reads the uphill side — a bias of `h·(1 − cos 20°) ≈ 6%`, i.e. 3 m at 50 m.
Flying **towards** rising terrain, the reading shrinks faster than the true AGL and the EKF
commands a climb. `EK3_RNG_USE_SPD = 8.0` limits the damage by disabling rangefinder height
above 8 m/s; also cap `--max-speed` and prefer level terrain for early flights.

### 12.5 The aircraft's own pitch swings the beam
In forward flight a multirotor pitches nose-down by `atan(a/g)`. At 3 m/s² that is 17°. Your
40° mount becomes a **57° effective depression** during acceleration and **23° during
braking**. Two effects:
- The beam sweeps across 34° of depression during a normal accel/decel cycle. Table B says
  DOP_Vx moves from 5.8 to 3.3 — actually a *benefit* for forward velocity, but the ground
  patch also translates several tens of metres, breaking SLAM correspondence.
- With `imu_level_points = True` this is handled correctly (points are rotated into the
  levelled frame). **Which is precisely why S-01 must not be left to chance.**
- Set `ANGLE_MAX = 2000` (20°) to bound it.

### 12.6 Radar-to-radar interference
Both units are 76–81 GHz FMCW. The nose radar's sidelobes can illuminate the ground patch the
belly radar is measuring, and vice versa. Symptoms: intermittent altimeter spikes, or ghost
detections at implausible ranges correlated between the two.

- Physically separate them and orient so neither is in the other's main beam (a 40° nose-down
  and a nadir belly unit are ~50° apart — acceptable).
- Ask Linpowave whether the U300 and U200A can be **configured to non-overlapping sub-bands**
  (e.g. 76–78 and 79–81 GHz) (§14 Q6).
- The median+rate filter in `altimeter_bridge` (§5.5 item 6) is your software mitigation.

### 12.7 Repetitive terrain defeats GICP
A ploughed field, an orchard, a solar farm, or a car park has a strong spatial periodicity.
GICP will happily converge to a **wrong local minimum one furrow/row over** — a clean,
confident, systematically wrong registration. This is what the 20 m pose-jump rejector in
§8.2 is for. `_project_onto_observable_subspace` (line 341) helps, but it tests observability,
not correctness.

### 12.8 `KeyframeAccumulator` deskew is the hidden accuracy limit
`accum.build()` merges 0.3–0.5 s of frames into one keyframe using the RIO velocity to deskew.
At 5 m/s that is 1.5–2.5 m of motion. If the RIO velocity is off by 5%, every keyframe has
10 cm of built-in smear, which becomes GICP noise. **Reduce `--window-s` to 0.2 s for flight**
and raise `--slam-decimation` instead, so keyframes are shorter in time but no more frequent.

### 12.9 Jetson thermal throttling and GICP latency
`slam_node.py:503` already warns above 150 ms. On an Orin Nano at sustained load with a passive
heatsink in 35 °C ambient (Dehradun in summer), the CPU will throttle and GICP will exceed the
keyframe period, causing the `UdpReceiver` queues to back up. Pin the clocks:
```bash
sudo nvpmodel -m 0 && sudo jetson_clocks
```
and log `tegrastats` alongside the flight. Consider `--min-correspondences` and
`--voxel-size` increases if GICP time grows with map size (it will — the map cloud grows to
200 k points).

### 12.10 Map growth is unbounded in time
`_merge_into_map` (line 547) voxel-downsamples but the map grows until it hits the 200 k cap,
at which point it **randomly subsamples** (line 554), which silently deletes recently observed
structure. For a 10-minute flight over new terrain this degrades badly. Implement a **sliding
local map**: keep only points within ~150 m of the current pose, and write evicted points to
the `.pcd` file.

### 12.11 No time-sync between the Jetson and the Cube
`imu_bridge.py` stamps with Jetson `monotonic`; `doppler_rio` stamps with (after D-06)
Jetson `monotonic`. Those agree. But the Cube's ATTITUDE is stamped by the Cube and the
association is made by *arrival time*. At 50 Hz ATTITUDE over a 115200 serial link with other
traffic, the attitude you associate with a radar frame can be 20–40 ms old — which at 60 °/s
yaw is 2.4° of heading error injected into the levelling. **Use TELEM2 at 921600, not USB at
115200, and raise the ATTITUDE rate to 100 Hz** (`SR2_EXTRA1 = 100`).

### 12.12 Log volume
`gps_logger` writes JSONL at 20 Hz across three streams. Your `logs/` directory already has
90+ runs. Add rotation and make sure the Jetson's eMMC does not fill mid-flight — a failed
write in `gps_logger` is non-critical (`critical=False`), but a full filesystem will also stop
`_save_map` and the stdout `tee`.

---

## 13. Implementation order

Do them in this order. Each is independently testable.

**Phase A — make it safe to fly at all (blocks everything):**
1. **N-01** type_mask → 1479, plus the assertion in `sitl_test.py`
2. **N-02 / N-02b** obstacle sentinel
3. **N-03** pass `attitude` into `on_slam_pose`, latch `yaw_offset`
4. **D-01** remove `force_2d` from the flight path, `cov[2,2] = 1e6`
5. **D-02** static-hypothesis covariance gate
6. Re-run G0, G6, G7 in SITL. **Nothing else until SITL flies a clean square.**

**Phase B — wire in the altimeter (blocks active nav):**
7. **B6 / §5.5** write `altimeter_bridge.py`
8. **M-02** `DISTANCE_SENSOR` limits; **M-03** `select()` loop
9. **N-04** AGL from the altimeter in the geofence
10. **S-02** altimeter Z constraint in `_gravity_correct`
11. Gate **G5**

**Phase C — correctness and consistency:**
12. **D-03** real lever-arm rotation compensation (+ `filters.py`)
13. **S-01** unhardcode `imu_level_points`, enforce the pairing
14. **D-04** mandatory tilt/lever args; **D-05** `max_range 150`; **D-06** monotonic clocks
15. **M-01** timestamp sync; run **G9**, set `EK3_VIS_DELAY_MS`
16. Gates **G1–G4**

**Phase D — navigation behaviour:**
17. **N-06** split vertical/horizontal speed limits
18. **N-07 / N-07b** failsafe ladder + EKF health
19. **N-08** refuse to arm; require airborne + healthy EKF
20. **N-11 / N-12** breadcrumb RTH, map→NED helper with yaw-rate
21. **§8.2** SLAM pose-jump rejector
22. **N-05, N-09, N-10** ceiling, docstring, waypoint echo

**Phase E — flight:**
23. Load the parameter set (§7), verify `SET_GPS_GLOBAL_ORIGIN` lets GUIDED arm in SITL
24. Gates **G8 (TEST-01)**, **G10**, **G11**, then **G12**

---

## 14. What I need from you before I write the code

These are blocking. I have deliberately left every one of them as a `<placeholder>` in the
commands and parameters above rather than guessing — a guessed lever arm or tilt is exactly the
class of error that Table I shows costs 20 m of phantom altitude per minute.

### 🔴 Blocking — geometry

**Q1. Measured tilt angle of the U300.**
Not the design intent — the number from a digital inclinometer resting on the radar's mounting
face, with the airframe levelled on a flat surface, landing gear down, battery fitted.
→ `θ = ______ °` (target 40.0; report to 0.1°)

**Q2. U300 nose radar lever arm, from the Cube Orange's IMU to the radar's antenna phase centre**,
in **body FRD metres** (X forward, Y right, **Z down**):
```
Lx = ______ m   (+ = radar is AHEAD of the FC)
Ly = ______ m   (+ = radar is to the RIGHT of the FC)
Lz = ______ m   (+ = radar is BELOW the FC)
```
Measure to the **antenna face**, not the connector or the enclosure corner. ±1 cm is enough for
the position term but this now also scales the rotational velocity correction (D-03).

**Q3. U200A belly altimeter lever arm**, same convention:
```
Ax = ______ m       Ay = ______ m       Az = ______ m  (+ = below the FC)
```
`Az` directly biases every altitude in the system. Measure to the antenna face.

**Q4. Is the U300's boresight in the body XZ plane?** i.e. is there any yaw or roll
misalignment of the mount? If the bracket has a yaw error `ψ_mount`, I need to add it to
`TiltMount` — it does not currently exist, and per Table H a 2° mount yaw is 17.5 m of
cross-track over 500 m.
→ `ψ_mount = ______ °`, `φ_mount = ______ °` (or "both < 0.5°, ignore")

**Q5. `lateral_sign` bench result.** Place a corner reflector physically to the **right** of
boresight and report the sign of the resulting body-frame Y (Gate G1).
→ `lateral_sign = +1.0 / −1.0`

### 🔴 Blocking — sensors

**Q6. From Linpowave, for your firmware build:**
- Confirmed azimuth and elevation FOV (**your brief says 100°×40°, the datasheet says ±40°×±12°** — §1.1)
- **Max unambiguous velocity `v_max`** at your configured range profile (§12.1)
- Max detections reported per frame
- Whether the U300 and U200A can be put on **non-overlapping sub-bands** (§12.6)
- Whether the U300 firmware can expose a **Side Info / SNR TLV** — this would let
  `polar_uncertainty_weights` use true `SNR/R²` weights instead of the `1/R²` stand-in
  (`doppler_rio.py` docstring lines 19–26)

**Q7. The U200A protocol document / SDK.** I cannot write `altimeter_bridge.py` without the
frame format. Send the equivalent of `Points_float protocol and analysis.pdf` for the U200A,
plus its baud rate and whether it is on TTL, CAN, or RS422.

### 🟠 Blocking — operations

**Q8. Operating terrain and altitude.**
- Surface type at the test site (grass / soil / crops / asphalt / mixed)? (§12.3)
- Terrain slope? (§12.4)
- **What is the maximum AGL you actually need?** If the answer is "100–200 m", read §2.6 again —
  the honest answer is that this sensor pair cannot do it, and we should agree on a 50 m ceiling
  now rather than discover it at 80 m with a diverging estimator.

**Q9. Airframe.**
- Multirotor or fixed wing? All-up weight, prop diameter, rotor-plane-to-nose-radar geometry (§12.2)
- Is there carbon fibre anywhere in the radar's FOV? (§3.1)
- Max planned ground speed?

**Q10. Which physical links?**
- Cube ↔ Jetson: USB (`/dev/ttyACM0`) or TELEM2 serial? **I strongly recommend TELEM2 @ 921600** (§12.11)
- One MAVLink connection shared, or separate links for aiding vs control? (§6.3)

**Q11. Is the compass trustworthy?** `--trust-imu-yaw` should be ON for flight (§5.3 S-02),
but only with a compass calibrated on the actual airframe, away from metal, with
`COMPASS_LEARN = 0`. Have you run a CompassMot calibration with motors under load?
→ If no, say so — the fallback is GICP yaw, and the drift budget in Table H gets worse.

**Q12. Confirm the GPS policy.** My plan sets `EK3_SRC1_POSXY = 0` so GPS is *structurally*
excluded from the estimator while still being logged (§6.4), with an RC switch to a GPS source
set as a safety net. Confirm you want it this strict — it is the only way to honestly validate
the radar solution, but it means a first flight with **no absolute position aiding at all**.

---

## 15. One-line summary of the plan

> Mount the U300 at a **measured 40° nose-down**; wire the **U200A into a new
> `altimeter_bridge.py`** because it is currently absent and it is the only thing that bounds
> your Z; fix the **eleven defects in Section 5** — of which `type_mask`, the `-1.0` obstacle
> sentinel, the never-computed `yaw_offset`, the over-confident static hypothesis, and
> `force_2d` are individually sufficient to lose the aircraft; feed **RIO velocity + U200A
> range** into EKF3 with GPS structurally excluded; navigate on **EKF3's own state, not a
> second Python estimator**; return home by **retracing breadcrumbs**, not by trusting RTL; and
> accept a **50 m ceiling**, because 200 m AGL over diffuse terrain is not a tuning problem, it
> is a range-equation problem with no solution.

*Document generated 2026-09-16 from a full audit of the uploaded `radar` tree.
All tables computed with the reproducible script in `docs/tilt_analysis.py`.*
