# Stage 4 — Changes vs Results Log

> **How to use this file:**
> Every time a code change is suggested, it is added here under **PENDING** status.
> After the change is applied and a test is run, the result is filled in and status moves to **DONE**.
> This file is never deleted — only appended to. It is the single source of truth for what changed and why.

---

## Change #001 — Teammate's `--imu-level-points=False` flag

| Field | Value |
|-------|-------|
| **Date** | Sep 14, 2026 |
| **Status** | ❌ REVERTED |
| **File** | `src/doppler_rio.py` (via CLI flag) |
| **Changed by** | Teammate |

### What Changed
Teammate added `--imu-level-points=False` as a CLI flag, disabling IMU-based point cloud leveling.

### Why (Teammate's Reasoning)
Believed the IMU pitch offset was contaminating the velocity estimate.

### The Bug This Created
By disabling `imu_level_points`, the RIO solver operated in the Body frame (tilted 90°). But `force_2d` still forced Z-velocity to zero. Because Body-Z = forward walking direction, this zeroed out all forward motion — a complete hallucination.

### Test Result
- RIO: `v_body = [0, 0, 0]` almost always
- GPS vs RIO error: >200% — completely unusable

---

## Change #002 — Fix `force_2d` in `doppler_rio.py`

| Field | Value |
|-------|-------|
| **Date** | Sep 15, 2026 (morning) |
| **Status** | ✅ DONE — KEEPER |
| **File** | `src/doppler_rio.py` |
| **Changed by** | AI Agent (approved by Alok) |

### What Changed
Made `force_2d` conditional on `imu_level_points`. When `imu_level_points=False`, Body-Z = forward walking direction — zeroing it is catastrophically wrong.

```python
# BEFORE: always zeros vz
if self.force_2d:
    v[2] = 0.0

# AFTER: only zero vz if in Earth-leveled frame
if self.force_2d and self.imu_level_points:
    v[2] = 0.0
```

### Test Result (runs `164940` → `165313`)
- RIO mean speed: **1.0 m/s** ✅ (was 0.0 m/s before)
- RIO inliers: 40–70 per frame ✅
- RIO integrated distance: **41.5 m** for a ~40m walk ✅

---

## Change #003 — Fix omega cross-product frame in `doppler_rio.py`

| Field | Value |
|-------|-------|
| **Date** | Sep 15, 2026 (morning) |
| **Status** | ✅ DONE — KEEPER |
| **File** | `src/doppler_rio.py` |
| **Changed by** | AI Agent (approved by Alok) |

### What Changed
Fixed rotation compensation cross-product `omega × r` which was computed in the wrong frame.

### Why
Frame mismatch added ~0.2–0.3 m/s spurious velocity during turns (omega ≈ 0.7 rad/s).

### Test Result
Velocity noise reduced during turning. Mean speed more stable.

---

## Change #004 — Revert `doppler_eps_mps` to 0.40 in `filters.py`

| Field | Value |
|-------|-------|
| **Date** | Sep 14–15, 2026 |
| **Status** | ✅ DONE — KEEPER |
| **File** | `src/filters.py` |
| **Changed by** | AI Agent |

### What Changed
Reverted `doppler_eps_mps` from 0.15/0.20 back to **0.40 m/s**.

### Why
Tighter threshold was for indoor drone. Handheld walking rig has ~0.2–0.3 m/s lever-arm bounce. 0.40 needed to maintain inlier count.

### Test Result
Inlier count improved from ~20 → 40–70 per frame. RIO stable.

---

## Change #005 — Bake outdoor walking defaults into `supervisor.py`

| Field | Value |
|-------|-------|
| **Date** | Sep 15, 2026 (morning) |
| **Status** | ✅ DONE — KEEPER |
| **File** | `src/supervisor.py` |
| **Changed by** | AI Agent (approved by Alok) |

### What Changed

| Parameter | Old Default | New Default |
|-----------|------------|------------|
| `voxel_size` | 1.5 m | 0.10 m |
| `max_corr_dist` | 6.0 m | 2.0 m |
| `min_correspondences` | 15 | 4 |
| `persistence_radius` | 1.0 m | 0.50 m |
| `persistence_min_hits` | 3 | 1 |
| `lambda_min_observable` | 10.0 | 3.0 |
| `observable_ratio` | 0.50 | 0.25 |
| `doppler_eps` | 0.20 | 0.40 |

### Why
Old defaults were for indoor drone flight. Open-field outdoor walking needs looser thresholds.

### Test Result
Simple run command works correctly. SLAM began accepting keyframes consistently.

---

## Change #006 — Force `imu_level_points=True` in `slam_node.py`

| Field | Value |
|-------|-------|
| **Date** | Sep 15, 2026 (afternoon) |
| **Status** | ❌ REVERTED / NEEDS REPLACEMENT — see Change #008 |
| **File** | `src/slam_node.py` |
| **Changed by** | AI Agent |

### What Changed
Hardcoded `imu_level_points=True` on line 682, overriding the supervisor flag.

### Why This Was Wrong
The IMU reports 65–72° pitch because the rig is physically tilted 90° (IMU X-axis points at sky). This is NOT a calibration error — it is the correct physical reading. Forcing `imu_level_points=True` made SLAM rotate every point cloud by 70° before adding to the map — building the map 70° sideways into the sky.

### Test Result (runs `175806`–`180007`)
- SLAM Z drift: **7–15 m** for 37 m walks ❌
- SLAM drift rate: **1.1 m/s** (gate: < 0.05 m/s) ❌
- RIO accuracy: **0.4%–4.7%** ✅ (RIO unaffected)

---

## Change #007 — IMU Physical Orientation Verified

| Field | Value |
|-------|-------|
| **Date** | Sep 15, 2026 |
| **Status** | 📋 KNOWLEDGE — No code change |

### Physical Setup (Confirmed by Alok)
- Drone/FC mounted flat; radar beneath belly faces DOWN.
- When held handheld with radar pointing at horizon, entire rig is pitched 90° forward.
- **IMU X-axis (nose)** → **Sky**
- **IMU Y-axis (right)** → **Right** (unchanged)
- **IMU Z-axis (belly/down)** → **Forward**

### Consequence
- Turning body left/right (world Yaw) = **Roll** in IMU frame (rotation around IMU X = sky).
- IMU pitch of 65–72° is physically correct. Any code using it for geometric corrections will apply wrong rotations.
- RIO is safe: uses `--no-imu-level-points` by default.
- SLAM must do the same.

---

## ⏳ Change #008 — PENDING APPROVAL — Fix `slam_node.py` (Two Parts)

| Field | Value |
|-------|-------|
| **Date** | Sep 15, 2026 |
| **Status** | ⏳ AWAITING ALOK'S APPROVAL — NO CODE TOUCHED |
| **File** | `src/slam_node.py` |
| **Suggested by** | AI Agent |

### Part A — Line 682: Respect supervisor flag

```diff
-xyz_body = mount.to_body(pts_radar[:, :3], latest_attitude,
-                         imu_level_points=True)
+xyz_body = mount.to_body(pts_radar[:, :3], latest_attitude,
+                         imu_level_points=slam.imu_level_points)
```

**Why:** With `imu_level_points=True`, SLAM applies the rig's 70° physical tilt to every point cloud, building the map sideways. With `slam.imu_level_points` (=False by default), SLAM works in Body frame ignoring the swapped IMU axes.

### Part B — Lines 533-534: Make `_gravity_correct()` conditional

```diff
-if self.last_attitude is not None:
-    self.T_world = self._gravity_correct(self.T_world)
+if self.imu_level_points and self.last_attitude is not None:
+    self.T_world = self._gravity_correct(self.T_world)
```

**Why:** `_gravity_correct()` assumes world-Z = UP (Earth frame). In Body frame (imu_level_points=False), world-Z = FORWARD. Clamping "yaw around forward" is physically meaningless and corrupts the rotation matrix every keyframe — proven by run `185053` where SLAM tracked only 15m of a 40m walk.

### Expected Result After These Two Changes
- SLAM will build map in Body frame (Z = forward walking direction)
- No IMU pitch contamination
- GICP matches freely without artificial rotation constraints
- SLAM Z ≈ 35–40 m for a 40 m straight walk (this is CORRECT, not drift)

### Hardware Check Before Next Test
- **Check GPS antenna** — run `185242` reported 497 m GPS for a ~40 m walk (antenna may be loose)
- Wait for **≥12 GPS satellites** before walking
- Walk in a **straight line** — do not turn back, prevents false loop-closure

---

*Last updated: Sep 15, 2026 19:34 IST*


## ⏳ Change #009 — PENDING APPROVAL — Fix RIO Distance and SLAM XY Drift

| Field | Value |
|-------|-------|
| **Date** | Sep 18, 2026 |
| **Status** | ⏳ AWAITING ALOK'S APPROVAL |
| **File** | `N/A` (Algorithm Tuning & Hardware Cal) |
| **Suggested by** | AI Agent |

### What Needs to Change (XY Drift < 5m)
Currently, SLAM drifts 6-9m over 300m (~2.3% error), which is mathematically expected for open-loop mmWave SLAM. To eliminate this:
1. **Loop Closure (Factor Graph):** Implement GTSAM or g2o so the drone recognizes its own past trajectory and mathematically erases accumulated XY drift upon returning.
2. **IMU Pre-integration:** Tightly couple the IMU rotation rate into the GICP initial guess to constrain horizontal slipping.

### What Needs to Change (RIO Distance Error)
Currently, RIO underestimates distance by up to 60% at high speeds (20 m/s).
1. **Radar Hardware Saturation:** The U300 max unambiguous velocity (Nyquist limit) is likely capping out around 20 m/s. **Fix:** Flash radar firmware to increase max velocity (trade-off with range resolution).
2. **Pitch Bias Calibration:** A 1° error in the mechanical tilt calibration vs software tilt (`--tilt-deg 40.0`) will massively leak forward velocity into the Z-axis when flying at 70 km/h. **Fix:** Perform a rigorous static pitch calibration on the bench.
3. **Verify `force_2d` is OFF:** Ensure `--force-2d` is never passed to `supervisor.py`. Forcing Z-velocity to zero while the drone is heavily pitched down will completely destroy 3D distance integration.

*Last updated: Sep 18, 2026 11:15 IST*
