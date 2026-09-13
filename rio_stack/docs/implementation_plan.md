# Stage 4 Implementation Plan — Cross-Verified & Corrected

## Executive Summary

I cross-verified Claude's Stage 4 plan against the actual codebase and ran empirical tests on your machine. The plan is **mostly excellent**, but contains **two critical errors** that would have broken the system if implemented as-is. This corrected plan fixes those errors.

---

## Cross-Verification Results

### ✅ What Claude Got RIGHT

| Item | Verdict |
|------|---------|
| **IRLS math** (Huber weights, convergence, gross-outlier gate) | Correct. The formulas are standard M-estimation. |
| **Polar Jacobian** (`∂(x,y,z)/∂(r,az,el)`) | Correct. I verified by finite-difference — matches. |
| **Projection onto residual** (`v_perp`) | Correct. Standard chain-rule identity. |
| **`weighted_refit()` stays unchanged** as seed solver | Correct. No need to touch it. |
| **Eigendecomposition for observability** | Correct concept. The subspace-projection math is standard. |
| **`_is_planar_degenerate` should be deleted** | Correct. It's the brittle heuristic being replaced. |
| **`force_2d` removal from RANSAC/refit** | **Not mentioned by Claude** but important: since IRLS now refines in full 3D body frame (using proper covariance-aware weighting), the `force_2d` flag in RANSAC and `weighted_refit` is no longer needed when attitude is available. However, I will keep `force_2d` in RANSAC for now (it's a separate, independent guard) and only remove it from the IRLS path. |

### ❌ What Claude Got WRONG

#### Critical Error 1: `get_information_matrix_from_point_clouds` CANNOT detect translational degeneracy

> [!CAUTION]
> Claude proposed using `o3d.pipelines.registration.get_information_matrix_from_point_clouds()` to build the Hessian. **I empirically tested this on your machine** and found that its translation block `H[3:6, 3:6]` is always `N_correspondences × I₃ₓ₃` — it uses a **point-to-point** counting scheme internally, NOT point-to-plane Jacobians. On a perfectly flat floor with all normals pointing up, it returns eigenvalues `[300, 300, 300]` — meaning it thinks X, Y, Z are all equally observable. This is mathematically wrong.
>
> **The correct manual Hessian** (which I also verified on your machine) returns `[0, 0, 300]` on the same flat floor — correctly identifying X and Y as unobservable and Z as observable.

**Fix:** Build the point-to-plane Hessian ourselves from correspondences + normals. The math is simple and verified:

```
For each correspondence (source p_i, target normal n_i):
  J_i = [(p_i × n_i) | n_i]     ← 1×6 row vector
  H += J_iᵀ J_i                  ← 6×6 accumulation

Result: H_tt = Σ nᵢ nᵢᵀ
  - Flat floor (all n=[0,0,1]): eigenvalues = [0, 0, N]  → X,Y unobservable ✅
  - Wall at X=5 (n=[1,0,0]):    adds observability to X   ✅
  - Full structure:             all eigenvalues large      ✅
```

#### Critical Error 2: `TiltMount.current_rotation()` with hardcoded `R_imu_mount`

Claude proposed a `current_rotation()` method that hardcodes an `R_imu_mount` matrix. **This matrix does not exist anywhere in the current codebase** — `to_body()` uses `R_level @ self.R_static` directly (line 192 of doppler_rio.py). Claude fabricated `R_imu_mount` from nothing. 

**Fix:** `current_rotation()` simply returns the same rotation that `to_body()` uses:

```python
def current_rotation(self, attitude=None):
    if attitude is not None:
        return self.get_R(attitude) @ self.R_static
    return self.R_static
```

#### Non-Critical Issue: Claude's "Stage 3 bugs" warning

Claude warns about two "unresolved Stage 3 bugs" — `R_tilt` dropped from `to_body()`'s dynamic branch, and inverted rotation sign in `KeyframeAccumulator.build()`. **I checked the actual code:**

- [doppler_rio.py line 192](file:///home/alok/radar/rio_stack/src/doppler_rio.py#L192): `R_dynamic = R_level @ self.R_static` — `R_static` IS `R_tilt @ P`. Not dropped. ✅
- [slam_node.py line 173](file:///home/alok/radar/rio_stack/src/slam_node.py#L173): `dtheta = -omega_hint * dt` — the negative sign IS correct (Rodrigues counter-rotation). ✅

**These bugs do not exist.** Claude hallucinated them from a prior version of the code.

---

## Proposed Changes (Corrected)

### File 1: [`doppler_rio.py`](file:///home/alok/radar/rio_stack/src/doppler_rio.py)

#### [MODIFY] `TiltMount` — add `current_rotation()` accessor

A simple helper that returns the same rotation `to_body()` uses, so Stage 4C doesn't duplicate the R_static/R_dynamic selection logic.

```python
def current_rotation(self, attitude=None):
    """Returns the rotation matrix to_body() would use for this attitude state."""
    if attitude is not None:
        return self.get_R(attitude) @ self.R_static
    return self.R_static
```

Add after [line 198](file:///home/alok/radar/rio_stack/src/doppler_rio.py#L198).

---

#### [ADD] `polar_uncertainty_weights()` — Stage 4C

New module-level function. Implements the verified polar-to-Cartesian covariance projection. Re-evaluated every IRLS iteration with the current velocity estimate.

**Math (verified by finite-difference on this machine):**
```
σ²_total,i = σ_v² + (1/r²)·v_perpᵢᵀ · R·J·Σ_polar·Jᵀ·Rᵀ · v_perpᵢ
w_meas,i   = 1 / σ²_total,i
```

Add after the new `current_rotation()` method.

**Sensor noise defaults** (placeholders — must be calibrated from U300 datasheet):
- `σ_r = 0.10 m`
- `σ_az = 2.0°`
- `σ_el = 4.0°`  
- `σ_v = 0.05 m/s`

---

#### [ADD] `irls_refit()` — Stage 4A

New module-level function. IRLS refinement over the full candidate set (not just the RANSAC mask), seeded by `weighted_refit()`.

**Key design decisions:**
- **Max iterations:** 4 (capped to [3,5] range, configurable)
- **Gross outlier gate:** One-time hard gate at `10×δ` at the seed, before IRLS starts
- **Covariance:** `(AᵀWA)⁻¹` at final iterate (standard IRLS approximation, not the full Huber sandwich — documented, not hidden)
- **Convergence:** `‖v^(k) − v^(k-1)‖ < 1e-3 m/s`

Add after `polar_uncertainty_weights()`.

---

#### [MODIFY] `DopplerRIO.__init__` — new config parameters

Add Stage 4A + 4C parameters: `huber_delta_mps`, `irls_max_iters`, `irls_tol_mps`, `gross_outlier_mult`, `sigma_r_m`, `sigma_az_rad`, `sigma_el_rad`, `sigma_v_mps`.

Modify at [line 370](file:///home/alok/radar/rio_stack/src/doppler_rio.py#L370).

---

#### [MODIFY] `DopplerRIO.process_frame` — integrate IRLS

- Track `xyz_radar_native = points_radar[:, 0:3]` alongside body-frame xyz (needed for 4C's polar Jacobian)
- Replace the `weighted_refit()` call in the non-static branch with `irls_refit()`, using `weighted_refit()` only as the seed
- Pass `R_used = self.mount.current_rotation(attitude)` to `irls_refit()`

Modify at [line 385](file:///home/alok/radar/rio_stack/src/doppler_rio.py#L385).

---

#### [MODIFY] argparse block — new CLI flags

Add: `--huber-delta`, `--irls-max-iters`, `--irls-tol`, `--gross-outlier-mult`, `--sigma-r`, `--sigma-az-deg`, `--sigma-el-deg`, `--sigma-v`

Modify at [line 588](file:///home/alok/radar/rio_stack/src/doppler_rio.py#L588).

---

### File 2: [`slam_node.py`](file:///home/alok/radar/rio_stack/src/slam_node.py)

#### [ADD] `RadarSLAM._build_point_to_plane_hessian()` — the CORRECT observability analysis

> [!IMPORTANT]
> This replaces Claude's proposal to use `get_information_matrix_from_point_clouds()`. That API cannot detect translational degeneracy (empirically verified).

New method that:
1. Takes the GICP result's correspondence set + source/target clouds
2. Builds the 6×6 point-to-plane Hessian manually: `H = Σ Jᵢᵀ Jᵢ` where `Jᵢ = [(pᵢ×nᵢ) | nᵢ]`
3. Returns the symmetric 6×6 matrix

```python
def _build_point_to_plane_hessian(self, source, target, correspondence_set, T):
    """Build the 6x6 point-to-plane information matrix from GICP correspondences.
    
    Unlike Open3D's get_information_matrix_from_point_clouds (which uses
    point-to-point counting and CANNOT detect translational degeneracy),
    this builds H = Σ J_i^T J_i from point-to-plane Jacobians:
      J_i = [(R·p_i) × n_i | n_i]
    where p_i is the transformed source point and n_i is the target normal.
    
    Empirically verified: on a flat floor with all normals = [0,0,1],
    translation eigenvalues = [0, 0, N] (X,Y unobservable, Z observable).
    """
    ...
```

Add as a method on `RadarSLAM`.

---

#### [ADD] `RadarSLAM._project_onto_observable_subspace()` — Stage 4B core

Same math as Claude proposed (this part was correct):
```
Δt_gicp         = t_gicp − t_pred
Δt_eig          = Vᵀ · Δt_gicp        (express in eigenbasis)
Δt_eig_filtered = Δt_eig ⊙ observable  (zero unobservable components)
Δt_filtered     = V · Δt_eig_filtered
t_final         = t_pred + Δt_filtered
```

Config: `lambda_min_observable=10.0`, `observable_ratio=0.05` (both need bench calibration).

---

#### [DELETE] `RadarSLAM._is_planar_degenerate()` 

Remove the static method at [line 259](file:///home/alok/radar/rio_stack/src/slam_node.py#L259). No longer needed.

---

#### [MODIFY] `RadarSLAM.__init__` — new config

Add `lambda_min_observable` and `observable_ratio` parameters.

Modify at [line 197](file:///home/alok/radar/rio_stack/src/slam_node.py#L197).

---

#### [MODIFY] `RadarSLAM.process_keyframe` — always run GICP, then gate by Hessian

Replace the `off_plane_ratio` / `is_degenerate` block ([lines 400-411](file:///home/alok/radar/rio_stack/src/slam_node.py#L400-L411)) with:
1. Always run GICP (no pre-bypass)
2. After GICP converges, build point-to-plane Hessian from correspondences
3. Eigendecompose the translation block
4. Project GICP's translation correction onto observable subspace only
5. Keep GICP's rotation as-is (still corrected by `_gravity_correct()` afterward)

Also remove the hardcoded `dz = 0.0` clamp at [line 452](file:///home/alok/radar/rio_stack/src/slam_node.py#L452) — the Hessian guard now handles this mathematically instead of hardcoding "trust RIO 100% for Z."

Also remove the `dx/dy` clamp at [lines 450-451](file:///home/alok/radar/rio_stack/src/slam_node.py#L450-L451) — the Hessian guard replaces this too. GICP corrections on well-observed axes should not be artificially limited to ±0.20m.

Modify at [line 400](file:///home/alok/radar/rio_stack/src/slam_node.py#L400).

---

#### [MODIFY] argparse block — new CLI flags

Add: `--lambda-min-observable`, `--observable-ratio`

Modify at [line 653](file:///home/alok/radar/rio_stack/src/slam_node.py#L653).

---

#### [MODIFY] `run()` status print — show observable axes count

Change the keyframe-OK print to include `observable_axes=N/3`.

Modify at [line 616](file:///home/alok/radar/rio_stack/src/slam_node.py#L616).

---

### File 3: [`supervisor.py`](file:///home/alok/radar/rio_stack/src/supervisor.py)

#### [MODIFY] `build_children` — pass new flags to `doppler_rio.py` and `slam_node.py`

Forward the new Stage 4 CLI arguments. No new flags needed on `supervisor.py` itself for the first iteration — use the defaults.

---

## Summary of Changes

| File | Function/Section | Action | Stage |
|------|-----------------|--------|-------|
| `doppler_rio.py` | `TiltMount.current_rotation()` | **ADD** | 4C |
| `doppler_rio.py` | `polar_uncertainty_weights()` | **ADD** | 4C |
| `doppler_rio.py` | `irls_refit()` | **ADD** | 4A |
| `doppler_rio.py` | `DopplerRIO.__init__` | **MODIFY** | 4A+4C |
| `doppler_rio.py` | `DopplerRIO.process_frame` | **MODIFY** | 4A+4C |
| `doppler_rio.py` | `weighted_refit()` | **UNCHANGED** (seed only) | — |
| `doppler_rio.py` | argparse | **MODIFY** | 4A+4C |
| `slam_node.py` | `RadarSLAM._build_point_to_plane_hessian()` | **ADD** | 4B |
| `slam_node.py` | `RadarSLAM._project_onto_observable_subspace()` | **ADD** | 4B |
| `slam_node.py` | `RadarSLAM._is_planar_degenerate()` | **DELETE** | 4B |
| `slam_node.py` | `RadarSLAM.__init__` | **MODIFY** | 4B |
| `slam_node.py` | `RadarSLAM.process_keyframe` | **MODIFY** (biggest change) | 4B |
| `slam_node.py` | `dz=0.0` and `dx/dy` clamps | **DELETE** | 4B |
| `slam_node.py` | argparse + status print | **MODIFY** | 4B |
| `supervisor.py` | `build_children` | **MODIFY** (pass-through) | All |

---

## Verification Plan

### Automated
1. Run `python3 src/doppler_rio.py --selftest` — must still pass with IRLS
2. Add a Jacobian finite-difference self-test for `polar_uncertainty_weights` (as Claude suggested — this part was correct)
3. Add a synthetic Hessian test: flat floor → eigenvalues `[0, 0, N]`; floor + wall → all eigenvalues large

### Field Test
Run the exact same walk test you just did (hold stick at ~70° up, `--tilt-deg 90`):
```bash
python3 src/supervisor.py --port /dev/ttyUSB0 --imu-port /dev/ttyACM0 \
  --no-mavlink --voxel-size 0.10 --tilt-deg 90 --max-corr-dist 1.0 \
  --eps 0.10 --save-pcd ../maps/ --gps-port /dev/ttyUSB1 --log-dir logs/
```

**Expected results:**
- RIO distance error: **< 2%** (IRLS will smooth the already-good 0.2% further)
- SLAM Z error: **< 1m** (Hessian guard will trust RIO for Z on flat ground, use GICP for Z near walls)
- SLAM XY error: **< 3m** (GICP corrections no longer clamped to ±0.20m when well-observed)
- Drift rate: **< 0.05 m/s** (the target gate)

---

## Open Questions

> [!IMPORTANT]
> **Sensor noise calibration:** The defaults for `σ_r`, `σ_az`, `σ_el`, `σ_v` are placeholders. Do you have the U300 datasheet with the range/angle resolution specs? If so, share them and I'll plug in the real values.

> [!IMPORTANT]
> **Hessian threshold calibration:** `lambda_min_observable=10.0` and `observable_ratio=0.05` are educated guesses. After the first field test, I'll log the eigenvalues and we can tune these from real data.
