# RIO Cross-Verified Audit Report & Implementation Plan

*Sources cross-checked: Claude audit (44 sections), Consensus AI (100 papers), Teammate V3 audit (4 issues), ChatGPT audit, and direct line-by-line code verification.*

---

## Verdict Summary

All four auditors **agree** the two original diagnoses are correct:
1. ✅ Static-hypothesis trap → zero-publishing
2. ✅ Vx/Vz geometric degeneracy → hallucination spikes
3. ✅ The Vz prior math (`_augment_with_vz_prior`) is **mathematically sound** (confirmed by Claude, Consensus, and ChatGPT independently)
4. ✅ The Huber/IRLS interaction with the fixed-weight prior is **correct** (prior is not Huber-weighted — intentional and correct)

However, the auditors collectively found **real bugs in the new code** that need fixing before flight. Below is every finding, cross-verified against the actual code, with my own verdict on each.

---

## TIER 1: CONFIRMED BUGS (Must fix before flight)

### BUG-1: `--max-speed-mps` is NOT wired into `DopplerRIO`
| Detail | Value |
|---|---|
| **Source** | Claude §17, confirmed by code inspection |
| **Location** | [run_udp_loop](file:///home/alok/radar/rio_stack/src/doppler_rio.py#L803-L819) |
| **The Problem** | CLI arg `--max-speed-mps 18.0` exists but is NEVER passed to `DopplerRIO()`. The constructor has no `max_speed_mps` parameter. The internal functions `weighted_refit` and `irls_refit` hardcode `max_speed_mps=25.0` in their signatures. |
| **Impact** | The 18 m/s safety cap doesn't work. Spikes up to 25 m/s can still pass through. |
| **My Verdict** | ✅ **CONFIRMED BUG.** Line 803-819 constructs `DopplerRIO(...)` without passing `args.max_speed_mps`. |
| **Fix** | Thread `max_speed_mps` through `DopplerRIO.__init__` → `doppler_ransac` → `weighted_refit` → `irls_refit`. |

---

### BUG-2: IRLS solver failure silently returns stale previous-iteration velocity
| Detail | Value |
|---|---|
| **Source** | Claude §16 |
| **Location** | [irls_refit](file:///home/alok/radar/rio_stack/src/doppler_rio.py#L515-L528) |
| **The Problem** | Lines 515-516: on `LinAlgError` or `ValueError`, the code does `break` but does NOT invalidate `v` or `cov`. Lines 526-528: it then returns whatever `v`/`cov` were set in the *previous* successful iteration. |
| **Impact** | A frame where the solver crashes mid-IRLS returns a velocity from a previous iteration as if it were the current answer. Directly violates the "gap over hallucination" philosophy. |
| **My Verdict** | ✅ **CONFIRMED BUG.** The `break` on line 516 falls through to `return v, cov` on line 528, which can contain values from iteration k-1. |
| **Fix** | After the `break`, set `cov = None` so the `if cov is None` check on line 526 catches it. |

---

### BUG-3: `vz_ned` / `airborne` freshness is NOT independently tracked
| Detail | Value |
|---|---|
| **Source** | Claude §15, ChatGPT §1d, Teammate §2 |
| **Location** | [imu_bridge.py](file:///home/alok/radar/rio_stack/src/imu_bridge.py#L120-L172), [IMUListener.get_vz_ned](file:///home/alok/radar/rio_stack/src/doppler_rio.py#L118-L126) |
| **The Problem** | `imu_bridge.py` caches `_vz_ned` and `_airborne` as plain variables (lines 120-121). They are only updated when `LOCAL_POSITION_NED` or `EXTENDED_SYS_STATE` messages arrive (which may be slow or missing). But the timestamp `t_mono` is set from ATTITUDE (line 151), which arrives at 50Hz. So `get_vz_ned(max_age_s=0.3)` checks the ATTITUDE timestamp, NOT the actual age of `_vz_ned`. |
| **Impact** | If `LOCAL_POSITION_NED` stops streaming, a stale `vz_ned` from 10 seconds ago will appear "fresh" because ATTITUDE keeps updating `t_mono`. The Vz prior will inject a **wrong, confidently-weighted vertical velocity** during exactly the maneuvers it's supposed to protect. Similarly, if `EXTENDED_SYS_STATE` never streams, `_airborne` stays `0` for the entire flight, silently disabling the static hardening. |
| **My Verdict** | ✅ **CONFIRMED BUG.** This is the single most dangerous defect. Both ChatGPT and Claude independently found it. |
| **Fix** | Track `_t_vz_ned` and `_t_airborne` as separate timestamps. Update them only when their respective MAVLink messages arrive. In `get_vz_ned()`, check `time.monotonic() - self._t_vz_ned`. In `get_airborne()`, if `time.monotonic() - self._t_airborne > 2.0`, return `True` as a safe default (assume airborne if we don't know). |

> [!IMPORTANT]
> **Teammate was WRONG about the source message.** Teammate said we use `GLOBAL_POSITION_INT` for Vz. Code inspection proves we actually use `LOCAL_POSITION_NED` (line 138-142 of `imu_bridge.py`). The source is correct; the freshness tracking is the real bug.

---

### BUG-4: TAKEOFF/LANDING states bypass static hardening
| Detail | Value |
|---|---|
| **Source** | Claude §13 |
| **Location** | [imu_bridge.py L148](file:///home/alok/radar/rio_stack/src/imu_bridge.py#L148) |
| **The Problem** | `_airborne = 1 if landed_state == 2 else 0`. MAVLink `landed_state=3` (TAKEOFF) and `landed_state=4` (LANDING) both map to `_airborne=0`. During takeoff/landing, 8-10 self-return points at `vr=0` can pass the relaxed 70% static threshold (8/10 = 80% > 70%) and declare the drone stationary while it's still moving. |
| **Impact** | Zero-publishing can reappear during takeoff and landing phases. |
| **My Verdict** | ✅ **CONFIRMED BUG.** |
| **Fix** | Change to `_airborne = 1 if landed_state != 1 else 0` (only ON_GROUND=1 is treated as ground). |

---

### BUG-5: Acceleration gate compares velocities in yaw-rotating frame
| Detail | Value |
|---|---|
| **Source** | Claude §6 |
| **Location** | [process_frame L728-L740](file:///home/alok/radar/rio_stack/src/doppler_rio.py#L728-L740) |
| **The Problem** | `v_body - self._v_prev` is computed in the leveled-yaw frame. During a yaw turn at constant speed (e.g., 10 m/s), the body-frame velocity rotates from `[10,0,0]` to `[0,10,0]`, giving `||Δv||=14.1 m/s`. At `dt=0.1s`, implied accel = 141 m/s² >> 15 m/s² threshold. Valid frames are rejected during every turn. |
| **Impact** | The acceleration gate will reject correct velocity estimates during any turn, creating unnecessary RIO gaps and degrading SLAM. |
| **My Verdict** | ✅ **CONFIRMED BUG.** At your typical flight speeds of 5-10 m/s, even a moderate 20°/s yaw rate will trigger false rejections. |
| **Fix** | Store the yaw angle alongside `_v_prev`. Before comparing, rotate both velocities into NED using their respective yaw angles, then compute the acceleration in NED. |

---

### BUG-6: `filters.py` still uses the old no-op rotational model
| Detail | Value |
|---|---|
| **Source** | Claude §21 |
| **Location** | [filters.py L114-L116](file:///home/alok/radar/rio_stack/src/filters.py#L114-L116) |
| **The Problem** | `v_rot_comp = np.sum(u * np.cross(omega_hint, xyz), axis=1)`. Since `u = xyz/||xyz||`, this computes `u · (ω × p)` which is the scalar triple product `p · (ω × p) / ||p||` = **identically zero** for all ω and p. |
| **Impact** | The Doppler consistency gate in `filters.py` (used by SLAM's point filtering) provides zero rotational compensation. During fast yaw, legitimate ground points can be rejected as "inconsistent", degrading the SLAM point cloud. |
| **My Verdict** | ✅ **CONFIRMED BUG.** `doppler_rio.py` correctly uses the lever-arm model (`ω × r_lever`), but `filters.py` still has the mathematically inert old model. |
| **Fix** | Replace `np.cross(omega_hint, xyz)` with `np.cross(omega_hint, lever_arm)` in `gate_doppler_consistency`, matching `doppler_rio.py`'s corrected physics. |

---

## TIER 2: CONFIRMED ARCHITECTURAL ISSUES (Fix before autonomous flight)

### ARCH-1: Condition-number rejection happens BEFORE Vz prior augmentation
| Detail | Value |
|---|---|
| **Source** | Claude §7 |
| **Location** | [doppler_ransac L370-L376](file:///home/alok/radar/rio_stack/src/doppler_rio.py#L370-L376) vs [process_frame L686-L704](file:///home/alok/radar/rio_stack/src/doppler_rio.py#L686-L704) |
| **The Problem** | RANSAC checks `cond(A) > 12` on the raw radar-only geometry (line 374). If it fails, RANSAC returns `None` (line 375). The Vz prior is only injected LATER in `weighted_refit`/`irls_refit` (lines 393, 500). So in exactly the degenerate geometry the Vz prior was designed to rescue, the frame is already rejected before the prior gets a chance. |
| **Impact** | The Vz prior cannot fix frames where the radar geometry is too narrow. This is the exact scenario it was built for. |
| **My Verdict** | ✅ **CONFIRMED.** Claude's analysis is correct. The prior and the condition check are in the wrong order. |
| **Fix** | Either: (a) Loosen the RANSAC `cond_reject_threshold` to ~50 and let the posterior sigma gate (R2-L3) handle the quality check after the prior has been applied, or (b) compute the fused information matrix `H_F = H_R + H_P` and check observability on that instead. |

---

### ARCH-2: SLAM retains stale RIO velocity during gaps
| Detail | Value |
|---|---|
| **Source** | Claude §27-28, ChatGPT §3, Teammate §3 |
| **Location** | [slam_node.py L263-L277, L495-L498](file:///home/alok/radar/rio_stack/src/slam_node.py#L263-L277) |
| **The Problem** | When RIO rejects a frame, it sends nothing downstream. SLAM's `last_v_body` retains the previous valid velocity indefinitely. `T_pred` extrapolates this stale velocity over a growing `dt`, creating an increasingly wrong GICP initial guess. |
| **Impact** | During aggressive maneuvers (where RIO gaps are most likely), SLAM gets the worst motion prior exactly when it needs the best one. GICP can fail to converge, causing map drift. |
| **Fix** | Add a timestamp to `last_v_body`. If `t - last_v_body_time > 0.3s`, zero out the translation prior or widen the GICP search radius. |

---

### ARCH-3: `weighted_refit` covariance excludes the Vz prior row
| Detail | Value |
|---|---|
| **Source** | Claude §23, ChatGPT §1a |
| **Location** | [weighted_refit L400-L401](file:///home/alok/radar/rio_stack/src/doppler_rio.py#L400-L401) |
| **The Problem** | `ATA = A_w[:-1].T @ A_w[:-1]` when vz_prior is used — covariance is computed WITHOUT the prior, but the solve used the prior. Reports pessimistic uncertainty. |
| **My Verdict** | ✅ **CONFIRMED** but currently dormant (the seed covariance is discarded). Still a landmine. |
| **Fix** | Use `ATA = A_w.T @ A_w` consistently (matching `irls_refit`'s correct implementation on line 507). |

---

## TIER 3: DESIGN RECOMMENDATIONS (Not bugs, but important improvements)

### REC-1: `rcond=1e-2` is too aggressive
| Source | Claude §9, ChatGPT §1b |
|---|---|
| **Impact** | Can silently discard a physically meaningful weak singular value, collapsing the solution to minimum-norm zero in the weak direction. The posterior sigma gate then sees near-zero variance (not large variance), so it doesn't fire. |
| **Fix** | Change to `rcond=1e-4` or use explicit SVD with manual rank logic. |

### REC-2: FC EKF Vz creates a potential information loop
| Source | Claude §12, ChatGPT §2, Consensus AI §4 |
|---|---|
| **Impact** | If RIO velocity is later fed back to the FC as a velocity aid (via `mavlink_bridge.py`), the FC's own Vz partially determines RIO's Vz, and RIO's Vz feeds back — classic "information incest." Currently mitigated because `--no-mavlink` is used, but will become critical when MAVLink feedback is enabled. |
| **Fix** | Long-term: source vertical aid from a raw barometer, not the FC's fused EKF. Short-term: document the constraint. |

### REC-3: SLAM map bloat
| Source | Teammate §1 |
|---|---|
| **Impact** | GICP against a 200k-point global map in Python causes SLAM to lag to 0.1-0.4 Hz. |
| **Fix** | Use a local sliding window of the last 10 keyframes for GICP registration. Save the full map only on exit for the `.pcd` file. |

### REC-4: `analyze_run.py` doesn't parse new fields
| Source | Teammate §4 |
|---|---|
| **Impact** | Can't see how often gates trigger, condition numbers, or Vz prior usage in post-flight analysis. |
| **Fix** | Parse and display `is_static`, `airborne`, `vz_prior`, `cond`, `n_total` from the JSONL logs. |

---

## CLAIMS I DISAGREE WITH (Auditor errors)

| Claim | Source | My Verdict |
|---|---|---|
| "Vz prior uses GLOBAL_POSITION_INT" | Teammate §2 | ❌ **WRONG.** Code uses `LOCAL_POSITION_NED` (imu_bridge.py L138-142). |
| "The RIO frame is mislabeled as BODY_FRD" | Claude §5, §7 | ⚠️ **PARTIALLY VALID** but low-priority. Currently `--no-mavlink` is used, so nothing actually publishes `MAV_FRAME_BODY_FRD`. This only matters when MAVLink feedback is enabled. |
| "LOS uses lever-arm-shifted coordinates" | Claude §19 | ⚠️ **TECHNICALLY CORRECT but negligible.** At ranges of 10-50m with a lever arm of 0.18m, the angular error is < 0.01°. Not worth fixing now. |
| "Nav should integrate raw IMU accelerations" | Teammate §3 | ❌ **DANGEROUS.** Raw MEMS accelerometer double-integration in Python without a full Kalman filter will diverge in milliseconds. Emergency hover is the correct response. |

---

## Proposed Changes (Priority Order)

### Phase 1: Critical bug fixes (before next flight)

#### [MODIFY] `src/imu_bridge.py`
- Add `_t_vz_ned` and `_t_airborne` separate timestamps
- Change `_airborne` logic: `_airborne = 1 if landed_state != 1 else 0`

#### [MODIFY] `src/doppler_rio.py`
- **BUG-1:** Thread `max_speed_mps` through `DopplerRIO.__init__` to all solver calls
- **BUG-2:** After IRLS `break` on exception, set `cov = None`
- **BUG-3:** Add separate timestamp checks in `IMUListener.get_vz_ned()` and `get_airborne()`
- **BUG-5:** Store yaw with `_v_prev`, rotate to NED before acceleration comparison
- **ARCH-1:** Raise `cond_reject_threshold` from 12 to 50, let posterior sigma gate handle quality

#### [MODIFY] `src/filters.py`
- **BUG-6:** Replace `np.cross(omega_hint, xyz)` with `np.cross(omega_hint, lever_arm)` in `gate_doppler_consistency`

### Phase 2: Architectural hardening (before autonomous flight)

#### [MODIFY] `src/slam_node.py`
- **ARCH-2:** Add `last_v_body_time` tracking; zero out stale translation prior after 0.3s
- **REC-3:** Implement local sliding window for GICP registration

#### [MODIFY] `tools/analyze_run.py`
- **REC-4:** Parse and display new JSONL fields (`cond`, `is_static`, `airborne`, `vz_prior`, `n_total`)

## Verification Plan

### Automated Tests
```bash
python3 src/doppler_rio.py --selftest
python3 tools/analyze_run.py logs/run_20260919_125654.jsonl
```

### Manual Verification
1. Bench test: confirm `airborne` flips to 1 when armed, `vz_ned` updates during vertical movements
2. Flight test: same route as Sept 19, compare RIO distance error against GPS baseline
