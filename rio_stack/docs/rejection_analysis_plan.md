# Audit Implementation Plan: RIO Frame Rejection & SLAM Starvation

## 1. The Root Cause of `max_sigma=10.00 > 0.6`
Your teammate ran the **unpatched** codebase in `/home/aman/Work_Repository/...` and encountered a catastrophic failure where 100% of RIO frames were rejected.

**The Bug:**
In the original `doppler_rio.py` (around line 1179), the following code existed:
```python
cxx, cyy, czz = cov[0,0], cov[1,1], cov[2,2]
if result.get('vz_prior_used', False):
    czz = 100.0  # Strip the prior from published covariance

max_sigma = math.sqrt(max(cxx, cyy, czz))
if max_sigma > rio.max_sigma_v_mps:
    print(f"t={t_frame:.3f}  RIO frame rejected (max_sigma={max_sigma:.2f} > {rio.max_sigma_v_mps})")
    continue
```
Whenever the vertical velocity prior was active, the code artificially forced the vertical variance `czz` to `100.0`. It then immediately checked `math.sqrt(max(cxx, cyy, czz))`, which evaluated to `math.sqrt(100.0) = 10.00`. 
Because `10.00 > 0.60`, **every single frame was rejected by its own safety gate.**

**The Impact:**
The RIO node processed the frame successfully (which is why it printed `v_body=... inliers=24/24`), but then suicidally rejected its own output before sending it over UDP. As a result, the downstream SLAM node received 0 velocity updates.

## 2. The Root Cause of SLAM `too_sparse_after_filtering`
Because RIO was rejecting 100% of frames, SLAM received absolutely no velocity priors. 
When SLAM receives no prior, its `deskew` algorithm completely fails to motion-compensate the point cloud. Furthermore, without a velocity prior, the Doppler filter rejects valid ground points because it cannot match their radial velocity against the drone's true motion. The point cloud is stripped bare, resulting in `keyframe REJECTED: too_sparse_after_filtering`.

## 3. The Fix
This defect is **already solved** in the `ALL_FIXES.patch` that I have applied to your local `/home/alok/radar/rio_stack/` directory!

Claude's patch moved the `czz = 100.0` logic to occur **after** the `max_sigma` check. The safety gate now correctly evaluates the true physical covariance of the solve, and `czz = 100.0` is only applied when packing the UDP packet to prevent the autopilot from trusting the vertical prior as a true radar measurement.

```python
# PATCHED LOGIC:
cxx, cyy, czz = cov[0,0], cov[1,1], cov[2,2]
max_sigma = math.sqrt(max(cxx, cyy, czz))
if max_sigma > rio.max_sigma_v_mps:
    print("Rejected!")
    continue

# Applied AFTER the gate!
if result.get('vz_prior_used', False):
    czz = 100.0  
```

**Next Steps:**
Tell your teammate to pull the changes from `/home/alok/radar/rio_stack/` or apply `ALL_FIXES.patch` to their local repository. The RIO node will immediately start publishing packets, and SLAM will stop starving.
