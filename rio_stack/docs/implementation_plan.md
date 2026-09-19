# Root Cause Analysis & Fix Plan for Flight Failures (170904 & 171216)

I have deeply analyzed the three flight logs you provided. The stack ran, but the drone flew wildly (a 26m vertical jump in 170904) and SLAM completely collapsed, resulting in the sparse point cloud in your screenshot. 

Here is exactly what happened, and how we will fix it.

## 🔴 The Three Cascading Failures

1. **The Double-Rotation Flyaway (`BUG-7`)**
   - **What happened:** In `doppler_rio.py`, we level the points using the IMU to correctly apply the Vz prior. Because the points are leveled, the resulting solved velocity (`v_body`) is *also* leveled (i.e., its pitch and roll are 0). However, `nav_node.py` assumes `v_body` is strictly in the raw FRD body frame and unconditionally rotates it using the drone's current pitch and roll. 
   - **The impact:** If the drone pitches forward 15°, `v_body` is already horizontal. `nav_node.py` pitches it *another* 15°, injecting massive phantom vertical velocities. The drone thinks it is plummeting and violently flies up to compensate (explaining the 26m vertical climb).

2. **The Condition Number Over-filtering (`BUG-8`)**
   - **What happened:** In `supervisor.py`, `--cond-reject-threshold` is set to `12.0`. At 30m AGL, the radar footprint hits the ground as a highly planar patch. A flat plane of points has a naturally high condition number (often 10–25).
   - **The impact:** `doppler_rio.py` rejected almost 30-40% of the flight frames simply because they were coplanar, returning `valid=False` (a RIO dropout) even though the data was perfect.

3. **The SLAM Collapse on RIO Dropout (`BUG-9`)**
   - **What happened:** When RIO dropped out (due to `BUG-8`), `slam_node.py` defaulted the velocity hint (`v_hint`) to `[0, 0, 0]`.
   - **The impact:** The drone was flying at 2–10 m/s. `filters.py` checked every point: "Does this point have a Doppler velocity near 0 m/s?" The real ground was moving at 10 m/s, so **every single ground point was dropped**. The only points that passed were noise, self-returns, and multipath ghosts (which move *with* the drone at 0 m/s). SLAM then registered these noise points, causing it to hallucinate that the drone was completely stationary (explaining the 29m SLAM distance vs 193m GPS distance).

---

## 🟢 Proposed Changes

### 1. Fix `doppler_rio.py` Double Rotation
Before packing the UDP packet in `doppler_rio.py`, we must rotate `v_body` *back* into the raw unleveled body frame if `imu_level_points` was used. This preserves the API contract so `nav_node.py` can integrate it safely.
- **MODIFY** `doppler_rio.py`: Apply `R_level.T @ v_body` before sending.

### 2. Relax the Condition Gate
The monograph recommends a condition threshold of 30.0 for the planar degeneracy gate, not 12.0. 
- **MODIFY** `supervisor.py`: Change `--cond-reject-threshold` default from 12.0 to 30.0.

### 3. Implement Coasting in `slam_node.py`
If RIO drops out for a frame, `slam_node.py` must *not* assume the drone slammed on the brakes to 0 m/s. It must coast on the last known velocity for up to 1.0 seconds to keep the Doppler gate centered on the ground.
- **MODIFY** `slam_node.py`: Add `self.last_v_rio` and `self.last_v_rio_t`. Use it when `rio_valid == False`.

> [!IMPORTANT]
> **User Review Required**
> Please review this plan. The double-rotation bug is the most critical issue, and fixing it will restore stable flight control. If you approve, I will execute these three fixes immediately.
