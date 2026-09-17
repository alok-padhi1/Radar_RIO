# Autonomous Return-to-Home & True Displacement Plan

This document outlines the architecture for safely guiding a GPS-denied drone back to its launch point using breadcrumb retracing and true 3D displacement tracking, executed over a dedicated background thread.

## 1. True Displacement & Breadcrumbs (`PoseTracker`)
We will add vector displacement tracking and safe-corridor breadcrumb logging.
- Add `start_position_nav` to record the `[0,0,0]` equivalent upon first valid SLAM pose.
- Add `get_displacement_m()` which returns the scalar magnitude of `current_pose - start_position_nav`.
- Add a thread-safe `breadcrumb_trail = []` list.
- During any position update (SLAM or RIO dead-reckoning), if `norm(current_pose - last_breadcrumb) > 2.0` meters, append the current pose.

## 2. RTH Planner Thread (`RTHPlanner`)
A background thread to offload compute from the 20Hz MAVLink control loop.
- Inherits `threading.Thread` and waits on a threading `Event`.
- When triggered, it safely acquires a copy of `breadcrumb_trail`.
- **Smoothing Logic:** Reverses the list and applies a Simple Moving Average (window size = 3) to the 3D points. This smooths out sharp corners or jagged dead-reckoning artifacts while keeping the path securely within the obstacle-free corridor.
- Appends the original `start_position_nav` as the final waypoint.
- Enqueues the smoothed path into a thread-safe `queue.Queue`.

## 3. State Machine Integration (`NavNode`)
We will intercept the existing, dangerous `FAILSAFE_RTL` (which hands control to a GPS-dependent autopilot) and replace it with our closed-loop autonomous RTH.
- **`NavState.FAILSAFE_RTL`**: Instead of calling `ap.request_rtl()`, this state will trigger the `RTHPlanner`. Once the planner queue has a generated path, the state transitions to `NavState.EXECUTING_RTH`.
- **`NavState.EXECUTING_RTH`**: 
  - Pops waypoints sequentially and navigates toward them using the existing `velocity_toward` controller.
  - Upon reaching the final `[0,0,0]` waypoint, transitions to `NavState.HOLD` to safely hover over the launch point.
- **Reactive Braking**: Inside `EXECUTING_RTH`, the drone continually checks `fwd_obstacle_range_m` (dynamic obstacles). If an obstacle appears within `cfg.obstacle_hard_stop_m` (5.0m), it will override the RTH velocity to `[0,0,0]` and hover safely until the obstacle clears.

## User Review Required
> [!IMPORTANT]
> - **Smoothing Window**: I have proposed a moving average window of 3 for smoothing the breadcrumbs. If the environment is highly constrained (tight hallways), this might cut corners too aggressively. Let me know if you prefer raw, un-smoothed breadcrumbs.
> - **Terminal State**: Currently, when RTH finishes, I plan to enter `NavState.HOLD`. Would you prefer it to automatically command `MAV_CMD_NAV_LAND` instead?

Please approve this plan, or let me know if you'd like to adjust the smoothing or terminal state behavior before I begin execution.
