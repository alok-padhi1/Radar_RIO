#!/usr/bin/env python3
"""
analyze_run.py
Post-run drift and performance analyzer for RIO Radar.

Reads a JSONL log file produced by gps_logger.py and computes:
  - Absolute Position Error (SLAM vs GPS) decomposed into XY and Z
  - RIO integrated distance vs GPS path length
  - Velocity comparison (RIO vs GPS-derived)
  - Drift rate (linear fit of position error over time)
  - Pipeline health statistics (inlier counts, SLAM success rate)

Prints a human-readable report with PASS/FAIL gates aligned to the
Field Runbook's stage progression.

Usage:
    python3 analyze_run.py logs/run_20260911_150100.jsonl
    python3 analyze_run.py logs/run_20260911_150100.jsonl --save-npz results.npz

Requires: pip install numpy
"""

import argparse
import json
import math
import os
import sys

import numpy as np


# ─── Data Loading ────────────────────────────────────────────────────────────

def load_log(path: str):
    """Parse a JSONL log file into separate GPS, RIO, SLAM, IMU, and Alt entries."""
    gps, rio, slam, imu, alt = [], [], [], [], []
    meta = {}

    with open(path) as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                entry = json.loads(line)
            except json.JSONDecodeError:
                print(f"  [warn] Skipping malformed line {line_num}")
                continue

            etype = entry.get('type')
            if etype == 'gps':
                gps.append(entry)
            elif etype == 'rio':
                rio.append(entry)
            elif etype == 'slam':
                slam.append(entry)
            elif etype == 'imu':
                imu.append(entry)
            elif etype == 'altimeter':
                alt.append(entry)
            elif etype == 'meta':
                meta = entry

    return gps, rio, slam, imu, alt, meta


# ─── Analysis Functions ─────────────────────────────────────────────────────

def gps_path_length(gps: list[dict]) -> float:
    """Total GPS path length (sum of inter-fix distances in ENU).
    Downsamples to 1Hz to avoid the 'coastline paradox' from high-frequency EKF jitter.
    """
    if not gps:
        return 0.0
    total = 0.0
    last_idx = 0
    for i in range(1, len(gps)):
        if gps[i]['t_mono'] - gps[last_idx]['t_mono'] >= 1.0:
            dx = gps[i]['enu'][0] - gps[last_idx]['enu'][0]
            dy = gps[i]['enu'][1] - gps[last_idx]['enu'][1]
            dz = gps[i]['enu'][2] - gps[last_idx]['enu'][2]
            total += math.sqrt(dx*dx + dy*dy + dz*dz)
            last_idx = i
    return total


def gps_net_displacement(gps: list[dict]) -> tuple[float, float, float]:
    """Net displacement (XY, Z, 3D) from first to last GPS fix."""
    if len(gps) < 2:
        return 0.0, 0.0, 0.0
    dx = gps[-1]['enu'][0] - gps[0]['enu'][0]
    dy = gps[-1]['enu'][1] - gps[0]['enu'][1]
    dz = gps[-1]['enu'][2] - gps[0]['enu'][2]
    xy = math.sqrt(dx*dx + dy*dy)
    return xy, abs(dz), math.sqrt(dx*dx + dy*dy + dz*dz)


def rio_integrated_distance(rio: list[dict]) -> tuple[float, np.ndarray]:
    """Integrate RIO velocity to get total distance and final displacement."""
    total_dist = 0.0
    pos = np.zeros(3)
    for i in range(1, len(rio)):
        dt = rio[i]['t_mono'] - rio[i-1]['t_mono']
        if dt <= 0 or dt > 0.5:
            continue
        v = np.array([rio[i]['vx'], rio[i]['vy'], rio[i]['vz']])
        pos += v * dt
        total_dist += float(np.linalg.norm(v)) * dt
    return total_dist, pos


def gps_derived_velocity(gps: list[dict]) -> list[dict]:
    """Compute GPS velocity by finite-differencing consecutive ENU positions.

    At 1 Hz this is noisy but provides a ground-truth velocity baseline.
    We compute speed (scalar) and per-axis velocities.
    """
    vels = []
    for i in range(1, len(gps)):
        dt = gps[i]['t_mono'] - gps[i-1]['t_mono']
        if dt < 0.1:  # skip duplicates
            continue
        dx = gps[i]['enu'][0] - gps[i-1]['enu'][0]
        dy = gps[i]['enu'][1] - gps[i-1]['enu'][1]
        dz = gps[i]['enu'][2] - gps[i-1]['enu'][2]
        speed = math.sqrt(dx*dx + dy*dy + dz*dz) / dt
        vels.append({
            't_mono': (gps[i]['t_mono'] + gps[i-1]['t_mono']) / 2.0,
            'speed':  speed,
            'vx':     dx / dt,
            'vy':     dy / dt,
            'vz':     dz / dt,
        })
    return vels


def time_align_nearest(ref_entries: list[dict], target_entries: list[dict],
                       max_dt: float = 1.0) -> list[tuple[dict, dict]]:
    """For each ref entry, find the nearest target entry within max_dt.

    Returns list of (ref, target) pairs.
    """
    if not ref_entries or not target_entries:
        return []

    target_times = np.array([e['t_mono'] for e in target_entries])
    pairs = []
    for ref in ref_entries:
        t = ref['t_mono']
        idx = int(np.argmin(np.abs(target_times - t)))
        if abs(target_times[idx] - t) <= max_dt:
            pairs.append((ref, target_entries[idx]))
    return pairs


def compute_position_errors(slam: list[dict], gps: list[dict], alt: list[dict] = None) -> dict:
    """Compute SLAM-vs-GPS position errors over time.

    For each SLAM pose, finds the nearest GPS fix and computes:
    - 3D absolute position error
    - XY (horizontal) error
    - Z (vertical) error (using Altimeter if available, fallback to GPS)

    Also fits a linear trend to estimate drift rate.
    """
    pairs = time_align_nearest(slam, gps, max_dt=1.5)
    if not pairs:
        return {}

    # Extract all XY points
    slam_pts = np.array([p[0]['pos'][:2] for p in pairs])
    gps_pts = np.array([p[1]['enu'][:2] for p in pairs])
    
    # Compute optimal 2D rotation (SVD/Kabsch) to align SLAM heading to GPS heading
    H = slam_pts.T @ gps_pts
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
        
    # Rotate all SLAM XY points
    slam_pts_aligned = slam_pts @ R.T

    errors_3d = []
    errors_xy = []
    errors_z = []
    times = []

    pairs_alt = None
    alt_lookup = {}
    if alt:
        pairs_alt = time_align_nearest(slam, alt, max_dt=0.5)
        alt_lookup = {p[0]['t_mono']: p[1]['range_m'] for p in pairs_alt}

    z_offset_slam = pairs[0][0]['pos'][2]
    z_offset_gps = pairs[0][1]['enu'][2]
    z_offset_alt = alt[0]['range_m'] if alt else 0.0

    for i, (slam_e, gps_e) in enumerate(pairs):
        sp_xy = slam_pts_aligned[i]
        gp_xy = gps_pts[i]
        
        # SLAM Z is in FRD (positive Down). We negate it so relative altitude is Up.
        rel_sp_z = -(slam_e['pos'][2] - z_offset_slam)

        t = slam_e['t_mono']
        if alt_lookup and t in alt_lookup:
            # Altimeter range is directly altitude (Up)
            rel_gp_z = alt_lookup[t] - z_offset_alt
        else:
            # GPS ENU Z is Up
            rel_gp_z = gps_e['enu'][2] - z_offset_gps

        err_xy = float(np.linalg.norm(sp_xy - gp_xy))
        err_z  = abs(rel_sp_z - rel_gp_z)
        err_3d = math.sqrt(err_xy**2 + err_z**2)

        errors_3d.append(err_3d)
        errors_xy.append(err_xy)
        errors_z.append(err_z)
        times.append(slam_e['t_mono'])

    result = {
        'n_pairs':      len(errors_3d),
        'ape_3d_mean':  float(np.mean(errors_3d)),
        'ape_3d_max':   float(np.max(errors_3d)),
        'ape_3d_final': errors_3d[-1],
        'ape_xy_mean':  float(np.mean(errors_xy)),
        'ape_xy_max':   float(np.max(errors_xy)),
        'ape_xy_final': errors_xy[-1],
        'ape_z_mean':   float(np.mean(errors_z)),
        'ape_z_max':    float(np.max(errors_z)),
        'ape_z_final':  errors_z[-1],
    }

    # Drift rate: linear fit of 3D APE vs elapsed time
    if len(errors_3d) >= 3:
        t_arr = np.array(times) - times[0]
        coeffs = np.polyfit(t_arr, errors_3d, 1)
        result['drift_rate_mps'] = float(abs(coeffs[0]))

    # Error time series for optional export
    result['_errors_3d'] = errors_3d
    result['_errors_xy'] = errors_xy
    result['_errors_z']  = errors_z
    result['_times']     = times

    return result


def compute_velocity_errors(rio: list[dict], gps: list[dict]) -> dict:
    """Compare RIO speed against GPS-derived speed.

    We low-pass both to ~1 Hz for fair comparison (GPS is only 1 Hz).
    """
    gps_vels = gps_derived_velocity(gps)
    if not gps_vels or not rio:
        return {}

    # Bin RIO velocities into 1-second windows and average
    rio_times = np.array([r['t_mono'] for r in rio])
    rio_speeds = np.array([math.sqrt(r['vx']**2 + r['vy']**2 + r['vz']**2)
                           for r in rio])

    # For each GPS velocity, find average RIO speed in a ±0.5s window
    speed_errors = []
    for gv in gps_vels:
        t = gv['t_mono']
        mask = (rio_times >= t - 0.5) & (rio_times <= t + 0.5)
        if mask.sum() < 2:
            continue
        rio_avg_speed = float(np.mean(rio_speeds[mask]))
        gps_speed = gv['speed']
        speed_errors.append(abs(rio_avg_speed - gps_speed))

    if not speed_errors:
        return {}

    return {
        'vel_err_mean': float(np.mean(speed_errors)),
        'vel_err_max':  float(np.max(speed_errors)),
        'vel_err_p90':  float(np.percentile(speed_errors, 90)),
        'n_vel_pairs':  len(speed_errors),
    }


# ─── Report Printer ─────────────────────────────────────────────────────────

def print_report(gps, rio, slam, imu, alt, meta, log_path) -> bool:
    """Compute all metrics and print human-readable report.

    Returns True if all gates pass, False otherwise.
    """
    BORDER = "=" * 72

    print(f"\n{BORDER}")
    print(f"  RIO RADAR — POST-RUN ANALYSIS REPORT")
    print(f"  Log: {os.path.basename(log_path)}")
    print(f"{BORDER}")

    # ── Session Info ──
    duration = 0.0
    all_t = ([e['t_mono'] for e in gps] +
             [e['t_mono'] for e in rio] +
             [e['t_mono'] for e in slam])
    if all_t:
        duration = max(all_t) - min(all_t)

    print(f"\n📊 SESSION INFO")
    print(f"  Duration:        {duration:.1f} s")
    print(f"  GPS Fixes:       {len(gps)}")
    print(f"  RIO Frames:      {len(rio)}")
    print(f"  SLAM Poses:      {len(slam)}")
    if imu:
        print(f"  IMU Frames:      {len(imu)}")
    if alt:
        print(f"  Alt Frames:      {len(alt)}")
    if meta.get('ref_height_m'):
        print(f"  Ref Height:      {meta['ref_height_m']:.2f} m (meta)")
    if rio:
        rio_hz = len(rio) / max(duration, 1)
        print(f"  RIO Rate:        {rio_hz:.1f} Hz")
    if slam:
        slam_hz = len(slam) / max(duration, 1)
        print(f"  SLAM Rate:       {slam_hz:.1f} Hz")

    # ── Distance Comparison ──
    print(f"\n📏 DISTANCE COMPARISON")
    gps_path = gps_path_length(gps)
    gps_disp_xy, gps_disp_z, gps_disp_3d = gps_net_displacement(gps)
    rio_dist, rio_pos = rio_integrated_distance(rio)
    rio_disp = float(np.linalg.norm(rio_pos))

    print(f"  GPS path length:         {gps_path:.2f} m")
    print(f"  GPS net displacement:    {gps_disp_3d:.2f} m (XY={gps_disp_xy:.2f}, Z={gps_disp_z:.2f})")
    print(f"  RIO integrated distance: {rio_dist:.2f} m")
    print(f"  RIO net displacement:    {rio_disp:.2f} m")

    rio_dist_err_pct = None
    if gps_path > 1.0:
        rio_dist_err_pct = abs(rio_dist - gps_path) / gps_path * 100
        status = "✅ PASS" if rio_dist_err_pct <= 5.0 else "❌ FAIL"
        print(f"  Distance error:          {rio_dist_err_pct:.1f}% {status} (gate: ≤5%)")

    if slam:
        slam_disp = float(np.linalg.norm(slam[-1]['pos']))
        print(f"  SLAM net displacement:   {slam_disp:.2f} m")
        if gps_disp_3d > 1.0:
            slam_disp_err = abs(slam_disp - gps_disp_3d) / gps_disp_3d * 100
            print(f"  SLAM displacement error: {slam_disp_err:.1f}%")

    # ── Position Error (SLAM vs GPS/Alt) ──
    pos_errs = compute_position_errors(slam, gps, alt)
    if pos_errs:
        print(f"\n📐 POSITION ERROR (SLAM vs GPS)  [{pos_errs['n_pairs']} time-aligned pairs]")
        print(f"  {'Metric':<22s}  {'XY (horiz)':<14s}  {'Z (vert)':<14s}  {'3D (total)':<14s}")
        print(f"  {'─'*22}  {'─'*14}  {'─'*14}  {'─'*14}")
        print(f"  {'Mean error':<22s}  {pos_errs['ape_xy_mean']:>8.3f} m     "
              f"{pos_errs['ape_z_mean']:>8.3f} m     {pos_errs['ape_3d_mean']:>8.3f} m")
        print(f"  {'Max error':<22s}  {pos_errs['ape_xy_max']:>8.3f} m     "
              f"{pos_errs['ape_z_max']:>8.3f} m     {pos_errs['ape_3d_max']:>8.3f} m")
        print(f"  {'Final error':<22s}  {pos_errs['ape_xy_final']:>8.3f} m     "
              f"{pos_errs['ape_z_final']:>8.3f} m     {pos_errs['ape_3d_final']:>8.3f} m")

        # Diagnose: which axis dominates?
        if pos_errs['ape_z_mean'] > pos_errs['ape_xy_mean'] * 1.5:
            print(f"\n  ⚠️  Z-error dominates XY by {pos_errs['ape_z_mean']/max(pos_errs['ape_xy_mean'],0.01):.1f}×")
            print(f"      → Confirms audit risk R-4: gravity alignment / IMU needed")
        elif pos_errs['ape_xy_mean'] > pos_errs['ape_z_mean'] * 1.5:
            print(f"\n  ℹ️  XY-error dominates Z by {pos_errs['ape_xy_mean']/max(pos_errs['ape_z_mean'],0.01):.1f}×")
            print(f"      → GICP registration quality is the primary accuracy limiter")
        else:
            print(f"\n  ℹ️  XY and Z errors are comparable — no single dominant axis")

    # ── Drift Rate ──
    if 'drift_rate_mps' in pos_errs:
        dr = pos_errs['drift_rate_mps']
        status = "✅ PASS" if dr < 0.05 else "⚠️  HIGH"
        print(f"\n📈 DRIFT RATE")
        print(f"  Position drift rate:     {dr:.4f} m/s {status} (gate: <0.05 m/s)")
        print(f"  Projected drift at 60s:  {dr * 60:.2f} m")
        print(f"  Projected drift at 300s: {dr * 300:.2f} m")

    # ── Velocity Comparison ──
    vel_errs = compute_velocity_errors(rio, gps)
    if vel_errs:
        print(f"\n🚗 VELOCITY COMPARISON (RIO vs GPS)  [{vel_errs['n_vel_pairs']} pairs]")
        print(f"  Mean speed error:   {vel_errs['vel_err_mean']:.3f} m/s")
        print(f"  Max speed error:    {vel_errs['vel_err_max']:.3f} m/s")
        print(f"  90th %ile error:    {vel_errs['vel_err_p90']:.3f} m/s")

    # ── RIO Statistics ──
    if rio:
        speeds = [math.sqrt(r['vx']**2 + r['vy']**2 + r['vz']**2) for r in rio]
        inliers = [r['inliers'] for r in rio]
        print(f"\n🔬 RIO VELOCITY STATISTICS")
        print(f"  Mean speed:     {np.mean(speeds):.3f} m/s")
        print(f"  Max speed:      {max(speeds):.3f} m/s")
        print(f"  Mean inliers:   {np.mean(inliers):.1f}")
        print(f"  Min inliers:    {min(inliers)}")

    # ── SLAM Statistics ──
    if slam:
        map_sizes = [s['n_map'] for s in slam]
        fwd_ranges = [s['fwd_range'] for s in slam if s['fwd_range'] > 0]
        print(f"\n🗺️  SLAM STATISTICS")
        print(f"  Total keyframes: {len(slam)}")
        print(f"  Final map size:  {slam[-1]['n_map']} points")
        if fwd_ranges:
            print(f"  Obstacle range:  min={min(fwd_ranges):.1f}m, "
                  f"mean={np.mean(fwd_ranges):.1f}m, max={max(fwd_ranges):.1f}m")

    # ── IMU Statistics ──
    if imu:
        omegas = [math.sqrt(i['wx']**2 + i['wy']**2 + i['wz']**2) for i in imu]
        print(f"\n🔄 IMU STATISTICS")
        print(f"  Total IMU pkts: {len(imu)}")
        print(f"  Mean rot speed: {np.mean(omegas):.3f} rad/s")
        print(f"  Peak rot speed: {np.max(omegas):.3f} rad/s")

    # ── Verdict ──
    print(f"\n{BORDER}")
    print(f"  VERDICT")
    print(f"{BORDER}")

    gates = []
    all_pass = True

    if rio_dist_err_pct is not None:
        ok = rio_dist_err_pct <= 5.0
        gates.append(("RIO distance ≤5%", ok, f"{rio_dist_err_pct:.1f}%"))

    if 'ape_xy_final' in pos_errs:
        ok = pos_errs['ape_xy_final'] < 5.0
        gates.append(("SLAM XY error <5m", ok, f"{pos_errs['ape_xy_final']:.2f} m"))

    if 'ape_z_final' in pos_errs:
        ok = pos_errs['ape_z_final'] < 2.0
        gates.append(("SLAM Z error <2m", ok, f"{pos_errs['ape_z_final']:.2f} m"))

    if 'drift_rate_mps' in pos_errs:
        ok = pos_errs['drift_rate_mps'] < 0.05
        gates.append(("Drift rate <0.05 m/s", ok,
                      f"{pos_errs['drift_rate_mps']:.4f} m/s"))

    for name, ok, val in gates:
        icon = "✅" if ok else "❌"
        print(f"  {icon} {name}: {val}")
        if not ok:
            all_pass = False

    if not gates:
        print("  ⚠️  No gates evaluated (insufficient data for comparison)")
        print("     Check: is GPS getting a fix? Is SLAM producing poses?")
        all_pass = False

    overall = "✅ ALL GATES PASSED" if all_pass else "❌ SOME GATES FAILED"
    print(f"\n  {overall}")
    print(f"{BORDER}\n")

    return all_pass


# ─── NPZ Export ──────────────────────────────────────────────────────────────

def save_npz(path: str, gps, rio, slam, imu, alt):
    """Export raw arrays for external plotting."""
    arrays = {}

    if gps:
        arrays['gps_t'] = np.array([g['t_mono'] for g in gps])
        arrays['gps_enu'] = np.array([g['enu'] for g in gps])
        arrays['gps_sats'] = np.array([g['sats'] for g in gps])

    if rio:
        arrays['rio_t'] = np.array([r['t_mono'] for r in rio])
        arrays['rio_v'] = np.array([[r['vx'], r['vy'], r['vz']] for r in rio])
        arrays['rio_inliers'] = np.array([r['inliers'] for r in rio])

    if slam:
        arrays['slam_t'] = np.array([s['t_mono'] for s in slam])
        arrays['slam_pos'] = np.array([s['pos'] for s in slam])
        arrays['slam_n_map'] = np.array([s['n_map'] for s in slam])

    if imu:
        arrays['imu_t'] = np.array([i['t_mono'] for i in imu])
        arrays['imu_rpy'] = np.array([[i['roll'], i['pitch'], i['yaw']] for i in imu])
        arrays['imu_omega'] = np.array([[i['wx'], i['wy'], i['wz']] for i in imu])

    if alt:
        arrays['alt_t'] = np.array([a['t_mono'] for a in alt])
        arrays['alt_range'] = np.array([a['range_m'] for a in alt])

    np.savez_compressed(path, **arrays)
    print(f"  Saved raw arrays to {path}")


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description="Analyze a GPS+Radar JSONL log from gps_logger.py")
    p.add_argument('log_file',
                    help="Path to the JSONL log file from gps_logger.py")
    p.add_argument('--save-npz', default=None,
                    help="Save raw time-series arrays as .npz for plotting")
    args = p.parse_args()

    if not os.path.exists(args.log_file):
        print(f"ERROR: File not found: {args.log_file}")
        sys.exit(1)

    gps, rio, slam, imu, alt, meta = load_log(args.log_file)

    if not gps and not rio and not slam and not imu and not alt:
        print("ERROR: Log file is empty or contains no recognized entries.")
        sys.exit(1)

    if not gps:
        print("WARNING: No GPS data in log. Position comparison will be skipped.")
        print("         Was the GPS module connected and getting satellite fixes?")

    passed = print_report(gps, rio, slam, imu, alt, meta, args.log_file)

    if args.save_npz:
        save_npz(args.save_npz, gps, rio, slam, imu, alt)

    sys.exit(0 if passed else 1)


if __name__ == '__main__':
    main()
