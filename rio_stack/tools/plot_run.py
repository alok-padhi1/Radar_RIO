#!/usr/bin/env python3
"""
plot_run.py  ─  Absolute Visual Analyzer for RIO Radar Flight Logs
===================================================================

Generates a MATLAB-quality multi-panel analysis figure from a JSONL log
produced by gps_logger.py.  Every sensor stream in the log is used:

    GPS   → ground-truth trajectory, path length, derived velocity
    RIO   → integrated trajectory, speed, inliers, condition #, flags
    SLAM  → 6-DOF poses, map size, forward obstacle range
    IMU   → attitude (roll/pitch/yaw), angular rates
    ALT   → AGL range, climb/descent rate

Panels
------
  Row 0  : [A] 3-D Trajectory  (GPS blue, RIO-integrated red, SLAM green)
  Row 1  : [B] Speed comparison + [C] Per-axis velocity (Vx/Vy/Vz)
  Row 2  : [D] SLAM vs GPS XY position error over time
            [E] SLAM vs GPS Z position error over time
  Row 3  : [F] RIO inliers + total points  [G] RIO condition number
  Row 4  : [H] IMU pitch / roll  [I] IMU yaw + angular-rate magnitude
  Row 5  : [J] Altimeter AGL + computed vz  [K] SLAM map size + fwd range
  Row 6  : [L] Summary text panel (all key metrics + PASS/FAIL gates)

Usage
-----
    python3 tools/plot_run.py  logs/run_20260920_151941.jsonl
    python3 tools/plot_run.py  logs/run_20260920_151941.jsonl  --out my_report.png

Requires:  pip install numpy matplotlib
"""

import argparse
import json
import math
import os
import sys
from typing import Optional

import numpy as np
import matplotlib
matplotlib.use("Agg")          # headless-safe; swap to "TkAgg" for interactive
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.lines import Line2D
from analyze_run import print_report


# ═══════════════════════════════════════════════════════════════════════════════
#  Styling  (Light MATLAB-inspired)
# ═══════════════════════════════════════════════════════════════════════════════

STYLE = {
    "figure.facecolor":      "#ffffff",
    "axes.facecolor":        "#ffffff",
    "axes.edgecolor":        "#cccccc",
    "axes.labelcolor":       "#333333",
    "axes.titlecolor":       "#000000",
    "axes.titlesize":        9,
    "axes.labelsize":        8,
    "axes.grid":             True,
    "grid.color":            "#eeeeee",
    "grid.linewidth":        0.6,
    "xtick.color":           "#666666",
    "ytick.color":           "#666666",
    "xtick.labelsize":       7,
    "ytick.labelsize":       7,
    "legend.fontsize":       7,
    "legend.facecolor":      "#ffffff",
    "legend.edgecolor":      "#cccccc",
    "legend.labelcolor":     "#333333",
    "lines.linewidth":       1.4,
    "text.color":            "#000000",
}

C_GPS   = "blue"
C_RIO   = "red"
C_SLAM  = "green"
C_ALT   = "purple"
C_IMU   = "orange"
C_ERR   = "red"
C_GOOD  = "green"


# ═══════════════════════════════════════════════════════════════════════════════
#  Data Loading
# ═══════════════════════════════════════════════════════════════════════════════

_DEPRECATION = """
+---------------------------------------------------------------------------+
|  plot_run.py is SUPERSEDED by tools/analyze_run_v2.py.                    |
|                                                                            |
|  Known defects retained here for figure compatibility:                     |
|   * rio_integrate() does not rotate body velocity into the world frame,    |
|     so its trajectory and displacement are meaningless under yaw.          |
|   * gps_path_length_1hz() has no deadband, while analyze_run.py has one,   |
|     so the two tools report different "GPS path length" for the same log.  |
|   * align_slam_to_gps() reports Kabsch SHAPE error, while analyze_run.py   |
|     reports RAW absolute error, and neither says which it is.              |
|  Do not gate a flight decision on this file.                               |
+---------------------------------------------------------------------------+
"""


def load_log(path: str):
    gps, rio, slam, imu, alt, meta = [], [], [], [], [], {}
    with open(path) as f:
        for ln, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                e = json.loads(line)
            except json.JSONDecodeError:
                print(f"  [warn] malformed JSON at line {ln}", file=sys.stderr)
                continue
            t = e.get("type")
            if   t == "gps":       gps.append(e)
            elif t == "rio":       rio.append(e)
            elif t == "slam":      slam.append(e)
            elif t == "imu":       imu.append(e)
            elif t == "altimeter": alt.append(e)
            elif t == "meta":      meta = e
    return gps, rio, slam, imu, alt, meta


# ═══════════════════════════════════════════════════════════════════════════════
#  Core Computations
# ═══════════════════════════════════════════════════════════════════════════════

def _t0(gps, rio, slam):
    """Common time origin = earliest timestamp across all streams."""
    times = ([e["t_mono"] for e in gps] +
             [e["t_mono"] for e in rio] +
             [e["t_mono"] for e in slam])
    return min(times) if times else 0.0


def gps_path_length_1hz(gps):
    """Downsample to 1 Hz to avoid coastline-paradox from EKF jitter."""
    if not gps:
        return 0.0
    total = 0.0
    last_idx = 0
    for i in range(1, len(gps)):
        if gps[i]["t_mono"] - gps[last_idx]["t_mono"] >= 1.0:
            d = np.linalg.norm(np.array(gps[i]["enu"]) - np.array(gps[last_idx]["enu"]))
            total += d
            last_idx = i
            
    if last_idx < len(gps) - 1:
        d = np.linalg.norm(np.array(gps[-1]["enu"]) - np.array(gps[last_idx]["enu"]))
        total += d
    return total


def rio_integrate(rio):
    """Integrate RIO velocity -> position trajectory + scalar distances.

    WARNING (finding A-4): this integrates BODY-frame velocity with no rotation
    into the world frame. The result is not a trajectory as soon as the aircraft
    yaws -- on the 2026-09-20 logs it produces 50 m of 'displacement' for a
    flight that returned to within 0.4 m of its start. It disagrees with
    tools/analyze_run.py, which does rotate. Use tools/analyze_run_v2.py.
    Kept only so existing figures still render; see the banner drawn on the
    trajectory panel.

    FIX: the empty-input branch returned THREE values while the normal path
    returns TWO, so `positions, dists = rio_integrate([])` raised ValueError on
    any log with no RIO entries.
    """
    if not rio:
        return np.zeros((0, 3)), np.array([])
    pos = np.zeros(3)
    positions = [pos.copy()]
    dists = [0.0]
    for i in range(1, len(rio)):
        dt = rio[i]["t_mono"] - rio[i - 1]["t_mono"]
        if dt <= 0:
            positions.append(positions[-1].copy())
            dists.append(dists[-1])
            continue
        v_prev = np.array([rio[i - 1]["vx"], rio[i - 1]["vy"], rio[i - 1]["vz"]])
        v_curr = np.array([rio[i]["vx"], rio[i]["vy"], rio[i]["vz"]])
        v_avg = (v_prev + v_curr) / 2.0
        pos = pos + v_avg * dt
        positions.append(pos.copy())
        dists.append(dists[-1] + float(np.linalg.norm(v_avg)) * dt)
    return np.array(positions), np.array(dists)


def gps_derived_velocity_1hz(gps):
    """1-Hz GPS-derived velocity (finite-difference)."""
    vels, last = [], 0
    for i in range(1, len(gps)):
        dt = gps[i]["t_mono"] - gps[last]["t_mono"]
        if dt < 0.5:
            continue
        denu = np.array(gps[i]["enu"]) - np.array(gps[last]["enu"])
        v = denu / dt
        vels.append({
            "t": (gps[i]["t_mono"] + gps[last]["t_mono"]) / 2.0,
            "v": v,
            "speed": float(np.linalg.norm(v)),
        })
        last = i
    return vels


def interpolate_gps_vec(gps, target_t):
    gps_t = np.array([e['t_mono'] for e in gps])
    if target_t <= gps_t[0]: return np.array(gps[0]['enu'])
    if target_t >= gps_t[-1]: return np.array(gps[-1]['enu'])
    idx = np.searchsorted(gps_t, target_t)
    t0, t1 = gps_t[idx-1], gps_t[idx]
    p0 = np.array(gps[idx-1]['enu'])
    p1 = np.array(gps[idx]['enu'])
    frac = (target_t - t0) / (t1 - t0) if t1 > t0 else 0.0
    return p0 + (p1 - p0) * frac

def align_slam_to_gps(slam, gps):
    """
    Kabsch (SVD) alignment of SLAM XY trajectory to GPS XY.
    Returns aligned SLAM positions (N,3) and per-frame errors.
    """
    if not slam or not gps:
        return None, None, None, None

    slam_valid = [s for s in slam if gps[0]['t_mono'] <= s['t_mono'] <= gps[-1]['t_mono']]
    if not slam_valid:
        return None, None, None, None

    slam_t  = np.array([e["t_mono"] for e in slam_valid])
    slam_pos = np.array([e["pos"] for e in slam_valid])      # (N,3)

    slam_xy  = slam_pos[:, :2]
    gps_xyz = np.array([interpolate_gps_vec(gps, t) for t in slam_t])
    gps_xy = gps_xyz[:, :2]

    # Kabsch with centroid centering
    slam_c, gps_c = slam_xy.mean(0), gps_xy.mean(0)
    H = (slam_xy - slam_c).T @ (gps_xy - gps_c)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T

    slam_xy_al = (slam_xy - slam_c) @ R.T + gps_c

    # Z: SLAM FRD → Up; origin-relative
    z0_slam = slam_pos[0, 2]
    z0_gps  = gps_xyz[0, 2]
    slam_z_al = -(slam_pos[:, 2] - z0_slam) + z0_gps   # aligned Z

    aligned_xyz = np.column_stack([slam_xy_al, slam_z_al])  # (M,3)

    err_xy  = np.linalg.norm(slam_xy_al - gps_xy, axis=1)
    err_z   = np.abs(slam_z_al - gps_xyz[:, 2])
    err_3d  = np.sqrt(err_xy**2 + err_z**2)

    return aligned_xyz, gps_xyz, slam_t, {
        "err_xy": err_xy, "err_z": err_z, "err_3d": err_3d,
        "mean_xy": err_xy.mean(), "max_xy": err_xy.max(),
        "mean_z":  err_z.mean(),  "max_z":  err_z.max(),
        "mean_3d": err_3d.mean(), "max_3d": err_3d.max(),
        "final_xy": err_xy[-1],   "final_z": err_z[-1],
    }


def alt_vz(alt, ema_alpha=0.3):
    """Compute smoothed vz from altimeter finite-difference."""
    if len(alt) < 2:
        return [], []
    ts, vzs = [], []
    vz_ema = 0.0
    for i in range(1, len(alt)):
        dt = alt[i]["t_mono"] - alt[i - 1]["t_mono"]
        if dt <= 0 or dt > 1.0:
            continue
        raw_vz = -(alt[i]["range_m"] - alt[i - 1]["range_m"]) / dt
        vz_ema = ema_alpha * raw_vz + (1 - ema_alpha) * vz_ema
        ts.append((alt[i]["t_mono"] + alt[i - 1]["t_mono"]) / 2.0)
        vzs.append(vz_ema)
    return ts, vzs


# ═══════════════════════════════════════════════════════════════════════════════
#  Figure Builder
# ═══════════════════════════════════════════════════════════════════════════════

def build_figure(gps, rio, slam, imu, alt, meta, log_path: str):
    matplotlib.rcParams.update(STYLE)

    fig = plt.figure(figsize=(22, 30), dpi=130)
    fig.patch.set_facecolor(STYLE["figure.facecolor"])

    log_name = os.path.basename(log_path)
    fig.suptitle(f"RIO Radar — Absolute Flight Analysis\n{log_name}",
                 fontsize=12, color="black", y=0.995, weight="bold")

    gs = gridspec.GridSpec(
        6, 4,
        figure=fig,
        hspace=0.55,
        wspace=0.38,
        left=0.06, right=0.97,
        top=0.97, bottom=0.03,
    )

    # ─── Pre-compute everything ──────────────────────────────────────────────
    t0 = _t0(gps, rio, slam)

    gps_enu   = np.array([e["enu"] for e in gps])           if gps  else np.zeros((0, 3))
    gps_t     = np.array([e["t_mono"] for e in gps]) - t0   if gps  else np.array([])
    gps_sats  = np.array([e.get("sats", 0) for e in gps])   if gps  else np.array([])

    rio_t     = np.array([e["t_mono"] for e in rio]) - t0   if rio  else np.array([])
    rio_speeds = np.array([math.sqrt(e["vx"]**2 + e["vy"]**2 + e["vz"]**2)
                           for e in rio])                    if rio  else np.array([])
    rio_vx    = np.array([e["vx"] for e in rio])            if rio  else np.array([])
    rio_vy    = np.array([e["vy"] for e in rio])            if rio  else np.array([])
    rio_vz    = np.array([e["vz"] for e in rio])            if rio  else np.array([])
    rio_inliers = np.array([e["inliers"] for e in rio])     if rio  else np.array([])
    rio_ntotal  = np.array([e.get("n_total", e["inliers"]) for e in rio]) if rio else np.array([])
    rio_cond  = np.array([e.get("cond", float("nan")) for e in rio]) if rio else np.array([])
    rio_static = np.array([e.get("is_static", False) for e in rio], bool) if rio else np.array([], bool)
    rio_airborne = np.array([e.get("airborne", False) for e in rio], bool) if rio else np.array([], bool)
    rio_vz_prior = np.array([e.get("vz_prior", False) for e in rio], bool) if rio else np.array([], bool)

    rio_pos, rio_dists = rio_integrate(rio)

    slam_t   = np.array([e["t_mono"] for e in slam]) - t0  if slam else np.array([])
    slam_nmap = np.array([e["n_map"] for e in slam])        if slam else np.array([])
    slam_fwd  = np.array([e.get("fwd_range", 0) for e in slam]) if slam else np.array([])

    imu_t   = np.array([e["t_mono"] for e in imu]) - t0    if imu  else np.array([])
    imu_roll  = np.degrees(np.array([e["roll"]  for e in imu])) if imu else np.array([])
    imu_pitch = np.degrees(np.array([e["pitch"] for e in imu])) if imu else np.array([])
    imu_yaw   = np.degrees(np.array([e["yaw"]   for e in imu])) if imu else np.array([])
    imu_omega = np.array([math.sqrt(e["wx"]**2 + e["wy"]**2 + e["wz"]**2)
                           for e in imu])                   if imu  else np.array([])

    alt_t    = np.array([e["t_mono"] for e in alt]) - t0   if alt  else np.array([])
    alt_range = np.array([e["range_m"] for e in alt])      if alt  else np.array([])
    alt_vz_t, alt_vz_vals = alt_vz(alt)
    alt_vz_t = np.array(alt_vz_t) - t0

    gps_vels = gps_derived_velocity_1hz(gps)
    gv_t     = np.array([v["t"] for v in gps_vels]) - t0   if gps_vels else np.array([])
    gv_speed = np.array([v["speed"] for v in gps_vels])     if gps_vels else np.array([])
    gv_vx    = np.array([v["v"][0] for v in gps_vels])      if gps_vels else np.array([])
    gv_vy    = np.array([v["v"][1] for v in gps_vels])      if gps_vels else np.array([])
    gv_vz    = np.array([v["v"][2] for v in gps_vels])      if gps_vels else np.array([])

    slam_aligned, gps_aligned_pts, err_times, pos_errs = align_slam_to_gps(slam, gps)
    if err_times is not None:
        err_t = err_times - t0

    gps_path = gps_path_length_1hz(gps)
    rio_total_dist = rio_dists[-1] if len(rio_dists) else 0.0
    dist_err_pct = (abs(rio_total_dist - gps_path) / max(gps_path, 1.0)) * 100

    duration = (max(np.concatenate([gps_t, rio_t, slam_t]))
                if any(len(x) > 0 for x in [gps_t, rio_t, slam_t]) else 0.0)

    # ─── Panel A: 2.5-D Trajectory ───────────────────────────────────────────
    ax_xy = fig.add_subplot(gs[0, :])
    ax_xy.set_facecolor(STYLE["axes.facecolor"])
    ax_xy.tick_params(colors="#666666", labelsize=7)

    legend_handles = []
    
    # We will use a MATLAB-like 'jet' colormap for the Z axis (Altitude)
    cmap = plt.get_cmap("jet")

    # 1. Plot GPS Truth
    if len(gps_enu):
        # Continuous line for the base trajectory
        ax_xy.plot(gps_enu[:, 0], gps_enu[:, 1], color="#cccccc", lw=2.0, alpha=0.5, zorder=1)
        # Scatter colored by Z
        sc = ax_xy.scatter(gps_enu[:, 0], gps_enu[:, 1], c=gps_enu[:, 2], 
                           cmap=cmap, s=15, alpha=0.8, edgecolor="none", zorder=2)
        cbar = fig.colorbar(sc, ax=ax_xy, pad=0.01, aspect=40)
        cbar.set_label("Altitude Z (m)", fontsize=7, color="#333333")
        cbar.ax.tick_params(labelsize=6, colors="#666666")
        
        legend_handles.append(Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=6, label="GPS (Color = Alt)"))
        ax_xy.scatter(gps_enu[0, 0], gps_enu[0, 1], c="black", s=60, zorder=5, marker="*", label="Start")
        ax_xy.scatter(gps_enu[-1, 0], gps_enu[-1, 1], c="black", s=60, zorder=5, marker="X", label="End")

    # 2. Plot RIO Integrated
    if len(rio_pos) > 1 and len(gps_enu):
        rio_shifted = rio_pos + gps_enu[0] - rio_pos[0]
        ax_xy.plot(rio_shifted[:, 0], rio_shifted[:, 1], color=C_RIO, lw=2.0, alpha=0.8, linestyle="--", zorder=3)
        legend_handles.append(Line2D([0], [0], color=C_RIO, lw=2, ls="--", label="RIO integrated"))

    # 3. Plot SLAM Aligned
    if slam_aligned is not None:
        ax_xy.plot(slam_aligned[:, 0], slam_aligned[:, 1], color=C_SLAM, lw=2.0, alpha=0.8, linestyle="-.", zorder=4)
        legend_handles.append(Line2D([0], [0], color=C_SLAM, lw=2, ls="-.", label="SLAM (Kabsch aligned)"))

    ax_xy.set_xlabel("East (m)", fontsize=7)
    ax_xy.set_ylabel("North (m)", fontsize=7)
    ax_xy.set_title("A — 2.5D Trajectory (MATLAB style Z-Colormap)", pad=4)
    ax_xy.set_aspect("equal")
    
    if legend_handles:
        ax_xy.legend(handles=legend_handles, loc="upper left", fontsize=7)

    # ─── Panel B: Speed time-series ──────────────────────────────────────────
    ax_spd = fig.add_subplot(gs[1, :2])
    if len(rio_t):
        ax_spd.plot(rio_t, rio_speeds, color=C_RIO, lw=1.2, label="RIO speed")
        # Shade static frames
        if rio_static.any():
            ax_spd.fill_between(rio_t, 0, rio_speeds, where=rio_static,
                                 color="blue", alpha=0.15, label="Static frames")
    if len(gv_t):
        ax_spd.plot(gv_t, gv_speed, color=C_GPS, lw=1.5, ls="--", label="GPS speed (1 Hz)")
    ax_spd.set_xlabel("Time (s)")
    ax_spd.set_ylabel("Speed (m/s)")
    ax_spd.set_title(f"B — Speed Comparison  (RIO dist error: {dist_err_pct:.1f}%)")
    ax_spd.legend()
    ax_spd.set_xlim(0, duration)

    # ─── Panel C: Per-axis velocity ───────────────────────────────────────────
    ax_vel = fig.add_subplot(gs[1, 2:])
    if len(rio_t):
        ax_vel.plot(rio_t, rio_vx, color="blue", lw=0.9, label="RIO Vx")
        ax_vel.plot(rio_t, rio_vy, color="green", lw=0.9, label="RIO Vy")
        ax_vel.plot(rio_t, rio_vz, color="red", lw=0.9, label="RIO Vz")
    if len(gv_t):
        ax_vel.plot(gv_t, gv_vx, color="blue", lw=1.4, ls="--", alpha=0.6, label="GPS Vx")
        ax_vel.plot(gv_t, gv_vy, color="green", lw=1.4, ls="--", alpha=0.6, label="GPS Vy")
        ax_vel.plot(gv_t, gv_vz, color="red", lw=1.4, ls="--", alpha=0.6, label="GPS Vz")
    ax_vel.axhline(0, color="#cccccc", lw=0.7)
    ax_vel.set_xlabel("Time (s)")
    ax_vel.set_ylabel("Velocity (m/s)")
    ax_vel.set_title("C — Per-Axis Velocity (RIO solid / GPS dashed)")
    ax_vel.legend(ncol=2)
    ax_vel.set_xlim(0, duration)

    # ─── Panel D: SLAM XY error ───────────────────────────────────────────────
    ax_exy = fig.add_subplot(gs[2, :2])
    if pos_errs is not None:
        gate_color = C_GOOD if pos_errs["mean_xy"] < 5.0 else C_ERR
        ax_exy.plot(err_t, pos_errs["err_xy"], color=gate_color, lw=1.3)
        ax_exy.axhline(5.0, color="orange", lw=0.8, ls="--", label="Gate: 5 m")
        ax_exy.fill_between(err_t, 0, pos_errs["err_xy"], alpha=0.15, color=gate_color)
        ax_exy.text(0.99, 0.94, f"mean={pos_errs['mean_xy']:.2f} m  max={pos_errs['max_xy']:.2f} m",
                    transform=ax_exy.transAxes, ha="right", va="top",
                    fontsize=7, color="black",
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#cccccc"))
        ax_exy.legend()
    ax_exy.set_xlabel("Time (s)")
    ax_exy.set_ylabel("Error (m)")
    ax_exy.set_title("D — SLAM vs GPS  XY Horizontal Error")
    ax_exy.set_xlim(0, duration)

    # ─── Panel E: SLAM Z error ────────────────────────────────────────────────
    ax_ez = fig.add_subplot(gs[2, 2:])
    if pos_errs is not None:
        gate_color_z = C_GOOD if pos_errs["mean_z"] < 2.0 else C_ERR
        ax_ez.plot(err_t, pos_errs["err_z"], color=gate_color_z, lw=1.3)
        ax_ez.axhline(2.0, color="orange", lw=0.8, ls="--", label="Gate: 2 m")
        ax_ez.fill_between(err_t, 0, pos_errs["err_z"], alpha=0.15, color=gate_color_z)
        ax_ez.text(0.99, 0.94, f"mean={pos_errs['mean_z']:.2f} m  max={pos_errs['max_z']:.2f} m",
                   transform=ax_ez.transAxes, ha="right", va="top",
                   fontsize=7, color="black",
                   bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#cccccc"))
        ax_ez.legend()
    ax_ez.set_xlabel("Time (s)")
    ax_ez.set_ylabel("Error (m)")
    ax_ez.set_title("E — SLAM vs GPS  Z Vertical Error")
    ax_ez.set_xlim(0, duration)

    # ─── Panel F: RIO Inliers ─────────────────────────────────────────────────
    ax_inl = fig.add_subplot(gs[3, :2])
    if len(rio_t):
        ax_inl.fill_between(rio_t, 0, rio_ntotal, color="#eeeeee", label="Total pts")
        ax_inl.fill_between(rio_t, 0, rio_inliers, color=C_RIO, alpha=0.7, label="Inliers")
        inlier_ratio = rio_inliers / np.maximum(rio_ntotal, 1)
        ax2_inl = ax_inl.twinx()
        ax2_inl.plot(rio_t, inlier_ratio * 100, color="orange", lw=0.8, ls="--", alpha=0.7)
        ax2_inl.set_ylabel("Inlier %", color="orange", fontsize=7)
        ax2_inl.tick_params(colors="orange", labelsize=6)
        ax2_inl.set_ylim(0, 110)
    ax_inl.set_xlabel("Time (s)")
    ax_inl.set_ylabel("Points")
    ax_inl.set_title("F — RIO Radar Pipeline: Inlier Count")
    ax_inl.legend(loc="upper left")
    ax_inl.set_xlim(0, duration)

    # ─── Panel G: Condition number + flags ───────────────────────────────────
    ax_cond = fig.add_subplot(gs[3, 2:])
    if len(rio_t):
        cond_valid = np.where(np.isfinite(rio_cond), rio_cond, 0)
        ax_cond.plot(rio_t, cond_valid, color=C_ALT, lw=0.9, label="Condition #")
        ax_cond.axhline(50, color=C_ERR, lw=0.7, ls="--", label="Reject gate (50)")
        if rio_vz_prior.any():
            ax_cond.fill_between(rio_t, 0, cond_valid, where=rio_vz_prior,
                                  color=C_SLAM, alpha=0.2, label="Vz prior active")
        if rio_airborne.any():
            ymax = cond_valid.max() if cond_valid.max() > 0 else 60
            ax_cond.fill_between(rio_t, 0, ymax, where=~rio_airborne,
                                  color="#eeeeee", alpha=0.5, label="On ground")
    ax_cond.set_xlabel("Time (s)")
    ax_cond.set_ylabel("Condition #")
    ax_cond.set_title("G — RIO Condition Number + Vz Prior Fusion")
    ax_cond.legend(loc="upper right")
    ax_cond.set_xlim(0, duration)

    # ─── Panel H: IMU Pitch + Roll ────────────────────────────────────────────
    ax_att = fig.add_subplot(gs[4, :2])
    if len(imu_t):
        ax_att.plot(imu_t, imu_pitch, color=C_IMU, lw=0.9, label="Pitch (°)")
        ax_att.plot(imu_t, imu_roll, color=C_GPS, lw=0.9, alpha=0.7, label="Roll (°)")
        ax_att.axhline(0, color="#cccccc", lw=0.5)
    ax_att.set_xlabel("Time (s)")
    ax_att.set_ylabel("Degrees")
    ax_att.set_title("H — IMU Attitude: Pitch & Roll")
    ax_att.legend()
    ax_att.set_xlim(0, duration)

    # ─── Panel I: IMU Yaw + Angular rate ─────────────────────────────────────
    ax_yaw = fig.add_subplot(gs[4, 2:])
    if len(imu_t):
        ax_yaw.plot(imu_t, imu_yaw, color=C_SLAM, lw=0.9, label="Yaw (°)")
        ax2_yaw = ax_yaw.twinx()
        ax2_yaw.plot(imu_t, imu_omega, color="orange", lw=0.7, alpha=0.7, label="Rot rate (rad/s)")
        ax2_yaw.set_ylabel("Rot rate (rad/s)", color="orange", fontsize=7)
        ax2_yaw.tick_params(colors="orange", labelsize=6)
    ax_yaw.set_xlabel("Time (s)")
    ax_yaw.set_ylabel("Degrees")
    ax_yaw.set_title("I — IMU Yaw + Angular Rate")
    ax_yaw.legend(loc="upper left")
    ax_yaw.set_xlim(0, duration)

    # ─── Panel J: Altimeter + Computed Vz ────────────────────────────────────
    ax_alt = fig.add_subplot(gs[5, :2])
    if len(alt_t):
        ax_alt.plot(alt_t, alt_range, color=C_ALT, lw=1.2, label="AGL range (m)")
        if len(alt_vz_vals):
            ax2_alt = ax_alt.twinx()
            ax2_alt.plot(alt_vz_t, alt_vz_vals, color=C_IMU, lw=0.8, alpha=0.7, label="Vz from alt (m/s)")
            ax2_alt.axhline(0, color="#cccccc", lw=0.5)
            ax2_alt.set_ylabel("Vz (m/s)", color=C_IMU, fontsize=7)
            ax2_alt.tick_params(colors=C_IMU, labelsize=6)
    ax_alt.set_xlabel("Time (s)")
    ax_alt.set_ylabel("Range AGL (m)")
    ax_alt.set_title("J — Altimeter AGL + Computed Vertical Velocity")
    ax_alt.legend(loc="upper right")
    ax_alt.set_xlim(0, duration)

    # ─── Panel K: SLAM Map + Forward Range ───────────────────────────────────
    ax_map = fig.add_subplot(gs[5, 2:])
    if len(slam_t):
        ax_map.fill_between(slam_t, 0, slam_nmap, color=C_SLAM, alpha=0.5, label="Map pts")
        ax_map.plot(slam_t, slam_nmap, color=C_SLAM, lw=1.2)
        fwd_valid = slam_fwd[(slam_fwd > 0) & (slam_fwd < 1e5)]
        fwd_t_valid = slam_t[(slam_fwd > 0) & (slam_fwd < 1e5)]
        if len(fwd_t_valid):
            ax2_map = ax_map.twinx()
            ax2_map.plot(fwd_t_valid, fwd_valid, color=C_ERR, lw=1.0, label="Fwd range (m)")
            ax2_map.set_ylabel("Fwd range (m)", color=C_ERR, fontsize=7)
            ax2_map.tick_params(colors=C_ERR, labelsize=6)
    ax_map.set_xlabel("Time (s)")
    ax_map.set_ylabel("Map size (pts)")
    ax_map.set_title("K — SLAM Map Growth + Obstacle Range")
    ax_map.legend(loc="upper left")
    ax_map.set_xlim(0, duration)

    # ─── Summary text box (replaces row 6) ───────────────────────────────────
    # We add an extra subplot spanning the bottom
    ax_sum = fig.add_axes([0.06, 0.01, 0.91, 0.045])
    ax_sum.set_facecolor("white")
    ax_sum.axis("off")

    def gate(val, thr, fmt, inv=False):
        ok = (val < thr) if not inv else (val > thr)
        icon = "PASS" if ok else "FAIL"
        return f"[{icon}] {fmt.format(val)}"

    lines = [
        f"Duration: {duration:.1f}s   |   GPS: {len(gps)} fixes   "
        f"RIO: {len(rio)} frames @ {len(rio)/max(duration,1):.1f}Hz   "
        f"SLAM: {len(slam)} kf   IMU: {len(imu)} pkts   Alt: {len(alt)} pkts",
    ]
    dist_part = gate(dist_err_pct, 5.0, "Dist err: {:.1f}%")
    xy_part = gate(pos_errs["final_xy"], 5.0, "SLAM XY final: {:.2f}m") if pos_errs else "SLAM XY: N/A"
    z_part  = gate(pos_errs["final_z"],  2.0, "SLAM Z final: {:.2f}m")  if pos_errs else "SLAM Z: N/A"

    if pos_errs:
        t_arr = err_t
        if len(t_arr) >= 3:
            coeffs = np.polyfit(t_arr, pos_errs["err_3d"], 1)
            drift = abs(coeffs[0])
            drift_part = gate(drift, 0.05, "Drift: {:.4f}m/s")
        else:
            drift_part = "Drift: N/A"
    else:
        drift_part = "Drift: N/A"

    rio_mean_speed = float(rio_speeds.mean()) if len(rio_speeds) else 0.0
    slam_final_pts = int(slam_nmap[-1]) if len(slam_nmap) else 0

    lines.append(
        f"{dist_part}   |   {xy_part}   |   {z_part}   |   "
        f"{drift_part}   |   RIO mean speed: {rio_mean_speed:.2f}m/s   "
        f"SLAM final map: {slam_final_pts} pts"
    )

    ax_sum.text(0.5, 0.8, lines[0], transform=ax_sum.transAxes,
                ha="center", va="top", fontsize=7.5, color="#666666",
                fontfamily="monospace")
    ax_sum.text(0.5, 0.1, lines[1], transform=ax_sum.transAxes,
                ha="center", va="bottom", fontsize=8.5, color="black",
                fontfamily="monospace", weight="bold")

    return fig


# ═══════════════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    print(_DEPRECATION)
    p = argparse.ArgumentParser(
        description="Absolute visual analyzer — generates MATLAB-quality plot from JSONL log")
    p.add_argument("log_file", help="Path to JSONL log produced by gps_logger.py")
    p.add_argument("--out", default=None,
                   help="Output PNG path (default: <log_file_base>_analysis.png)")
    p.add_argument("--dpi", type=int, default=130, help="Output resolution (default 130)")
    p.add_argument("--show", action="store_true",
                   help="Open figure in window after saving (needs display)")
    args = p.parse_args()

    if not os.path.exists(args.log_file):
        print(f"ERROR: File not found: {args.log_file}")
        sys.exit(1)

    print(f"Loading {args.log_file} ...")
    gps, rio, slam, imu, alt, meta = load_log(args.log_file)
    print(f"  GPS:{len(gps)}  RIO:{len(rio)}  SLAM:{len(slam)}  IMU:{len(imu)}  Alt:{len(alt)}")

    print("Building figure ...")
    fig = build_figure(gps, rio, slam, imu, alt, meta, args.log_file)

    out = args.out or os.path.splitext(args.log_file)[0] + "_analysis.png"
    fig.savefig(out, dpi=args.dpi, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    print(f"\n✅ Saved Image: {out}")

    # --- Append the text terminal report from analyze_run.py ---
    print("\n\n" + "═" * 72)
    print("  TEXT ANALYSIS REPORT (from analyze_run.py)")
    print("═" * 72)
    
    # We pass the same loaded data straight into the text printer
    print_report(gps, rio, slam, imu, alt, meta, args.log_file)
    
    print("\n✅ Done!")

    if args.show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
