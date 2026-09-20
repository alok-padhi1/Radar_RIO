import nbformat as nbf

nb = nbf.v4.new_notebook()

cells = []

# Markdown Header
cells.append(nbf.v4.new_markdown_cell("""\
# RIO Radar — Absolute Master Visualizer (Notebook Edition)
This notebook is based on `plot_run.py` and `analyze_run.py`. It loads all the telemetry and plots each panel individually so you don't have to zoom into one massive image.

Set your log file path below and run all cells!
"""))

# Cell 1: Imports and Setup
cells.append(nbf.v4.new_code_cell("""\
import json
import math
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# Notebook plotting setup
%matplotlib inline
plt.rcParams['figure.figsize'] = (10, 5)
plt.rcParams['axes.grid'] = True
plt.rcParams['figure.facecolor'] = 'white'

LOG_FILE = "../logs/run_20260920_151941.jsonl"
"""))

# Cell 2: Data Loading & Computation functions
cells.append(nbf.v4.new_code_cell("""\
def load_log(path):
    gps, rio, slam, imu, alt, meta = [], [], [], [], [], {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line: continue
            try: e = json.loads(line)
            except: continue
            t = e.get("type")
            if   t == "gps":       gps.append(e)
            elif t == "rio":       rio.append(e)
            elif t == "slam":      slam.append(e)
            elif t == "imu":       imu.append(e)
            elif t == "altimeter": alt.append(e)
            elif t == "meta":      meta = e
    return gps, rio, slam, imu, alt, meta

def _t0(gps, rio, slam):
    times = ([e["t_mono"] for e in gps] + [e["t_mono"] for e in rio] + [e["t_mono"] for e in slam])
    return min(times) if times else 0.0

def rio_integrate(rio):
    if not rio: return np.zeros((0, 3))
    pos = np.zeros(3)
    positions = [pos.copy()]
    for i in range(1, len(rio)):
        dt = rio[i]["t_mono"] - rio[i - 1]["t_mono"]
        if dt > 0:
            v_prev = np.array([rio[i - 1]["vx"], rio[i - 1]["vy"], rio[i - 1]["vz"]])
            v_curr = np.array([rio[i]["vx"], rio[i]["vy"], rio[i]["vz"]])
            v_avg = (v_prev + v_curr) / 2.0
            pos = pos + v_avg * dt
        positions.append(pos.copy())
    return np.array(positions)

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
    if not slam or not gps: return None, None
    slam_valid = [s for s in slam if gps[0]['t_mono'] <= s['t_mono'] <= gps[-1]['t_mono']]
    if not slam_valid: return None, None
    slam_t = np.array([e["t_mono"] for e in slam_valid])
    slam_pos = np.array([e["pos"] for e in slam_valid])
    slam_xy = slam_pos[:, :2]
    gps_xyz = np.array([interpolate_gps_vec(gps, t) for t in slam_t])
    gps_xy = gps_xyz[:, :2]
    slam_c, gps_c = slam_xy.mean(0), gps_xy.mean(0)
    H = (slam_xy - slam_c).T @ (gps_xy - gps_c)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    slam_xy_al = (slam_xy - slam_c) @ R.T + gps_c
    z0_slam, z0_gps = slam_pos[0, 2], gps_xyz[0, 2]
    slam_z_al = -(slam_pos[:, 2] - z0_slam) + z0_gps
    aligned_xyz = np.column_stack([slam_xy_al, slam_z_al])
    err_xy = np.linalg.norm(slam_xy_al - gps_xy, axis=1)
    err_z = np.abs(slam_z_al - gps_xyz[:, 2])
    return aligned_xyz, {"err_xy": err_xy, "err_z": err_z, "t": slam_t}

def alt_vz(alt, ema=0.3):
    ts, vzs, ema_val = [], [], 0.0
    for i in range(1, len(alt)):
        dt = alt[i]["t_mono"] - alt[i - 1]["t_mono"]
        if 0 < dt <= 1.0:
            raw = -(alt[i]["range_m"] - alt[i - 1]["range_m"]) / dt
            ema_val = ema * raw + (1 - ema) * ema_val
            ts.append((alt[i]["t_mono"] + alt[i-1]["t_mono"]) / 2.0)
            vzs.append(ema_val)
    return ts, vzs

def gps_velocity(gps):
    vels, last = [], 0
    for i in range(1, len(gps)):
        dt = gps[i]["t_mono"] - gps[last]["t_mono"]
        if dt >= 0.5:
            v = (np.array(gps[i]["enu"]) - np.array(gps[last]["enu"])) / dt
            vels.append({"t": (gps[i]["t_mono"] + gps[last]["t_mono"])/2, "v": v, "spd": np.linalg.norm(v)})
            last = i
    return vels
"""))

# Cell 3: Load Data
cells.append(nbf.v4.new_code_cell("""\
# Execute data loading
gps, rio, slam, imu, alt, meta = load_log(LOG_FILE)
t0 = _t0(gps, rio, slam)

gps_enu = np.array([e["enu"] for e in gps]) if gps else np.zeros((0,3))
rio_pos = rio_integrate(rio)
slam_al, pos_errs = align_slam_to_gps(slam, gps)

print(f"Loaded {len(gps)} GPS, {len(rio)} RIO, {len(slam)} SLAM, {len(imu)} IMU, {len(alt)} ALT")
"""))

# Cell 4: Trajectory Plot
cells.append(nbf.v4.new_markdown_cell("### Panel A: 2.5D Trajectory (Z-Axis mapped to Color)"))
cells.append(nbf.v4.new_code_cell("""\
plt.figure(figsize=(8,8))
if len(gps_enu):
    plt.plot(gps_enu[:,0], gps_enu[:,1], color='gray', alpha=0.5)
    sc = plt.scatter(gps_enu[:,0], gps_enu[:,1], c=gps_enu[:,2], cmap='jet', s=15, alpha=0.8)
    plt.colorbar(sc, label="Altitude Z (m)")
    
if len(rio_pos) > 1 and len(gps_enu):
    rio_shifted = rio_pos + gps_enu[0] - rio_pos[0]
    plt.plot(rio_shifted[:,0], rio_shifted[:,1], 'r--', label="RIO Integrated")
    
if slam_al is not None:
    plt.plot(slam_al[:,0], slam_al[:,1], 'g-.', label="SLAM Aligned")

plt.xlabel("East (m)")
plt.ylabel("North (m)")
plt.title("2.5D Trajectory (Color = Altitude)")
plt.axis('equal')
plt.legend()
plt.show()
"""))

# Cell 5: Speed Plot
cells.append(nbf.v4.new_markdown_cell("### Panel B & C: Velocities and Speeds"))
cells.append(nbf.v4.new_code_cell("""\
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

rio_t = np.array([e["t_mono"] for e in rio]) - t0
rio_spd = np.array([math.sqrt(e["vx"]**2 + e["vy"]**2 + e["vz"]**2) for e in rio])
gps_vels = gps_velocity(gps)
gv_t = np.array([v["t"] for v in gps_vels]) - t0

if len(rio_t): ax1.plot(rio_t, rio_spd, 'r-', label="RIO Speed")
if len(gv_t): ax1.plot(gv_t, [v["spd"] for v in gps_vels], 'b--', label="GPS Speed")
ax1.set_title("Speed Comparison")
ax1.set_ylabel("Speed (m/s)")
ax1.legend()

if len(rio_t):
    ax2.plot(rio_t, [e["vx"] for e in rio], 'b-', alpha=0.7, label="RIO Vx")
    ax2.plot(rio_t, [e["vy"] for e in rio], 'g-', alpha=0.7, label="RIO Vy")
    ax2.plot(rio_t, [e["vz"] for e in rio], 'r-', alpha=0.7, label="RIO Vz")
if len(gv_t):
    ax2.plot(gv_t, [v["v"][0] for v in gps_vels], 'b--', alpha=0.7, label="GPS Vx")
    ax2.plot(gv_t, [v["v"][1] for v in gps_vels], 'g--', alpha=0.7, label="GPS Vy")
    ax2.plot(gv_t, [v["v"][2] for v in gps_vels], 'r--', alpha=0.7, label="GPS Vz")
    
ax2.axhline(0, color='gray', lw=1)
ax2.set_title("Per-Axis Velocity")
ax2.set_ylabel("Velocity (m/s)")
ax2.set_xlabel("Time (s)")
ax2.legend(ncol=2)
plt.show()
"""))

# Cell 6: SLAM Error
cells.append(nbf.v4.new_markdown_cell("### Panel D & E: SLAM Errors"))
cells.append(nbf.v4.new_code_cell("""\
if pos_errs is not None:
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
    err_t = pos_errs["t"] - t0
    
    ax1.plot(err_t, pos_errs["err_xy"], 'g-')
    ax1.axhline(5, color='orange', ls='--', label='5m Gate')
    ax1.fill_between(err_t, 0, pos_errs["err_xy"], color='green', alpha=0.15)
    ax1.set_title(f"SLAM XY Error (Mean: {pos_errs['err_xy'].mean():.2f}m)")
    ax1.set_ylabel("Error (m)")
    ax1.set_xlabel("Time (s)")
    ax1.legend()
    
    ax2.plot(err_t, pos_errs["err_z"], 'r-')
    ax2.axhline(2, color='orange', ls='--', label='2m Gate')
    ax2.fill_between(err_t, 0, pos_errs["err_z"], color='red', alpha=0.15)
    ax2.set_title(f"SLAM Z Error (Mean: {pos_errs['err_z'].mean():.2f}m)")
    ax2.set_ylabel("Error (m)")
    ax2.set_xlabel("Time (s)")
    ax2.legend()
    
    plt.show()
"""))

# Cell 7: Altimeter
cells.append(nbf.v4.new_markdown_cell("### Panel J: Altimeter and Vz Tracking"))
cells.append(nbf.v4.new_code_cell("""\
if alt:
    fig, ax1 = plt.subplots(figsize=(10, 4))
    alt_t = np.array([e["t_mono"] for e in alt]) - t0
    ax1.plot(alt_t, [e["range_m"] for e in alt], 'purple', label="AGL Range")
    ax1.set_ylabel("AGL (m)", color='purple')
    
    ax2 = ax1.twinx()
    vt, vv = alt_vz(alt)
    if vt:
        vt = np.array(vt) - t0
        ax2.plot(vt, vv, 'orange', alpha=0.8, label="Vz from Alt")
        ax2.axhline(0, color='gray', lw=1)
        ax2.set_ylabel("Computed Vz (m/s)", color='orange')
    
    ax1.set_title("Altimeter Range and Derived Vz")
    ax1.set_xlabel("Time (s)")
    fig.legend(loc="upper right", bbox_to_anchor=(0.9, 0.9))
    plt.show()
"""))

# Cell 8: IMU
cells.append(nbf.v4.new_markdown_cell("### Panel H & I: IMU Attitude and Rotation Rates"))
cells.append(nbf.v4.new_code_cell("""\
if imu:
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    imu_t = np.array([e["t_mono"] for e in imu]) - t0
    
    ax1.plot(imu_t, np.degrees([e["pitch"] for e in imu]), color='orange', label="Pitch")
    ax1.plot(imu_t, np.degrees([e["roll"] for e in imu]), color='blue', alpha=0.5, label="Roll")
    ax1.axhline(0, color='gray', lw=1)
    ax1.set_title("IMU Pitch & Roll")
    ax1.set_ylabel("Degrees")
    ax1.legend()
    
    ax2.plot(imu_t, np.degrees([e["yaw"] for e in imu]), color='green', label="Yaw")
    omega = [math.sqrt(e["wx"]**2 + e["wy"]**2 + e["wz"]**2) for e in imu]
    ax2_twin = ax2.twinx()
    ax2_twin.plot(imu_t, omega, color='red', alpha=0.5, label="Rot Rate")
    
    ax2.set_title("IMU Yaw & Angular Rate")
    ax2.set_ylabel("Degrees (Yaw)", color='green')
    ax2_twin.set_ylabel("Rad/s (Rate)", color='red')
    ax2.set_xlabel("Time (s)")
    plt.show()
"""))

# Append cells to notebook
nb['cells'] = cells

with open("/home/alok/radar/rio_stack/tools/master_visualizer.ipynb", "w") as f:
    nbf.write(nb, f)

print("Notebook generated at /home/alok/radar/rio_stack/tools/master_visualizer.ipynb")
