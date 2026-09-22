import json
import os
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.collections as mcoll

def extract_yaw_from_rot9(R):
    return math.atan2(R[3], R[0])

def make_segments(x, y):
    points = np.array([x, y]).T.reshape(-1, 1, 2)
    return np.concatenate([points[:-1], points[1:]], axis=1)

def analyze_and_plot(log_file, output_image_path):
    print(f"Parsing {log_file}...")
    t_gps, gps_e, gps_n, gps_u = [], [], [], []
    gps_v_fwd, gps_v_right, gps_v_down = [], [], []
    gps_hdg = []

    t_rio, rio_vx, rio_vy, rio_vz = [], [], [], []
    
    t_slam, slam_x, slam_y, slam_z = [], [], [], []
    slam_hdg = []
    
    t_slam_rej = []

    last_yaw = 0.0
    
    with open(log_file) as f:
        for line in f:
            if not line.strip(): continue
            try: d = json.loads(line)
            except: continue

            t = d.get('t_mono')
            if t is None: continue

            msg_type = d.get('type')
            
            if msg_type == 'gps' and 'enu' in d and 'hdg_deg' in d:
                e, n, u = d['enu']
                ve, vn, vu = d.get('v_enu', [0, 0, 0])
                yaw = math.radians(d['hdg_deg'])
                last_yaw = yaw
                
                v_fwd = vn * math.cos(yaw) + ve * math.sin(yaw)
                v_right = ve * math.cos(yaw) - vn * math.sin(yaw)
                
                t_gps.append(t)
                gps_e.append(e)
                gps_n.append(n)
                gps_u.append(u)
                gps_v_fwd.append(v_fwd)
                gps_v_right.append(v_right)
                gps_v_down.append(-vu)
                gps_hdg.append(yaw)
                
            elif msg_type == 'rio' and 'vx' in d:
                t_rio.append(t)
                rio_vx.append(d['vx'])
                rio_vy.append(d['vy'])
                rio_vz.append(d['vz'])
                
            elif msg_type == 'slam' and 'pos' in d:
                t_slam.append(t)
                sx, sy, sz = d['pos']
                slam_x.append(sx)
                slam_y.append(sy)
                slam_z.append(sz)
                if 'R' in d and len(d['R']) == 9:
                    slam_hdg.append(extract_yaw_from_rot9(d['R']))
                else:
                    slam_hdg.append(0.0)
                    
            elif msg_type == 'slam_reject':
                t_slam_rej.append(t)

    if not t_gps or not t_rio:
        print(f"Not enough data in {log_file} to plot.")
        return

    # Integrate RIO velocity to create a pure RIO path
    rio_path_e = [gps_e[0]]
    rio_path_n = [gps_n[0]]
    last_t = t_rio[0]
    
    # We need yaw for RIO integration. Interpolate GPS yaw to RIO times.
    rio_yaws = np.interp(t_rio, t_gps, gps_hdg)
    
    for i in range(1, len(t_rio)):
        dt = t_rio[i] - t_rio[i-1]
        # Cap dt to prevent massive jumps during dropouts, or just integrate anyway
        # If dt > 0.5s, EKF would coast, but here we just integrate the latest velocity over dt.
        # Actually, let's limit dt to 0.2s. If there's a larger gap, RIO didn't move in our dead-reckoning.
        if dt > 0.2: dt = 0.2 
        
        yaw = rio_yaws[i]
        vx = rio_vx[i-1] # fwd
        vy = rio_vy[i-1] # right
        
        ve = vx * math.sin(yaw) + vy * math.cos(yaw)
        vn = vx * math.cos(yaw) - vy * math.sin(yaw)
        
        rio_path_e.append(rio_path_e[-1] + ve * dt)
        rio_path_n.append(rio_path_n[-1] + vn * dt)

    # Determine RIO Active/Dropout status for GPS trajectory coloring
    # A GPS point is "active" if a RIO packet arrived within 150ms of it.
    t_gps_arr = np.array(t_gps)
    t_rio_arr = np.array(t_rio)
    gps_rio_active = np.zeros(len(t_gps_arr), dtype=bool)
    
    for i, tg in enumerate(t_gps_arr):
        # find closest rio time
        idx = np.searchsorted(t_rio_arr, tg)
        dists = []
        if idx < len(t_rio_arr): dists.append(abs(t_rio_arr[idx] - tg))
        if idx > 0: dists.append(abs(t_rio_arr[idx-1] - tg))
        if min(dists, default=999) < 0.15:
            gps_rio_active[i] = True

    fig = plt.figure(figsize=(18, 14))
    fig.suptitle(f"Flight Analysis: {os.path.basename(log_file)}", fontsize=18)

    t0 = t_gps[0]
    t_gps_arr -= t0
    t_rio_arr -= t0
    t_slam = np.array(t_slam) - t0
    t_slam_rej = np.array(t_slam_rej) - t0

    # 1. 2D Top View (X-Y) Trajectory Map
    ax_top = plt.subplot2grid((3, 2), (0, 0), rowspan=2)
    
    # Plot SLAM
    ax_top.plot(slam_x, slam_y, label="SLAM Map Path", color='orange', alpha=0.8, linewidth=2)
    
    # Plot RIO Dead-Reckoning
    ax_top.plot(rio_path_e, rio_path_n, label="RIO Dead-Reckoning (Integrated Velocity)", color='purple', alpha=0.8, linewidth=2, linestyle='--')
    
    # Plot GPS Trajectory with Color-coded Dropout Segments
    # Create segments for LineCollection
    segments = make_segments(gps_e, gps_n)
    colors = ['green' if active else 'red' for active in gps_rio_active[:-1]]
    lc = mcoll.LineCollection(segments, colors=colors, linewidths=3, alpha=0.7)
    ax_top.add_collection(lc)
    
    # Dummy lines for GPS legend
    ax_top.plot([], [], color='green', linewidth=3, label="GPS Path (RIO Active)")
    ax_top.plot([], [], color='red', linewidth=3, label="GPS Path (RIO Dropped > 150ms)")
    
    # Arrows for heading
    stride_gps = max(1, len(gps_e) // 40)
    stride_slam = max(1, len(slam_x) // 40)
    
    gps_dx = np.sin(np.array(gps_hdg)[::stride_gps])
    gps_dy = np.cos(np.array(gps_hdg)[::stride_gps])
    ax_top.quiver(np.array(gps_e)[::stride_gps], np.array(gps_n)[::stride_gps], 
                  gps_dx, gps_dy, color='black', alpha=0.5, scale=25, width=0.004, headwidth=4)
                  
    if len(slam_x) > 0:
        slam_dx = np.cos(np.array(slam_hdg)[::stride_slam])
        slam_dy = np.sin(np.array(slam_hdg)[::stride_slam])
        ax_top.quiver(np.array(slam_x)[::stride_slam], np.array(slam_y)[::stride_slam], 
                      slam_dx, slam_dy, color='orange', alpha=0.8, scale=25, width=0.004, headwidth=4)
                      
    ax_top.set_title("Top View Trajectory (Dropouts Mapped to Location)")
    ax_top.set_xlabel("East / SLAM X [m]")
    ax_top.set_ylabel("North / SLAM Y [m]")
    ax_top.axis('equal')
    ax_top.grid(True)
    ax_top.legend(loc="upper left")

    # 2. Side View (Z vs Time)
    ax_side = plt.subplot2grid((3, 2), (0, 1))
    ax_side.plot(t_gps_arr, gps_u, label="GPS Up (Altitude)", color='blue')
    ax_side.plot(t_slam, slam_z, label="SLAM Z (Depth)", color='orange')
    ax_side.set_title("Side View (Altitude Z vs Time)")
    ax_side.set_ylabel("Z [m]")
    ax_side.grid(True)
    ax_side.legend(loc="upper left")

    # 3. Fwd Velocity vs Time
    ax_vf = plt.subplot2grid((3, 2), (1, 1))
    ax_vf.plot(t_gps_arr, gps_v_fwd, label="GPS Fwd (Projected)", color='blue', alpha=0.5)
    ax_vf.scatter(t_rio_arr, rio_vx, label="RIO Vx", color='purple', s=8, alpha=0.8)
    ax_vf.set_title("Forward Velocity (Body X) vs Time")
    ax_vf.set_ylabel("Velocity [m/s]")
    ax_vf.grid(True)
    ax_vf.legend(loc="upper left")

    # 4. Status / Dropout Timeline
    ax_tl = plt.subplot2grid((3, 2), (2, 0), colspan=2)
    ax_tl.set_title("Timeline: RIO Dropouts & SLAM Rejections")
    ax_tl.set_xlabel("Time (s)")
    ax_tl.set_yticks([])
    
    ax_tl.scatter(t_rio_arr, np.ones_like(t_rio_arr), color='green', s=5, label='RIO Frame Passed')
    ax_tl.scatter(t_slam_rej, np.ones_like(t_slam_rej)*0.8, color='orange', s=20, marker='x', label='SLAM Frame Rejected')
    
    rio_gaps = np.diff(t_rio_arr)
    for i, gap in enumerate(rio_gaps):
        if gap > 0.15:
            ax_tl.axvspan(t_rio_arr[i], t_rio_arr[i+1], color='red', alpha=0.3)
            
    import matplotlib.patches as mpatches
    gap_patch = mpatches.Patch(color='red', alpha=0.3, label='RIO Gap (>150ms) = EKF Coasting')
    handles, labels = ax_tl.get_legend_handles_labels()
    handles.append(gap_patch)
    labels.append('RIO Gap (>150ms)')
    ax_tl.legend(handles=handles, labels=labels, loc="upper right")
    ax_tl.grid(True, axis='x')

    plt.tight_layout()
    plt.savefig(output_image_path, dpi=150)
    print(f"Plot saved to {output_image_path}")

if __name__ == "__main__":
    analyze_and_plot("logs/run_20260921_161100.jsonl", "/home/alok/.gemini/antigravity-ide/brain/7f3e1061-f912-4048-b6f7-08e7f2f0aad9/plot_161100.png")
    analyze_and_plot("logs/run_20260921_161458.jsonl", "/home/alok/.gemini/antigravity-ide/brain/7f3e1061-f912-4048-b6f7-08e7f2f0aad9/plot_161458.png")
