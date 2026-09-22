import json, sys, math
import numpy as np

def analyze(log_file):
    print(f"\n--- Analyzing {log_file} ---")
    gps_v_body_x, gps_v_body_y = [], []
    rio_v_body_x, rio_v_body_y = [], []
    t_gps, t_rio = [], []

    with open(log_file) as f:
        for line in f:
            if not line.strip(): continue
            try:
                d = json.loads(line)
            except:
                continue
            
            # Simplified extraction to get the gist of the velocities
            if d['type'] == 'gps' and 'v_enu' in d and 'hdg_deg' in d:
                # GPS is ENU, hdg is degrees from North
                ve, vn, vu = d['v_enu']
                yaw = math.radians(d['hdg_deg'])
                # Rotate ENU to Body FRD (roughly)
                # North is yaw=0.
                # v_fwd = vn * cos(yaw) + ve * sin(yaw)
                # v_right = ve * cos(yaw) - vn * sin(yaw)
                v_fwd = vn * math.cos(yaw) + ve * math.sin(yaw)
                v_right = ve * math.cos(yaw) - vn * math.sin(yaw)
                
                gps_v_body_x.append(v_fwd)
                gps_v_body_y.append(v_right)
                t_gps.append(d['t_mono'])
                
            elif d['type'] == 'rio' and 'vx' in d:
                vx, vy, vz = d['vx'], d['vy'], d['vz']
                rio_v_body_x.append(vx)
                rio_v_body_y.append(vy)
                t_rio.append(d['t_mono'])

    print(f"GPS points: {len(t_gps)}, RIO points: {len(t_rio)}")
    if len(t_gps) > 0 and len(t_rio) > 0:
        print(f"GPS Fwd Velocity: Mean = {np.mean(gps_v_body_x):.2f}, Min = {np.min(gps_v_body_x):.2f}, Max = {np.max(gps_v_body_x):.2f}")
        print(f"RIO Fwd Velocity: Mean = {np.mean(rio_v_body_x):.2f}, Min = {np.min(rio_v_body_x):.2f}, Max = {np.max(rio_v_body_x):.2f}")
        print(f"GPS Right Velocity: Mean = {np.mean(gps_v_body_y):.2f}, Min = {np.min(gps_v_body_y):.2f}, Max = {np.max(gps_v_body_y):.2f}")
        print(f"RIO Right Velocity: Mean = {np.mean(rio_v_body_y):.2f}, Min = {np.min(rio_v_body_y):.2f}, Max = {np.max(rio_v_body_y):.2f}")

analyze("logs/run_20260921_161458.jsonl")
analyze("logs/run_20260921_161100.jsonl")
