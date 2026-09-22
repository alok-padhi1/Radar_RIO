import json
import math
import sys
import glob
import os

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2) * math.sin(dlam/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def process_log(filepath):
    gps_pts = []
    slam_pts = []
    rio_pts = []
    
    with open(filepath, 'r') as f:
        for line in f:
            if not line.strip(): continue
            try:
                d = json.loads(line)
                typ = d.get('type')
                if typ == 'gps':
                    gps_pts.append(d)
                elif typ == 'slam':
                    slam_pts.append(d)
                elif typ == 'rio' or typ == 'rio_frame':
                    # Sometimes RIO packets are under 'rio_frame', sometimes 'rio'
                    if 'valid' not in d or d['valid']:
                        rio_pts.append(d)
            except:
                pass

    gps_dist = 0.0
    for i in range(1, len(gps_pts)):
        gps_dist += haversine(gps_pts[i-1]['lat'], gps_pts[i-1]['lon'], gps_pts[i]['lat'], gps_pts[i]['lon'])

    slam_xy = 0.0
    slam_z = 0.0
    if len(slam_pts) > 0:
        p0 = slam_pts[0]['pos']
        p1 = slam_pts[-1]['pos']
        slam_xy = math.sqrt((p1[0]-p0[0])**2 + (p1[1]-p0[1])**2)
        slam_z = p1[2] - p0[2]

    rio_xy = 0.0
    rio_z = 0.0
    if len(rio_pts) > 1:
        rx, ry, rz = 0.0, 0.0, 0.0
        for i in range(1, len(rio_pts)):
            dt = rio_pts[i]['t'] - rio_pts[i-1]['t'] if 't' in rio_pts[i] else rio_pts[i].get('t_mono', 0) - rio_pts[i-1].get('t_mono', 0)
            if 0 < dt < 1.0:
                # v_body or vx,vy,vz
                if 'v_body' in rio_pts[i]:
                    vx, vy, vz = rio_pts[i]['v_body']
                else:
                    vx, vy, vz = rio_pts[i].get('vx', 0), rio_pts[i].get('vy', 0), rio_pts[i].get('vz', 0)
                rx += vx * dt
                ry += vy * dt
                rz += vz * dt
        rio_xy = math.sqrt(rx**2 + ry**2)
        rio_z = rz

    return {
        'name': os.path.basename(filepath),
        'gps_dist': gps_dist,
        'rio_xy': rio_xy,
        'rio_z': rio_z,
        'slam_xy': slam_xy,
        'slam_z': slam_z,
        'frames_gps': len(gps_pts),
        'frames_rio': len(rio_pts),
        'frames_slam': len(slam_pts)
    }

if __name__ == "__main__":
    logs = sys.argv[1:]
    if not logs:
        logs = glob.glob('logs/*.jsonl')
    
    print("| Log File | GPS Path (m) | RIO XY Disp (m) | SLAM XY Disp (m) | SLAM Z Disp (m) | RIO Pkts | SLAM Pkts |")
    print("|----------|-------------:|----------------:|-----------------:|----------------:|---------:|----------:|")
    
    for l in sorted(logs):
        if not os.path.exists(l):
            continue
        res = process_log(l)
        print(f"| `{res['name']}` | {res['gps_dist']:.1f} | {res['rio_xy']:.1f} | {res['slam_xy']:.1f} | {res['slam_z']:.1f} | {res['frames_rio']} | {res['frames_slam']} |")
