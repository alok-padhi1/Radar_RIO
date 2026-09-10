import open3d as o3d
import numpy as np
import sys
import glob

def analyze(f):
    try:
        pcd = o3d.io.read_point_cloud(f)
        pts = np.asarray(pcd.points)
        if len(pts) == 0:
            return
        min_b = pts.min(axis=0)
        max_b = pts.max(axis=0)
        span = max_b - min_b
        mean = pts.mean(axis=0)
        std = pts.std(axis=0)
        print(f"--- {f} ---")
        print(f"Points: {len(pts)}")
        print(f"Span: X={span[0]:.1f} Y={span[1]:.1f} Z={span[2]:.1f}")
        print(f"StdDev: X={std[0]:.1f} Y={std[1]:.1f} Z={std[2]:.1f}")
    except Exception as e:
        print(f"Error on {f}: {e}")

for f in sorted(glob.glob('/home/alok/radar/maps/radar_map_20260909_184*.pcd')):
    analyze(f)
