import numpy as np
import open3d as o3d
import copy

# Simulate a map at X=0, 1, 2, 3...
map_cloud = o3d.geometry.PointCloud()
for i in range(5):
    pts = np.random.rand(10, 3) + np.array([i, 0, 0])
    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(pts)
    map_cloud += p
map_cloud.estimate_covariances(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=3))

# Current frame is at X=5.
source = o3d.geometry.PointCloud()
source.points = o3d.utility.Vector3dVector(np.random.rand(10, 3)) # local frame
source_global = copy.deepcopy(source)
source_global.transform([[1,0,0,5], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
# Add it to map to ensure a perfect match exists at X=5
map_cloud += source_global

source.estimate_covariances(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=3))

# Buggy SLAM code from slam_node.py
T_world = np.eye(4)
T_world[0, 3] = 4.0 # Previous pose is at X=4

# T_init is just the relative step (v * dt)
T_init_rel = np.eye(4)
T_init_rel[0, 3] = 1.0 # 1 meter forward

# Run ICP with target=global map, source=local frame, init=relative step
res = o3d.pipelines.registration.registration_generalized_icp(
    source, map_cloud, 2.0, T_init_rel,
    o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
)
print("ICP returned T_step:")
print(res.transformation)

# Buggy update
T_world_buggy = T_world @ res.transformation
print("\nBuggy T_world (T_world @ T_step):")
print(T_world_buggy)
