import numpy as np
import open3d as o3d
import copy

source = o3d.geometry.PointCloud()
source.points = o3d.utility.Vector3dVector(np.random.rand(10, 3) * 10) # spread out

T_world = np.eye(4)
T_world[:3, 3] = [5.0, 0.0, 0.0]

target = copy.deepcopy(source)
target.transform(T_world)

# Give it a good initial guess
T_init = np.eye(4)
T_init[:3, 3] = [4.5, 0.0, 0.0]

res = o3d.pipelines.registration.registration_icp(
    source, target, 2.0, T_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)
print("Transformation returned by ICP:")
print(res.transformation)
