import numpy as np
import open3d as o3d
import copy

# Global map (target)
target = o3d.geometry.PointCloud()
target.points = o3d.utility.Vector3dVector(np.random.rand(10, 3) * 5)
target_cov = o3d.geometry.PointCloud()
target_cov.points = target.points
target_cov.estimate_covariances(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=3))

# Source (local frame)
T_true = np.eye(4)
T_true[:3, 3] = [2.0, 0.0, 0.0]
source = copy.deepcopy(target)
source.transform(np.linalg.inv(T_true)) # source is local observation
source.estimate_covariances(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=3))

# ICP
T_init = np.eye(4)
res = o3d.pipelines.registration.registration_generalized_icp(
    source, target_cov, 5.0, T_init,
    o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
)
print("Transformation returned by ICP:")
print(res.transformation)
