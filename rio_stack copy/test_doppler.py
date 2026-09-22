import numpy as np
from estimator.u300_velocity.doppler import RobustDopplerEstimator

def test_doppler():
    # True velocity
    v_true = np.array([1.5, -0.5, 0.2])
    
    # Generate points (direction vectors)
    num_points = 10
    points = np.random.randn(num_points, 3)
    points /= np.linalg.norm(points, axis=1)[:, None]
    points *= np.random.uniform(2.0, 10.0, size=(num_points, 1)) # ranges between 2 and 10
    
    # Measurements (d_i = -u_i^T * v_R)
    ranges = np.linalg.norm(points, axis=1)
    dirs = points / ranges[:, None]
    
    dopplers = -np.sum(dirs * v_true, axis=1)
    
    # Add noise
    dopplers += np.random.normal(0, 0.05, size=num_points)
    
    # Add an outlier
    dopplers[0] += 5.0
    
    estimator = RobustDopplerEstimator()
    res = estimator.estimate(points, dopplers)
    
    print("Test Doppler Velocity")
    print(f"True Velocity: {v_true}")
    print(f"Est Velocity : {res.velocity}")
    print(f"Valid: {res.is_valid}, Inliers: {res.num_points_used}, Reason: {res.reason}")
    assert res.is_valid
    assert np.allclose(res.velocity, v_true, atol=0.2)
    print("Pass!")

if __name__ == "__main__":
    test_doppler()
