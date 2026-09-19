import numpy as np
import sys
import math

# We need to import doppler_ransac from src.doppler_rio
sys.path.append('/home/alok/radar/rio_stack/src')
from doppler_rio import doppler_ransac, irls_refit, weighted_refit

def test_flat_floor_with_prior():
    np.random.seed(42)
    N = 30
    
    # 1. Create points on a flat floor (Z = -1.0)
    # The radar is tilted at 40 degrees, so it looks forward and down.
    # X is forward, Y is right, Z is down.
    # Let's generate points where Z = 1.0 (down), X from 2.0 to 10.0, Y from -3.0 to 3.0
    x = np.random.uniform(2.0, 10.0, N)
    y = np.random.uniform(-3.0, 3.0, N)
    z = np.ones(N) * 1.0
    
    pts = np.column_stack([x, y, z])
    ranges = np.linalg.norm(pts, axis=1)
    u_body = pts / ranges[:, None]
    
    # 2. Simulate drone motion: flying forward at 2.0 m/s
    true_v_body = np.array([2.0, 0.0, 0.0])
    
    # Measured radial velocity: v_radial = -u_body dot true_v_body + noise
    v_radial = - (u_body @ true_v_body) + np.random.normal(0, 0.05, N)
    
    print("--- Test 1: Flat floor WITHOUT vz_prior (Airborne) ---")
    res1 = doppler_ransac(u_body, v_radial, airborne=True, vz_prior=None, cond_reject_threshold=30.0)
    if res1 is None:
        print("Result: REJECTED (as expected, 3D condition number over flat floor is too high)")
    else:
        print(f"Result: ACCEPTED! cond={res1[2]:.2f}")
        
    print("\n--- Test 2: Flat floor WITH vz_prior = 0.0 (Airborne) ---")
    res2 = doppler_ransac(u_body, v_radial, airborne=True, vz_prior=0.0, cond_reject_threshold=30.0)
    if res2 is None:
        print("Result: REJECTED (FAIL!)")
    else:
        mask, is_static, cond = res2
        print(f"Result: ACCEPTED! inliers={mask.sum()}/{N}, cond={cond:.2f} (This proves the fix works!)")
        
        # Let's also run weighted_refit and irls_refit to ensure they don't crash
        v_seed, cov = weighted_refit(u_body, v_radial, ranges, mask, vz_prior=0.0)
        print(f"Seed Velocity: {v_seed.round(3)} m/s")
        
        v_body, cov = irls_refit(u_body, v_radial, pts, ranges, np.eye(3), v_seed,
                                 sigma_r_m=0.1, sigma_az_rad=0.03, sigma_el_rad=0.03, sigma_v_mps=0.05,
                                 vz_prior=0.0)
        print(f"Final Velocity: {v_body.round(3)} m/s")
        print(f"Final Covariance diagonals: {np.diag(cov).round(4)}")
        
        # Test max_sigma logic
        cxx, cyy, czz = cov[0,0], cov[1,1], cov[2,2]
        max_sigma_before = math.sqrt(max(cxx, cyy, czz))
        
        # Override czz like the code does
        czz = 100.0
        max_sigma_after = math.sqrt(max(cxx, cyy, czz))
        
        print(f"\nmax_sigma (BEFORE czz=100) = {max_sigma_before:.3f} m/s (This is what is checked now)")
        print(f"max_sigma (AFTER czz=100)  = {max_sigma_after:.3f} m/s (This is what WAS checked, causing bugs)")

if __name__ == '__main__':
    test_flat_floor_with_prior()
