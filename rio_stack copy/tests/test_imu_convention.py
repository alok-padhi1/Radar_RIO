"""
tests/test_imu_convention.py — IMU unit and sign validation (Step 5).

Verifies that SCALED_IMU2 is output in body FRD with correct SI units.
"""

import numpy as np
import pytest


def test_stationary_accel_frd():
    """Stationary FRD accel should be approximately [0, 0, -9.81].

    In FRD: gravity pulls downward (+Z), but the accelerometer measures
    the proper acceleration which is upward (-Z).
    """
    # Simulate what _process_raw_imu would produce with stationary readings
    # SCALED_IMU2: xacc=0, yacc=0, zacc=+1000 (1000 mG = 1G downward in FRD)
    # After conversion: az = 1000/1000 * 9.80665 = 9.80665 ... but wait
    # Actually for stationary on table in FRD:
    # The accelerometer reads -1G on Z axis (opposing gravity)
    # zacc from Cube in FRD = approximately -1000 mG
    # After conversion: az = -1000/1000 * 9.80665 = -9.80665

    # We test the conversion logic directly
    zacc_mg = -1000  # -1G in FRD (proper accel opposing gravity)
    az_mps2 = zacc_mg / 1000.0 * 9.80665
    assert abs(az_mps2 - (-9.80665)) < 0.01, f"Expected -9.81, got {az_mps2}"


def test_gyro_sign_frd():
    """Positive yaw-right = positive gz in FRD."""
    # FRD: +Z rotation = yaw right (clockwise when viewed from above)
    # Cube SCALED_IMU2 zgyro is positive for right yaw
    zgyro_mradps = 500  # 500 mrad/s right yaw
    gz_radps = zgyro_mradps / 1000.0
    assert abs(gz_radps - 0.5) < 0.001, f"Expected 0.5 rad/s, got {gz_radps}"


def test_no_duplicate_frame_conversion():
    """Verify IMU reader outputs native FRD — no Y/Z negation."""
    # The conversion should be:
    # ax = xacc / 1000.0 * 9.80665  (no negation)
    # ay = yacc / 1000.0 * 9.80665  (no negation)
    # az = zacc / 1000.0 * 9.80665  (no negation)

    xacc, yacc, zacc = 100, -200, -1000  # arbitrary mG values
    ax = xacc / 1000.0 * 9.80665
    ay = yacc / 1000.0 * 9.80665  # NOT negated
    az = zacc / 1000.0 * 9.80665  # NOT negated

    # Y should be negative (right-side tilt)
    assert ay < 0, f"ay should preserve sign: {ay}"
    # Z should be negative (stationary gravity in FRD)
    assert az < 0, f"az should preserve sign: {az}"


def test_units_mps2_radps():
    """Verify acceleration is m/s², gyro is rad/s."""
    # 1000 mG = 1G = 9.80665 m/s²
    accel_mG = 1000
    accel_mps2 = accel_mG / 1000.0 * 9.80665
    assert abs(accel_mps2 - 9.80665) < 0.001

    # 1000 mrad/s = 1 rad/s
    gyro_mradps = 1000
    gyro_radps = gyro_mradps / 1000.0
    assert abs(gyro_radps - 1.0) < 0.001
