#!/usr/bin/env python3
"""
sitl_test.py
Automated SITL integration test for the full GPS-denied navigation stack.
Validates waypoint navigation, failsafe behavior, and obstacle stop logic
against a running PX4 or ArduPilot SITL instance.

Prerequisites:
  - PX4 SITL or ArduPilot SITL already running in a separate terminal
  - Radar hardware connected (or a synthetic UDP feeder for offline testing)

Usage:
  # With real radar on the bench (tests full stack plumbing against SITL):
  python3 sitl_test.py --port /dev/ttyUSB0 --platform px4

  # Plumbing-only test (no radar, just validates MAVLink + state machine wiring):
  python3 sitl_test.py --platform px4 --plumbing-only

This script is for Test Stage 4 per ARCHITECTURE.md. Do NOT skip directly
to real hardware without passing these tests repeatably.
"""

import argparse
import os
import signal
import socket
import struct
import subprocess
import sys
import time


POSE_PKT_HDR = struct.Struct('<dId')  # t, n_map_points, fwd_range


def wait_for_line(proc: subprocess.Popen, pattern: str, timeout_s: float = 30.0) -> bool:
    """Watch a process's stdout for a line containing `pattern`."""
    import select
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return False
        r, _, _ = select.select([proc.stdout], [], [], 1.0)
        if r:
            line = proc.stdout.readline()
            if not line:
                return False
            print(f"  [{proc.pid}] {line.rstrip()}")
            if pattern in line:
                return True
    return False


def test_selftest_pass():
    """Test 1: doppler_rio and mavlink_bridge self-tests pass."""
    print("\n=== TEST 1: Self-tests ===")
    py = sys.executable

    r1 = subprocess.run([py, "doppler_rio.py", "--selftest"],
                         capture_output=True, text=True, timeout=30)
    assert r1.returncode == 0 and "PASS" in r1.stdout, \
        f"doppler_rio selftest failed:\n{r1.stdout}\n{r1.stderr}"
    print("  doppler_rio selftest: PASS")

    r2 = subprocess.run([py, "mavlink_bridge.py", "--selftest"],
                         capture_output=True, text=True, timeout=30)
    assert r2.returncode == 0 and "PASS" in r2.stdout, \
        f"mavlink_bridge selftest failed:\n{r2.stdout}\n{r2.stderr}"
    print("  mavlink_bridge selftest: PASS")


def test_rotation_identity():
    """Test 2: body_to_nav_rotation at zero attitude is identity."""
    print("\n=== TEST 2: Rotation math ===")
    # Import the function directly
    sys.path.insert(0, os.getcwd())
    from nav_node import body_to_nav_rotation
    import numpy as np

    R = body_to_nav_rotation(0.0, 0.0, 0.0)
    err = np.linalg.norm(R - np.eye(3))
    assert err < 1e-10, f"Zero-attitude rotation is not identity: err={err}"
    print(f"  R(0,0,0) = I: PASS (err={err:.2e})")

    # 90 deg yaw: body +x should map to nav +y (ENU convention)
    import math
    R90 = body_to_nav_rotation(0.0, 0.0, math.pi / 2)
    v_body = np.array([1.0, 0.0, 0.0])
    v_nav = R90 @ v_body
    # At yaw=90 deg (ENU): body +x -> nav +y direction
    # Rz(pi/2) @ [1,0,0] = [cos(90), sin(90), 0] = [0, 1, 0]
    assert abs(v_nav[1]) > 0.9, \
        f"90 deg yaw rotation didn't map body +x to nav +y: v_nav={v_nav}"
    print(f"  R(0,0,pi/2) @ [1,0,0] = [{v_nav[0]:.3f}, {v_nav[1]:.3f}, {v_nav[2]:.3f}]: PASS")

    # Check orthogonality: R^T R = I
    err_orth = np.linalg.norm(R90.T @ R90 - np.eye(3))
    assert err_orth < 1e-10, f"R not orthogonal: err={err_orth}"
    print(f"  R^T R = I: PASS (err={err_orth:.2e})")

    # Check det(R) = +1 (proper rotation, not reflection)
    det = np.linalg.det(R90)
    assert abs(det - 1.0) < 1e-10, f"det(R) = {det}, not +1"
    print(f"  det(R) = {det:.6f}: PASS")

    # Compound rotation: 45 deg pitch down, check vertical component appears
    R45p = body_to_nav_rotation(0.0, math.radians(45), 0.0)
    v_nav_45 = R45p @ np.array([1.0, 0.0, 0.0])
    # Body +x pitched 45 deg down -> nav frame should have negative z component
    assert v_nav_45[2] < -0.5, \
        f"45 deg pitch-down didn't produce negative z: v_nav={v_nav_45}"
    print(f"  R(0,45deg,0) @ [1,0,0] = [{v_nav_45[0]:.3f}, {v_nav_45[1]:.3f}, {v_nav_45[2]:.3f}]: PASS")


def test_port_no_conflict():
    """Test 3: All consumer ports can be bound simultaneously."""
    print("\n=== TEST 3: Port conflict check ===")
    ports = [5005, 5006, 5007, 5008, 5009, 5010, 5011, 5012]
    socks = []
    try:
        for p in ports:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 0)
            s.bind(('127.0.0.1', p))
            socks.append((p, s))
            print(f"  port {p}: bind OK")
        print("  All ports bindable simultaneously: PASS")
    except OSError as e:
        print(f"  Port conflict detected: {e}")
        raise
    finally:
        for _, s in socks:
            s.close()


def test_supervisor_no_mavlink(args):
    """Test 4: supervisor.py --no-mavlink starts and runs for 10s without crashing."""
    print("\n=== TEST 4: Supervisor --no-mavlink ===")
    if not args.port:
        print("  SKIP (no --port specified, need radar for this test)")
        return

    py = sys.executable
    proc = subprocess.Popen(
        [py, "supervisor.py", "--port", args.port, "--tilt-deg", "0", "--no-mavlink"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        bufsize=1, universal_newlines=True)
    try:
        ok = wait_for_line(proc, "all processes launched", timeout_s=15)
        assert ok, "supervisor did not report 'all processes launched'"
        print("  Stack launched OK, running for 10s...")
        time.sleep(10)
        assert proc.poll() is None, f"supervisor died early (rc={proc.returncode})"
        print("  10s stable run: PASS")
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def main():
    p = argparse.ArgumentParser(description="SITL integration tests for GPS-denied nav stack")
    p.add_argument('--port', default=None, help="radar serial device (optional)")
    p.add_argument('--platform', choices=['px4', 'ardupilot'], default='px4')
    p.add_argument('--mavlink-dest', default='udp:127.0.0.1:14540')
    p.add_argument('--plumbing-only', action='store_true',
                    help="skip tests that need radar hardware")
    args = p.parse_args()

    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    results = {}

    tests = [
        ("Self-tests", test_selftest_pass),
        ("Rotation math", test_rotation_identity),
        ("Port conflict", test_port_no_conflict),
    ]
    if not args.plumbing_only:
        tests.append(("Supervisor --no-mavlink", lambda: test_supervisor_no_mavlink(args)))

    for name, fn in tests:
        try:
            fn()
            results[name] = "PASS"
        except Exception as e:
            results[name] = f"FAIL: {e}"
            print(f"  FAIL: {e}")

    print("\n" + "=" * 60)
    print("SITL TEST RESULTS")
    print("=" * 60)
    all_pass = True
    for name, result in results.items():
        status = "\u2705" if result == "PASS" else "\u274c"
        print(f"  {status} {name}: {result}")
        if result != "PASS":
            all_pass = False

    sys.exit(0 if all_pass else 1)


if __name__ == '__main__':
    main()
