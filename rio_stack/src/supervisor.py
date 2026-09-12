#!/usr/bin/env python3
"""
supervisor.py
Launches and monitors the full stack as child processes, in the correct
order, with a shared log prefix per process so you can tell which
component printed what. Restarts a crashed non-critical process once, then
gives up and takes the whole stack down rather than limping along with a
dead RIO or SLAM thread feeding stale data into nav_node.

Order matters: radar_fanout.py must be up before doppler_rio.py/slam_node.py
have anything to listen to; mavlink_bridge.py and nav_node.py both need a
live MAVLink heartbeat, which for SITL means PX4/ArduPilot SITL must
already be running before you start this script.

Usage:
  python3 supervisor.py --port /dev/ttyUSB0 --tilt-deg 40 \
      --waypoints "0,10,5;10,10,5;10,0,5;0,0,5" --platform px4

Ctrl+C stops every child cleanly (SIGTERM, then SIGKILL after a grace period).
"""

import argparse
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
import logging

logging.basicConfig(level=logging.INFO, format="[SUPERVISOR] [%(levelname)s] %(message)s")


class Child:
    def __init__(self, name: str, cmd: list[str], critical: bool = True,
                 start_delay_s: float = 0.0):
        self.name = name
        self.cmd = cmd
        self.critical = critical
        self.start_delay_s = start_delay_s
        self.proc: subprocess.Popen | None = None
        self.restarts = 0

    def start(self):
        logging.info(f"starting {self.name}: {' '.join(shlex.quote(c) for c in self.cmd)}")
        # Root-cause #5 of the Stage 1 bug report: without forcing unbuffered
        # stdout, Python block-buffers (4-8KB) when stdout is a pipe rather
        # than a TTY. doppler_rio.py prints 20 lines/sec and fills its buffer
        # fast; slam_node.py and radar_fanout.py print far less often and
        # can sit buffered for many seconds, making it look like SLAM/fanout
        # "aren't logging" when they're just waiting to flush. Two belts:
        # `-u` on the interpreter itself, AND PYTHONUNBUFFERED=1 in the
        # child's environment in case `python3 -u` isn't the literal
        # executable being invoked (e.g. a wrapper script).
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        cmd = self.cmd
        if len(cmd) >= 2 and cmd[1] != "-u":
            cmd = [cmd[0], "-u"] + cmd[1:]
        self.proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            bufsize=1, universal_newlines=True, env=env)
        threading.Thread(target=self._pump_output, daemon=True).start()

    def _pump_output(self):
        assert self.proc and self.proc.stdout
        for line in self.proc.stdout:
            # Assumes the child script outputs: [INFO] message
            print(f"[{self.name.upper()}] {line.rstrip()}")

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def stop(self, grace_s: float = 3.0):
        if self.proc is None or self.proc.poll() is not None:
            return
        self.proc.terminate()
        try:
            self.proc.wait(timeout=grace_s)
        except subprocess.TimeoutExpired:
            self.proc.kill()


def build_children(args) -> list[Child]:
    py = sys.executable

    # Build the RIO velocity fan-out destination list. Each downstream consumer
    # that needs RIO velocity gets its own dedicated port to avoid the triple-bind
    # conflict that existed when all three tried to bind() on 5006.
    #   5006 -> slam_node.py      (always, for deskew + Doppler prior)
    #   5007 -> mavlink_bridge.py (when MAVLink is enabled)
    #   5008 -> nav_node.py       (when navigation is enabled)
    rio_forward_ports = ["5006", "5009"]  # 5006 -> slam_node, 5009 -> visualizer
    if not args.no_mavlink:
        rio_forward_ports.append("5007")
    if args.enable_nav:
        rio_forward_ports.append("5008")
    if args.gps_port:
        rio_forward_ports.append("5013")  # 5013 -> gps_logger

    # SLAM pose destination ports (comma-separated, parsed by slam_node.py)
    slam_pose_ports = ["5011"]   # default: nav_node / visualizer
    if args.gps_port:
        slam_pose_ports.append("5014")  # 5014 -> gps_logger

    children = [
        Child("fanout", [py, "radar_fanout.py",
                          *(["--port", args.port] if args.port else []),
                          "--slam-decimation", str(args.slam_decimation)],
              critical=True),
        Child("rio", [py, "doppler_rio.py",
                       "--listen-port", "5005",
                       "--forward-ports", ",".join(rio_forward_ports),
                       "--theta-tilt-deg", str(args.tilt_deg),
                       "--lateral-sign", str(args.lateral_sign),
                       "--eps", str(args.eps),
                       "--min-inlier-ratio", str(args.min_inlier_ratio),
                       "--cond-reject-threshold", str(args.cond_reject_threshold),
                       "--deadband", str(args.deadband)]
                       + (["--imu-port", "5020"] if args.imu_port else []),
              critical=True, start_delay_s=1.0),
        Child("slam", [py, "slam_node.py",
                        "--listen-port", "5010",
                        "--rio-port", "5006",
                        "--pose-port", ",".join(slam_pose_ports),
                        "--theta-tilt-deg", str(args.tilt_deg),
                        "--lateral-sign", str(args.lateral_sign),
                        "--voxel-size", str(args.voxel_size),
                        "--max-corr-dist", str(args.max_corr_dist),
                        "--min-correspondences", str(args.min_correspondences),
                        "--persistence-radius", str(args.persistence_radius),
                        "--persistence-min-hits", str(args.persistence_min_hits),
                        "--window-s", str(args.window_s)]
                        + (["--save-pcd", args.save_pcd] if args.save_pcd else [])
                        + (["--imu-port", "5021"] if args.imu_port else []),
              critical=True, start_delay_s=1.0),
    ]
    if not args.no_mavlink:
        children.append(Child(
            "mavlink_bridge", [py, "mavlink_bridge.py",
                                "--autopilot", args.platform,
                                "--mavlink-dest", args.mavlink_dest,
                                "--rio-port", "5007"],
            critical=False, start_delay_s=2.0))
    if args.enable_nav:
        if not args.waypoints:
            logging.error("--enable-nav requires --waypoints; refusing to start nav_node.")
        else:
            children.append(Child(
                "nav", [py, "nav_node.py",
                        "--mavlink-dest", args.mavlink_dest,
                        "--platform", args.platform,
                        "--waypoints", args.waypoints,
                        "--pose-port", "5011", "--rio-port", "5008"],
                critical=False, start_delay_s=3.0))
    if args.visualizer or args.visualizer_no_gui:
        vis_cmd = [py, "visualizer_3d.py", "--tilt-deg", str(args.tilt_deg),
                   "--lateral-sign", str(args.lateral_sign)]
        if args.visualizer_no_gui:
            vis_cmd.append("--no-gui")
        children.append(Child("vis", vis_cmd, critical=False, start_delay_s=1.5))
    if args.gps_port:
        log_dir = args.log_dir or 'logs'
        children.append(Child(
            "gps", [py, "gps_logger.py",
                    "--gps-serial", args.gps_port,
                    "--gps-baud", str(args.gps_baud),
                    "--log-dir", log_dir,
                    "--rio-port", "5013",
                    "--pose-port", "5014"]
                    + (["--imu-port", "5022"] if args.imu_port else []),
            critical=False, start_delay_s=2.0))
    if args.imu_port:
        imu_dest_ports = ["5020", "5021"]
        if args.gps_port:
            imu_dest_ports.append("5022")
        children.append(Child(
            "imu", [py, "imu_bridge.py",
                    "--port", args.imu_port,
                    "--baud", str(args.imu_baud),
                    "--dest-ports", ",".join(imu_dest_ports)],
            critical=False, start_delay_s=0.5))
    return children


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', default=None, help="serial device for radar_fanout.py")
    p.add_argument('--tilt-deg', type=float, default=40.0)
    p.add_argument('--lateral-sign', type=float, default=1.0, choices=[1.0, -1.0],
                    help="passed to both rio and slam -- see doppler_rio.py's TiltMount "
                         "docstring for the bench validation procedure")
    p.add_argument('--slam-decimation', type=int, default=1)
    # Bench/Stage-1 scale knobs (see FIELD_RUNBOOK.md / STAGE1_VERIFICATION.md).
    # Defaults here match production/flight scale -- override for a close-range
    # bench setup, e.g.: --voxel-size 0.10 --max-corr-dist 0.5
    # --min-correspondences 6 --persistence-radius 0.15 --deadband 0.05
    p.add_argument('--voxel-size', type=float, default=1.5)
    p.add_argument('--max-corr-dist', type=float, default=6.0)
    p.add_argument('--min-correspondences', type=int, default=15)
    p.add_argument('--persistence-radius', type=float, default=1.0)
    p.add_argument('--persistence-min-hits', type=int, default=2)
    p.add_argument('--window-s', type=float, default=0.3)
    p.add_argument('--eps', type=float, default=0.20, help="doppler_rio.py Doppler tolerance m/s")
    p.add_argument('--min-inlier-ratio', type=float, default=0.25, help="doppler_rio.py minimum inlier ratio")
    p.add_argument('--cond-reject-threshold', type=float, default=12.0, help="doppler_rio.py condition number reject threshold")
    p.add_argument('--deadband', type=float, default=0.05, help="doppler_rio.py m/s deadband")
    p.add_argument('--platform', choices=['px4', 'ardupilot'], default='px4')
    p.add_argument('--mavlink-dest', default='udp:127.0.0.1:14540')
    p.add_argument('--no-mavlink', action='store_true',
                    help="run the radar/SLAM stack only, no MAVLink bridge -- "
                         "use this for bench/Phase 1-2 testing per PIPELINE_NOTES.md")
    p.add_argument('--enable-nav', action='store_true',
                    help="also launch nav_node.py -- DO NOT set this until "
                         "Test Stage 4+ per ARCHITECTURE.md; nav_node.py arms "
                         "and commands the vehicle")
    p.add_argument('--waypoints', default=None)
    p.add_argument('--visualizer', action='store_true',
                    help="also launch visualizer_3d.py for real-time 3D trajectory & map display")
    p.add_argument('--visualizer-no-gui', action='store_true',
                    help="run visualizer in terminal HUD mode only (no GUI window)")
    p.add_argument('--save-pcd', default=None,
                    help="Save accumulated SLAM map as .pcd on exit. Pass a filepath or directory.")
    p.add_argument('--gps-port', default=None,
                    help="Serial port for GPS NMEA module (e.g. /dev/ttyUSB1). "
                         "Enables GPS ground-truth logging via gps_logger.py.")
    p.add_argument('--gps-baud', type=int, default=9600,
                    help="GPS serial baud rate (NEO-M8N default: 9600)")
    p.add_argument('--log-dir', default=None,
                    help="Directory for GPS+radar JSONL logs (default: logs/)")
    p.add_argument('--imu-port', default=None,
                    help="Serial port for IMU/FC (e.g. /dev/ttyACM0). "
                         "Enables IMU rotation compensation via imu_bridge.py.")
    p.add_argument('--imu-baud', type=int, default=115200,
                    help="IMU/FC serial baud rate (Cube Orange USB default: 115200)")
    args = p.parse_args()

    children = build_children(args)
    stop_flag = threading.Event()

    def handle_sigint(signum, frame):
        logging.info("shutting down...")
        stop_flag.set()

    signal.signal(signal.SIGINT, handle_sigint)

    for c in children:
        if c.start_delay_s:
            time.sleep(c.start_delay_s)
        c.start()

    logging.info("all processes launched. Ctrl+C to stop the stack.")
    try:
        while not stop_flag.is_set():
            time.sleep(1.0)
            for c in children:
                if not c.alive():
                    if c.critical:
                        logging.error(f"CRITICAL process '{c.name}' died -- "
                                      f"stopping the whole stack rather than running degraded.")
                        stop_flag.set()
                        break
                    elif c.restarts < 1:
                        logging.warning(f"non-critical '{c.name}' died, restarting once.")
                        c.restarts += 1
                        c.start()
                    else:
                        logging.error(f"'{c.name}' died again, not restarting further. "
                              f"Fix and restart the supervisor.")
    finally:
        for c in reversed(children):
            c.stop()
        logging.info("all processes stopped.")


if __name__ == '__main__':
    main()
