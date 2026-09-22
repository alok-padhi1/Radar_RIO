#!/usr/bin/env python3
"""
src/supervisor.py — RIO Stack Master Orchestrator.

Blueprint §33: The supervisor is the definitive entry point.
It implements a dependency-ordered startup sequence, watchdog monitoring,
and orchestrates the health manager.

Startup sequence (§33):
1. Config load
2. Hardware drivers (IMU, Altimeter, Radar)
3. Time synchronization
4. RIO interface (C++)
5. Mapping/SLAM
6. Health Manager
7. MAVLink Output
"""

import argparse
import logging
import os
import signal
import sys
import threading
import time
import numpy as np

# Ensure imports work from the new directory structure
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from config.system import load_config
from drivers.time_sync import TimeSyncManager, TimeSyncConfig
from drivers.u300.adapter import U300Adapter
from drivers.u300.reader import U300Reader
from drivers.altimeter.reader import AltimeterReader
from drivers.cube.raw_imu_reader import RawIMUReader
from estimator.health import NavigationHealthManager, HealthConfig
from autopilot.mavlink_output import MAVLinkOutput, OutputConfig
from estimator.logger import FlightLogger
from estimator.rio_interface import RIOInterface, RIOInterfaceConfig

logger = logging.getLogger("Supervisor")


class StackSupervisor:
    """Master orchestrator for the RIO navigation stack."""

    def __init__(self, config_path: str, disable_mavlink: bool = False):
        self.config_path = config_path
        self.disable_mavlink = disable_mavlink
        self.running = False
        self._watchdog_thread = None
        self._main_loop_thread = None
        self.logger = FlightLogger()

        self.time_sync = None
        self.u300_adapter = None
        self.altimeter_reader = None
        self.imu_reader = None
        self.rio_bridge = None
        self.health_manager = None
        self.mavlink_out = None
        
        # Flight Summary Stats
        self.start_position = None
        self.current_position = None
        self.total_distance_m = 0.0

    def _setup_logging(self):
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s [%(name)s] %(levelname)s: %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )

    def start(self):
        """Execute dependency-ordered startup sequence (§33)."""
        self._setup_logging()
        logger.info("=========================================")
        logger.info(" RIO Stack Supervisor Starting           ")
        logger.info(" Blueprint Architecture Phase 4/5        ")
        logger.info("=========================================")

        # 1. Config Load
        logger.info("[1/7] Loading configuration from system.yaml")
        sys_config = load_config(self.config_path)
        time_cfg = TimeSyncConfig()
        health_cfg = HealthConfig()
        out_cfg = OutputConfig()

        # 2. Start Data Logger
        logger.info("[2/7] Starting flight logger")
        self.logger.start()

        # 3. Hardware Drivers
        logger.info("[3/7] Initializing hardware drivers")
        self.u300_adapter = U300Reader(port='/dev/ttyUSB0', baud=921600, adapter=U300Adapter())
        self.altimeter_reader = AltimeterReader(port='/dev/ttyUSB1', baud=115200)
        self.imu_reader = RawIMUReader()
        self.imu_reader.start()

        # 4. Time Synchronization
        logger.info("[4/7] Initializing time synchronization")
        self.time_sync = TimeSyncManager(time_cfg)

        # 5. Core Adapters & Estimators
        logger.info("[5/7] Initializing ZMQ RIO interface")
        self.rio_bridge = RIOInterface()
        self.rio_bridge.start()
        
        # 6. Health Manager
        logger.info("[6/7] Initializing Navigation Health Manager")
        self.health_manager = NavigationHealthManager(health_cfg)

        # 7. MAVLink Output
        if not self.disable_mavlink:
            logger.info("[7/7] Initializing MAVLink Output")
            self.mavlink_out = MAVLinkOutput(out_cfg)
        else:
            logger.info("[7/7] MAVLink Output DISABLED (Safe Mode)")

        self.running = True
        
        # Start Threads
        self._watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog_thread.start()
        
        self._main_loop_thread = threading.Thread(target=self._main_loop, daemon=True)
        self._main_loop_thread.start()
        
        logger.info("=========================================")
        logger.info(" Stack is RUNNING. Waiting for sensors... ")
        logger.info("=========================================")

    def stop(self):
        """Clean shutdown."""
        logger.info("Initiating shutdown...")
        self.running = False
        if self._watchdog_thread:
            self._watchdog_thread.join(timeout=1.0)
        if self._main_loop_thread:
            self._main_loop_thread.join(timeout=1.0)
            
        if self.start_position is not None and self.current_position is not None:
            displacement = self.current_position - self.start_position
            net_dist = np.linalg.norm(displacement)
            print("\n==================================================")
            print(" FLIGHT SUMMARY (RIO ODOMETRY)")
            print("==================================================")
            print(f" Total Distance Travelled : {self.total_distance_m:.2f} m")
            print(f" Net Displacement         : {net_dist:.2f} m")
            print(f" Displacement X (Fwd)     : {displacement[0]:.2f} m")
            print(f" Displacement Y (Right)   : {displacement[1]:.2f} m")
            print(f" Displacement Z (Down)    : {displacement[2]:.2f} m")
            print("==================================================\n")

        if self.imu_reader:
            self.imu_reader.stop()
        if self.u300_adapter:
            self.u300_adapter.stop()
        if self.altimeter_reader:
            self.altimeter_reader.stop()
        if self.rio_bridge:
            self.rio_bridge.stop()
        if self.logger:
            self.logger.stop()
            
        logger.info("Shutdown complete.")

    def _main_loop(self):
        """Main data pumping loop from drivers to IPC bridge to output."""
        while self.running:
            # 1. Read hardware
            imu_sample = self.imu_reader.latest_raw
            if imu_sample:
                # 2. Sync timestamps & push to C++
                self.rio_bridge.send_imu(imu_sample)
                
            # 2b. Read Radar
            radar_scan = self.u300_adapter.read()
            if radar_scan and len(radar_scan.measurements) > 0:
                self.rio_bridge.send_radar(radar_scan.measurements)
            
            # 2c. Read Altimeter
            altimeter_sample = self.altimeter_reader.read()
            
            # 3. Read latest state from C++ RIO core
            state = self.rio_bridge.get_latest_state()
            if state:
                if altimeter_sample:
                    state.altimeter = altimeter_sample
                    
                # 4. Feed to Health Manager
                if altimeter_sample:
                    self.health_manager.update_altimeter_health(altimeter_sample)
                self.health_manager.update_state_machine(rio=state.rio)
                
                # 5. Populate state from Health Manager
                state.mode = self.health_manager.mode
                state.quality = self.health_manager.quality
                state.quality_reasons = list(self.health_manager._reasons)
                state.health = {
                    'radar': self.health_manager.radar_health.healthy,
                    'imu': self.health_manager.imu_health.healthy,
                    'altimeter': self.health_manager.altimeter_health.healthy,
                    'time_sync': self.health_manager.time_sync_healthy,
                    'observable': self.health_manager.observability.is_fully_observable(),
                    'navigation_valid': self.health_manager.navigation_valid,
                }
                
                # 6. Output to autopilot
                if self.mavlink_out:
                    self.mavlink_out.publish(state)
                
                # Log state
                self.logger.log_navigation_state(state)
                
                # Update Summary Stats
                if state.position is not None:
                    pos = np.array(state.position)
                    if self.start_position is None:
                        self.start_position = pos
                    if self.current_position is not None:
                        self.total_distance_m += float(np.linalg.norm(pos - self.current_position))
                    self.current_position = pos
                
            time.sleep(0.005) # Loop at ~200Hz

    def _watchdog_loop(self):
        """Watchdog monitoring — Blueprint §48."""
        while self.running:
            try:
                time.sleep(1.0)
                if self.health_manager and self.logger:
                    report = self.health_manager.get_quality_report()
                    self.logger.log_health_report(report)
                    
                    if self.imu_reader:
                        self.logger.log_imu_stats(self.imu_reader.get_health())
                        gps = self.imu_reader.latest_gps
                        if gps:
                            self.logger.log_gps(gps)
                    
            except Exception as e:
                logger.error(f"Watchdog error: {e}")


def main():
    parser = argparse.ArgumentParser(description="RIO Stack Supervisor")
    parser.add_argument('--config', default='config/system.yaml', help="Path to system.yaml")
    parser.add_argument('--no-mavlink', action='store_true', help="Disable MAVLink output to flight controller")
    args = parser.parse_args()

    supervisor = StackSupervisor(args.config, disable_mavlink=args.no_mavlink)
    
    def signal_handler(sig, frame):
        supervisor.stop()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        supervisor.start()
        # Keep main thread alive
        while supervisor.running:
            time.sleep(0.5)
    except Exception as e:
        logger.exception("Supervisor crashed")
        supervisor.stop()

if __name__ == '__main__':
    main()
