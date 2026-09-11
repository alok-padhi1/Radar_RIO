# RIO Radar Workspace Restructuring Plan

## Goal
Rename the awkwardly named `testing RIO+3d` folder and organize its contents into a clean, logical structure separating documentation, tests, analysis tools, and core system nodes.

## Proposed Directory Structure

We will rename `testing RIO+3d` to `rio_stack` and organize it as follows:

```text
Radar_RIO/
└── rio_stack/                   <-- (Renamed from 'testing RIO+3d')
    ├── docs/                    <-- Documentation
    │   ├── ARCHITECTURE.md
    │   ├── FIELD_RUNBOOK.md
    │   ├── NAVIGATION.md
    │   ├── PIPELINE_NOTES.md
    │   ├── STAGE1_VERIFICATION.md
    │   ├── STAGE_LOGBOOK.md
    │   └── SYSTEM_REFERENCE_AND_PHASE_GUIDE.md
    │
    ├── tools/                   <-- Offline analysis and utility scripts
    │   ├── analyze_run.py
    │   ├── analyze_drift.py
    │   └── analyze_pcd_shape.py
    │
    ├── tests/                   <-- Automated tests and debug scripts
    │   ├── sitl_test.py
    │   ├── test_gicp.py
    │   ├── test_slam_bug.py
    │   └── test_slam_icp.py
    │
    ├── logs/                    <-- Output directory (Git-ignored)
    │   └── .gitkeep
    │
    └── src/                     <-- Core radar pipeline & ROS-like nodes
        ├── supervisor.py        <-- Main stack entrypoint
        ├── radar_fanout.py
        ├── doppler_rio.py
        ├── slam_node.py
        ├── nav_node.py
        ├── filters.py           <-- Shared by slam_node and doppler_rio
        ├── mavlink_bridge.py
        ├── visualizer_3d.py
        └── gps_logger.py
```

## Required Code Changes (Imports & Paths)

When we move files into different folders, internal imports and relative paths will break. I will fix the following:

1. **`supervisor.py` Path Updates**:
   Currently, `supervisor.py` expects all node scripts (`doppler_rio.py`, etc.) to be in the same folder. Since they will all be inside `src/`, `supervisor.py` can remain the main entrypoint and launch them directly from the same `src/` directory.

2. **Test Imports**:
   Scripts in `tests/` (like `sitl_test.py` and `test_slam_regression.py`) import functions from the core nodes. I will add a path modification (`sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))`) to the test files so they can cleanly import from the `src/` directory without needing a complex Python package installation.

3. **Workspace References**:
   I will do a quick scan to ensure there are no hardcoded references to `testing RIO+3d` in the codebase or documentation.

## Execution Steps

1. Rename the directory `Radar_RIO/testing RIO+3d` to `Radar_RIO/rio_stack`
2. Create the `docs/`, `tools/`, `tests/`, and `src/` subdirectories.
3. Move the respective files into their new homes.
4. Update imports in the `tests/` files so they can find the core code in `src/`.
5. Run a syntax and basic execution check to ensure `supervisor.py` and `sitl_test.py` still function properly.
