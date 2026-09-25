# Antigravity GPS-Denied Navigation Stack (Livox Avia + FAST-LIO2)
**Complete Setup and Execution Guide**

This guide contains everything you need to know to take a fresh Ubuntu 24.04 computer, install the necessary dependencies, compile this workspace, and run the passive validation tests using a Livox Avia LiDAR.

---

## 1. Prerequisites & System Requirements
- **OS**: Ubuntu 24.04 (Noble Numbat)
- **Middleware**: ROS 2 Jazzy Jalisco
- **Hardware**: Livox Avia LiDAR

### 1.1 Setting up the Network for Livox
The Livox Avia requires a static IP address on your computer to communicate properly.
1. Connect the Livox Avia to your computer via Ethernet.
2. Open your Network Settings and edit the Wired Connection.
3. Go to the **IPv4 Settings** tab.
4. Change the method from *Automatic (DHCP)* to **Manual**.
5. Add the following configuration:
   - **Address**: `192.168.1.50`
   - **Netmask**: `255.255.255.0`
   - **Gateway**: `192.168.1.1`
6. Apply and reconnect the wired connection.

---

## 2. Installing Dependencies

If you haven't already installed ROS 2 Jazzy, please follow the [Official Ubuntu Install Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html).

Once ROS 2 Jazzy is installed, open a terminal and install the required dependencies for FAST-LIO2 and the custom navigation stack:

```bash
# Update package lists
sudo apt update

# Install ROS 2 build tools and core dependencies
sudo apt install -y python3-colcon-common-extensions python3-rosdep build-essential cmake

# Install Point Cloud Library (PCL), Eigen, and ROS 2 standard messages
sudo apt install -y libpcl-dev ros-jazzy-pcl-conversions ros-jazzy-pcl-ros
sudo apt install -y ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs ros-jazzy-nav-msgs ros-jazzy-tf2-ros ros-jazzy-tf2-eigen

# Initialize and update rosdep
sudo rosdep init
rosdep update
```

---

## 3. Building the Workspace

Once you have extracted the ZIP file (or cloned the repository) into your `~/LIO` directory, you must compile the ROS 2 workspace.

1. Navigate to the ROS 2 workspace directory:
   ```bash
   cd ~/LIO/ros2_ws
   ```

2. Source the global ROS 2 installation:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

3. Build the workspace using `colcon`. (We use `--symlink-install` so that changes to Python files do not require recompiling):
   ```bash
   colcon build --symlink-install
   ```

4. If the build completes successfully, you will see `Summary: X packages finished`.

---

## 4. Running the Passive Validation Trial

Before integrating flight controllers (PX4) or loop closure, we strictly enforce a **Passive Validation Gate**. This means we run the LiDAR and SLAM stack purely as an observer to measure drift, scale, and accuracy over known physical distances.

We have provided a fully automated Python supervisor script (`run_passive_trial.py`) that handles launching all nodes in the background, recording a rosbag, and displaying your live Odometry.

### Step-by-Step Execution:

1. Open a new terminal.
2. Ensure the script is executable:
   ```bash
   cd ~/LIO
   chmod +x run_passive_trial.py
   ```
3. Source your newly built workspace and run the script:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/LIO/ros2_ws/install/setup.bash
   ./run_passive_trial.py
   ```

### What happens next?
- The script will launch the Livox driver, FAST-LIO2, and the custom map backend nodes.
- After 5 seconds, it will automatically start recording a rosbag (saved in `~/LIO/bags/`).
- You will see a live updating line at the bottom of your terminal showing your current LIO position:
  ```text
  🚀 Current LIO Position: X=  0.005 m | Y= -0.012 m | Z=  0.002 m
  ```

---

## 5. Performing the Physical Tests

While the supervisor script is running and printing your position, perform the following physical tests to validate the SLAM geometry:

1. **Stationary Test:** Do not move the sensor for 2 minutes. Ensure the `Z` axis does not drift uncontrollably.
2. **0.5m Scale Test:** Physically move the sensor forward exactly 0.5 meters. Look at the terminal output. If `X` (or `Y`, depending on orientation) reads `1.0m`, the LIO scale is incorrect and must be recalibrated. It should read `~0.5m`.
3. **1.0m Scale Test:** Move the sensor to exactly 1.0 meters. Verify the terminal reads `~1.0m`.
4. **Closed Loop Test:** Walk a 5-meter path and return to the *exact* physical starting location. The terminal should report numbers very close to `0.000` across all axes.

---

## 6. Safely Stopping the Test

When you have finished walking the test patterns, simply press **`CTRL+C`** in the terminal where the script is running.

**The script will automatically:**
1. Stop the rosbag recording cleanly (preventing `.mcap.active` corruption).
2. Shut down the ROS 2 launch processes.
3. Wait 4 seconds to ensure all keyframes and map manifests (`map.csv` / `manifest.yaml`) are fully flushed to disk.
4. Print a summary showing exactly where your saved data is located.

---

## 7. Next Steps

If the scale tests are accurate (e.g. walking 1 meter results in a 1-meter reported odometry change), and the closed-loop endpoint error is low, the baseline stack is validated! 

You can then pass the recorded `.mcap` rosbag files to the backend team to begin testing **Phase D (Loop Closure and GTSAM Pose Graph Optimization)** using offline replay data.
