# Comprehensive Guide: Livox Avia + FAST-LIO2 on ROS 2 Jazzy

This guidebook covers the complete, end-to-end process of setting up a Livox Avia LiDAR with FAST-LIO2 SLAM on a fresh **Ubuntu 24.04 (Noble)** system running **ROS 2 Jazzy**. 

It includes all necessary bug fixes, C++ source patches, configuration tweaks for high-density mapping, and instructions for saving and visualizing the final 3D point cloud.

---

## 1. System Requirements & Dependencies

Ensure you have ROS 2 Jazzy installed. You will also need standard build tools, Git, and the PCL ROS package.

```bash
# Update system and install required ROS 2 packages and build tools
sudo apt-get update
sudo apt-get install -y build-essential cmake git
sudo apt-get install -y ros-jazzy-pcl-ros
```

---

## 2. Installing the Livox-SDK (With GCC 13 Fixes)

The standard Livox-SDK fails to build on Ubuntu 24.04 due to strict GCC 13 compiler warnings and missing includes. It also needs to be compiled with Position Independent Code (`-fPIC`) to avoid `ld` relocation errors when ROS 2 tries to link it.

**1. Clone the SDK:**
```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
```

**2. Patch the Source Code:**
Open `sdk_core/src/base/thread_base.h` and add the `<memory>` header near the top:
```cpp
#include <atomic>
#include <thread>
#include <memory>  // <-- Add this line
#include "noncopyable.h"
```

Open `sdk_core/CMakeLists.txt` and remove `-Werror` from the compiler flags around line 30:
```cmake
# Change this:
# PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -Werror -Wno-c++11-long-long>
# To this:
PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -Wno-c++11-long-long>
```

**3. Build and Install:**
```bash
mkdir build && cd build
cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON
make
sudo make install
```

---

## 3. Setting Up the ROS 2 Workspace

Create your workspace and clone the community-supported Avia driver and FAST-LIO2 forks that support modern ROS 2 distributions.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the Avia driver (ASIG-X fork)
git clone https://github.com/ASIG-X/livox_ros2_avia.git

# Clone FAST-LIO2 and its required submodules
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git
cd FAST_LIO_ROS2
git submodule update --init --recursive
```

---

## 4. Patching FAST-LIO2 for the Avia Driver

By default, the `FAST_LIO_ROS2` package hardcodes its message dependencies to `livox_ros_driver2`. However, our driver (`livox_ros2_avia`) generates its custom messages inside the `livox_interfaces` package. 

You must patch the FAST-LIO2 source to look for `livox_interfaces`.

**1. Replace dependencies in build files:**
Run this in the terminal to quickly swap the names in `package.xml` and `CMakeLists.txt`:
```bash
sed -i 's/livox_ros_driver2/livox_interfaces/g' ~/ros2_ws/src/FAST_LIO_ROS2/package.xml ~/ros2_ws/src/FAST_LIO_ROS2/CMakeLists.txt
```

**2. Update C++ Headers and Namespaces:**
Run this command to globally replace the old namespace and header paths in the C++ source code:
```bash
find ~/ros2_ws/src/FAST_LIO_ROS2 -type f -name "*.cpp" -o -name "*.h" -o -name "*.hpp" | xargs sed -i 's/livox_ros2_avia\/msg\/custom_msg.hpp/livox_interfaces\/msg\/custom_msg.hpp/g'
find ~/ros2_ws/src/FAST_LIO_ROS2 -type f -name "*.cpp" -o -name "*.h" -o -name "*.hpp" | xargs sed -i 's/livox_ros_driver2::msg::CustomMsg/livox_interfaces::msg::CustomMsg/g'
```

---

## 5. Configuring High-Density Mapping & Connectivity

### Fix LiDAR Connection (Whitelist)
By default, the Livox driver ignores your LiDAR unless its serial number is whitelisted.
Open `~/ros2_ws/src/livox_ros2_avia/livox_ros2_avia/config/livox_lidar_config.json` and replace the default `broadcast_code` (`3JEDLB100127651`) with your exact LiDAR's Broadcast Code (e.g., `3JEDP1N00161681`).

### Tune FAST-LIO2 for High Resolution
FAST-LIO2 defaults to a very sparse configuration for weak computers. To get a dense, beautiful point cloud, edit `~/ros2_ws/src/FAST_LIO_ROS2/config/avia.yaml` and modify the following parameters:

```yaml
# Change downsampling to 10cm (0.1) instead of 50cm (0.5)
filter_size_surf: 0.2
filter_size_map: 0.1

# Use 100% of points instead of 33%
point_filter_num: 1

# Set the map output path
map_file_path: "./scans.pcd"

publish:
    scan_publish_en: true
    map_en: true         # CRITICAL: This MUST be true or the map won't accumulate in memory!
    dense_publish_en: true
```

---

## 6. Building the Workspace

With everything patched and configured, build the workspace.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

---

## 7. Running the SLAM Pipeline

To run the pipeline, use three separate terminals.

> [!NOTE]
> Make sure your computer's Ethernet adapter is manually configured with a static IP in the Livox subnet (e.g., IP: `192.168.1.50`, Netmask: `255.255.255.0`) so it can communicate with the LiDAR.

**Terminal 1: Start the LiDAR Driver**
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch livox_ros2_avia livox_lidar_msg_launch.py
```
*(Wait until it says "Livox-SDK init success!" and begins streaming data)*

**Terminal 2: Start FAST-LIO2 Mapping**
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml
```
*(RViz will open automatically and you will see the map actively building as you move the sensor)*

---

## 8. Saving and Visualizing the Map

> [!WARNING]
> Do NOT simply press `Ctrl+C` to close FAST-LIO2, or you will lose your map data! You must trigger the save service while the node is still running.

**Terminal 3: Trigger Map Save**
When you are satisfied with your scan, open a third terminal and run:
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call /map_save std_srvs/srv/Trigger
```
Once it says "Map saved", you can safely `Ctrl+C` in Terminals 1 and 2 to close everything.

### Visualizing with Elevation Colorization
For a beautiful, high-quality rendering (colored by height/Z-axis):

1. Install CloudCompare:
   ```bash
   sudo snap install cloudcompare
   ```
2. Open CloudCompare and drag-and-drop your `~/ros2_ws/scans.pcd` file into the window.
3. Select the point cloud in the left sidebar tree.
4. Go to **Edit > Colors > Height Ramp**. Select the Z-axis, choose the default color gradient, and hit OK.
