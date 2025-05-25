# 🚗 Gazebo Harmonic Rover
This repository contains two ROS 2 Humble packages for testing simple rovers in Gazebo Harmonic:

- 🦾 simple_rover: simulate a basic rover with various sensors

- 🏎️ f1tenth_sim: run an F1Tenth car simulation

**Refer to this tutorial to first download the files from the offline model repository, then either export the model path to ```GZ_SIM_RESOURCE_PATH``` or add the export command to your ```.bashrc```.**

- https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics?tab=readme-ov-file

**If the model or world file still cannot be opened and shows errors, please open the SDF file and update any hardcoded paths to models, meshes, or other resources so that they point to the correct locations on your system.**

## 📦 Installation
```bash
git clone https://github.com/kimhoyun-robotair/Gazebo_Harmonic_Rover.git
cd Gazebo_Harmonic_Rover
colcon build --symlink-install
source install/local_setup.bash    # or source install/setup.bash
```
### 🦾 simple_rover Package
**1. 🚀 Usage**
```bash
ros2 launch simple_rover spawn_robot.launch.py
```
- 🔌 Sensor Configuration
In urdf/simple_rover.urdf, various sensor plugins are included. To enable the ones you want, uncomment the corresponding <xacro:include> lines at the top:

```xml
    <!-- Add macros for camera -->
    <!-- xacro:include filename="$(find simple_rover)/urdf/camera.xacro" /-->

    <!-- Add macros for 2D LiDAR -->
    <xacro:include filename="$(find simple_rover)/urdf/2Dlidar.xacro"/>

    <!-- Add macros for 3D LiDAR -->
    <!-- xacro:include filename="$(find simple_rover)/urdf/3Dlidar.xacro" /-->

    <!-- Add macros for RGB-D camera -->
    <xacro:include filename="$(find simple_rover)/urdf/RGB_D.xacro"/>

    <!-- Add macros for IMU -->
    <xacro:include filename="$(find simple_rover)/urdf/imu.xacro"/>

    <!-- Add macros for GPS -->
    <!-- xacro:include filename="$(find simple_rover)/urdf/gps.xacro" /-->
```

**2. 🔄 ROS–Gazebo Topic Bridging**

In launch/spawn_robot.launch.py, the ros_gz_bridge node bridges Gazebo topics to ROS 2. Comment or uncomment the lines for the topics you need:

```python
gz_bridge_node = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
        "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
        "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
        # "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
        # "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        # "/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",
        "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        # "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
        "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
    ],
    output="screen",
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
)
```
Uncomment a line to activate that bridge.

**3. 🌍 World Configuration**

Edit launch/world.launch.py to choose your Gazebo world file and add your own models path:

```python
def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Name of the Gazebo world file to load'
    )

    pkg_simple_rover = get_package_share_directory('simple_rover')
    pkg_ros_gz_sim   = get_package_share_directory('ros_gz_sim')

    # Add your own Gazebo models path below
    gazebo_models_path = "/home/kimhoyun/gazebo_models"
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path
```
Replace gazebo_models_path with your local directory.

**4. NO GPS**

To enable GPS in Gazebo, add the following to your SDF file:
```xml
<plugin
  filename="gz-sim-navsat-system"
  name="gz::sim::systems::NavSat">
</plugin>
<!-- Set the coordinates for the world origin -->
<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model>
  <world_frame_orientation>ENU</world_frame_orientation>
  <latitude_deg>47.478950</latitude_deg>
  <longitude_deg>19.057785</longitude_deg>
  <elevation>0</elevation>
  <heading_deg>0</heading_deg>
</spherical_coordinates>
```
Note: Gazebo is currently unable to load this plugin, so GPS functionality is not working.
Once this issue is resolved, a troubleshooting guide will be added here.

**5. Cartographer, AMCL, and Nav2 🚀**

The simple_rover package supports both Cartographer and Nav2 (assuming the required metapackages are already installed).

🗺️ Launching Cartographer
```bash
source install/setup.bash
ros2 launch simple_rover cartographer.launch.py
```

🧭 Launching AMCL and Nav2
```bash
# Terminal 1
source install/setup.bash
ros2 launch simple_rover amcl.launch.py

# Terminal 2
source install/setup.bash
ros2 launch simple_rover navigation.launch.py
```

🤖 Running Autonomous Navigation with Nav2
```bash
source install/setup.bash
ros2 run nav2_programming follow_waypoints
# ros2 run nav2_programming navigate_through_poses
# ros2 run nav2_programming navigate_to_pose
```

Tip:
You can use follow_waypoints, navigate_through_poses, or navigate_to_pose depending on your navigation needs.

### 🏎️ f1tenth_sim Package
The f1tenth_sim package provides an F1Tenth car simulation in Gazebo Harmonic.

---

# 📜 Credits & License Notice

This project was made possible thanks to the open‑source work published by the MOGI‑ROS organization. I would like to express my deepest gratitude to all of its authors and maintainers for generously sharing their code and documentation.

Portions of this repository are derived from MOGI‑ROS course materials and retain their original license headers. Unless otherwise noted, all new code in Gazebo_Harmonic_Rover is released under the Apache License 2.0, in full accordance with the upstream terms.

Copyright © 2025 MOGI‑ROS and contributors
Copyright © 2025 Gazebo_Harmonic_Rover contributors

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

For full details, please refer to the individual source files and the upstream MOGI‑ROS repositories.
