# 🚗 Gazebo Harmonic Rover
This repository contains two ROS 2 Humble packages for testing simple rovers in Gazebo Harmonic:

- 🦾 simple_rover: simulate a basic rover with various sensors

- 🏎️ f1tenth_sim: run an F1Tenth car simulation

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
---
### 🏎️ f1tenth_sim Package
The f1tenth_sim package provides an F1Tenth car simulation in Gazebo Harmonic.
