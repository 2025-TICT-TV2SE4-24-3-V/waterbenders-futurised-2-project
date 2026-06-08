## Setup
1. Install necessary packages after following the [setup](../lidarSensorOmgeving/ReadME.md) in the `lidarSensorOmgeving` folder 
```bash
apt update && apt install python3-colcon-common-extensions python3-rosdep ros-jazzy-ros-gz ros-jazzy-cartographer ros-jazzy-cartographer-ros ros-jazzy-slam-toolbox ros-jazzy-cartographer-rviz -y
```


## Running files
### In a single terminal

1. One-liner
```bash
# Change directory
cd /workspace/prototypes/3d-cartographer/
# Launch Gazebo simulation
gz sim room-v2.sdf &
# Source ROS2
source /opt/ros/jazzy/setup.bash
# Launch .launch.py file
ros2 launch slam_gazebo.launch.py &
```

2. Give command for test moement and mapping
```bash
gz topic -t "/model/FLIP/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"
```