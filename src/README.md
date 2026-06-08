# Setup guide

## Table of Contents

- [Setup guide](#setup-guide)
  - [Table of Contents](#table-of-contents)
  - [Installation process](#installation-process)
    - [1. Setting up ROS \& SLAM](#1-setting-up-ros--slam)
    - [2. Build packages with Colcon](#2-build-packages-with-colcon)
    - [3. Commit the Docker container](#3-commit-the-docker-container)
    - [4. Create venv to install packages in container](#4-create-venv-to-install-packages-in-container)
    - [5. Starting with .ps1 file](#5-starting-with-ps1-file)
  - [Running the full simulation](#running-the-full-simulation)
  - [Build your own ROS2 packages](#build-your-own-ros2-packages)
  - [Viewing multiple sensor outputs](#viewing-multiple-sensor-outputs)
    - [4-Terminal setup](#4-terminal-setup)
    - [Camera's + Lidar 2D](#cameras--lidar-2d)
  - [What to do in Rviz](#what-to-do-in-rviz)
      - [Thermal camera output from Python file](#thermal-camera-output-from-python-file)
      - [Front \& rear camera](#front--rear-camera)
      - [Lidar 2D mapping](#lidar-2d-mapping)

## Installation process

### 1. Setting up ROS & SLAM

- Preparing the Docker container

    - Run the Gazebo Docker container with (or use a [`.ps1`](/docker/setup/container-creation/example-ps1-file.txt) file):

        ```bash
        docker run -it <image> bash
        ```

    - Update container:

        ```bash
        apt update
        apt install -y locales curl gnupg2 lsb-release software-properties-common

        locale-gen en_US en_US.UTF-8
        update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
        export LANG=en_US.UTF-8
        ```

- Adding repositories

    ```bash
    add-apt-repository universe -y
    apt update

    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu noble main" \
    > /etc/apt/sources.list.d/ros2.list
    ```

- Installing required software

    ```bash
    apt update && apt install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard ros-jazzy-rviz2 ros-jazzy-nav2-map-server python3-colcon-common-extensions ros-jazzy-nav2-costmap-2d ros-jazzy-rtabmap ros-jazzy-rtabmap-launch
    ```

- Install additional optional tools

    ```bash
    apt update && apt install ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro 
    ```

### 2. Build packages with Colcon

- Navigate to simulation folder

    ```bash
    cd /src
    ```

- Build Colcon packages

    - Build all Colcon packages:

        ```bash
        colcon build
        ```

    - Only selectively build a single package:

        ```bash
        colcon build --packages-select <package-name>
        ```

### 3. Commit the Docker container

- Open a new terminal and run `docker ps` and identify the correct CONTAINER ID corresponding to the image that's running

    - Then commit the Docker container with:
    
        ```bash
        docker commit <container-id> name:version
        ```

### 4. Create venv to install packages in container
1. Install venv support (once)

    ```bash
    apt update
    apt install -y python3-venv python3-full
    ```

1. Create a virtual environment
    ```bash
    python3 -m venv /venv --system-site-packages
    ```
4. Activate the venv before running any code
    ```bash
    source /venv/bin/activate
    ```
5. Run the python script
    ```bash
    python <file-name>.py
    ```

### 5. Starting with .ps1 file
To enter the docker container easily and install the right versions of Numpy, ultralytics and torch, create a .ps1 file with the following commands:
```bash
docker start <container-name>
docker exec <container-name> /venv/bin/pip install --quiet numpy==1.26.4 "numpy<2.0" "opencv-python<4.11"
docker exec -it -e DISPLAY=host.docker.internal:0 <container-name> bash
```

Run the file inside the repository:
```bash
./<file-name>.ps1
```

If you want to run the .ps1 file with a container using GPU instead if CPU, [see here](/docker/setup/using-cuda/README.md)

## Running the full simulation

- Before running the simulation make sure you've completed all the steps in the installation process, then run the below commands in the Docker container:

    - Navigate to the simulation folder:

        ```bash
        cd /src
        ```
    - Source ROS2:

        ```bash
        source /opt/ros/jazzy/setup.bash
        ```
    - Source the ROS2 workspace:

        ```bash
        source install/setup.bash
        ```
    - Launch the simulation:

        ```bash
        ros2 launch simulation.launch.py &
        ```


## Build your own ROS2 packages

Navigate to your own ROS2 workspace and use the below command to create a package with a pre-built node:

    ros2 pkg create --build-type ament_python --node-name main my_package

This will generate a basic package template, which one can use to develop custom colcon packages for ROS2. You can now edit `my_package/my_package/main.py` to subscribe/publish to a topic and add custom logic. See [path_planning/main.py]() TODO! for an example of the code structure.

After that build the package using (inside the `packages/` folder):

    colcon build

And add it to the `.launch.py` file, like this:

```python
    my_package = Node(
        package='my_package',
        executable='main',  # Matches the entry point in setup.py
        name='minimal_publisher',
        output='screen',
    )

    return LaunchDescription([
        bridge,
        static_laser_tf,
        slam_launch,
        my_package # <-- Added custom package
    ])
```

Source: https://roboticsknowledgebase.com/wiki/common-platforms/ros/ros2-custom-package/

## Viewing multiple sensor outputs

Below is a guideline for running the Gazebo simulation and using Rviz together with ROS2 to get the output from different sensors. It is required to use multiple terminals at the same time within the docker container. <br>*For some sensors there might be more steps, this is just a guideline.*


### 4-Terminal setup

ROS2 + Gazebo + Thermal Pipeline Startup

| Terminal | Purpose                                |
| -------- | -------------------------------------- |
| 1        | Gazebo simulation (`gz sim`)           |
| 2        | ROS ↔ Gazebo bridge                    |
| 3        | Python thermal + YOLO + RViz publisher |
| 4        | RViz visualization                     |

---

**TERMINAL 1** — Gazebo Simulator

```bash
cd /src TODO!
gz sim room-v2.sdf TODO!
```
 This runs your world + thermal camera plugin.

---

**TERMINAL 2** — ROS ↔ Gazebo Bridge
```bash
source /opt/ros/jazzy/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
/FLIP/thermal_camera@sensor_msgs/msg/Image@gz.msgs.Image \
/front/image@sensor_msgs/msg/Image@gz.msgs.Image \
/rear/image@sensor_msgs/msg/Image@gz.msgs.Image
```
This exposes Gazebo topics to ROS 2.

---

**TERMINAL 3** — Python Processing Node
```bash
source /venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd <python-file-location>
python <python-file>
```

---
 **TERMINAL 4** — RViz2 Visualization
```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

### Camera's + Lidar 2D

--- 

**TERMINAL 1** — Gazebo Simulator

```bash
cd models TODO!
gz sim room-v2.sdf TODO!
```

---
**TERMINAL 2** — ROS2 Bridge

Instead of the regular **terminal 2** prompt as above, if we want **multiple** sensors to show in Rviz we need to make sure we're **bridging ROS2 with Rviz**. Use this prompt for both camera and thermal camera in **terminal 2**:
```bash 
source /opt/ros/jazzy/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
/FLIP/thermal_camera@sensor_msgs/msg/Image@gz.msgs.Image \
/front/image@sensor_msgs/msg/Image@gz.msgs.Image \
/rear/image@sensor_msgs/msg/Image@gz.msgs.Image
```
---
**TERMINAL 3** — Python Processing Node
```bash
source /venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd /workspace/models/scripts/thermal-camera TODO!
python thc-rviz.py
```
---
 **TERMINAL 4** — RViz2 Visualization
```bash
source /opt/ros/jazzy/setup.bash
rviz2
```
---

**TERMINAL 5:** - Lidar Python launch

```bash
cd /workspace/models/scripts/lidar-2d-rviz TODO!
source /opt/ros/jazzy/setup.bash
ros2 launch /workspace/models/scripts/lidar-2d-rviz/slam_gazebo.launch.py TODO!
```

## What to do in Rviz

---

#### Thermal camera output from Python file
*In RViz UI:*
1. Click **Add**
2. Select **Image**
3. Set topic to:
    `/thermal/heatmap_image/image`

---
#### Front & rear camera
*In RViz UI:*
1. Click **Add**
2. Select **Image**
3. Set topic to:
    `/front/image/image`
4. Set topic to:
    `/rear/image/image`

To keep things clear, after adding the first camera *rename* them for clear view.

---
#### Lidar 2D mapping
1. set **Fixed Frame** to `map`
2. Click **Add**
3. Select **Image**
4. Set topic to:
    `/map/map`
5. Set topic to:
    `/scan/LaserScan`
6. Set topic to:
    `/odom/Odometry`
7. Set **Display Type** to:
    `/TF`
