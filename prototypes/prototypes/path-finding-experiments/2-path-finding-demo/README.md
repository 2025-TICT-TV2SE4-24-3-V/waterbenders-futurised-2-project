# Setup guide

## Quick Docker setup command

    cd /workspace/prototypes/path-finding-demo/
    gz sim room-v2.sdf &
    source /opt/ros/jazzy/setup.bash
    ros2 launch slam_toolbox.launch.py &
    python3 path_finding.py

## 1. Install packages 

Run your Gazebo Docker container with:

    docker run -it <image> bash

Use the below command to install `colcon`:

    apt update
    apt install python3-colcon-common-extensions

colcon is the build tool used in ROS2. It's a general multi-package build tool for creating packages.

### Commit the Docker container

Open a new terminal and run `docker ps` and identify the correct CONTAINER ID corresponding to the image that's running.

Then commit the docker with:

    docker commit <container-id> name:version

## 2. Build ROS2 packages

First navigate to `/path-finding-demo/` with:

    cd /workspace/prototypes/path-finding-demo/

And run the following command to build all colcon packages used for path-finding:

    colcon build

## X. Source ROS2

Source ROS2 with the below command. This needs to be repeated each time you re-enter the Docker container.

    source /opt/ros/jazzy/setup.bash

