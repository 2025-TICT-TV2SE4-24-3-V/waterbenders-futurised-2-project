# ChatGPT Links
- [Research](https://chatgpt.com/share/6a01811d-fc7c-83eb-bdb2-38881a21a26c)


# 1.) Setup
**This setup needs the dependencies installed according the Setup guide in the [ReadME](../lidarSensorOmgeving/ReadME.md) of lidarSensorOmgeving.**

1. Install OctoMap and check installation
- install octomap server
    ```bash
    source /opt/ros/jazzy/setup.bash
    apt update
    apt install -y ros-jazzy-octomap-server
    ```

- check the executable name:
    ```bash
    ros2 pkg executables octomap_server
    ```
    

- You should see something like:
    ```bash
    octomap_server color_octomap_server_node
    octomap_server octomap_eraser_cli.py
    octomap_server octomap_saver_node
    octomap_server octomap_server_multilayer_node
    octomap_server octomap_server_node
    octomap_server octomap_server_static_node
    octomap_server tracking_octomap_server_node
    ```


---

# 2.) Running files:
## 2.1) Running .sdf file with teleop for driving only
### Terminal 1 - starting gazebo

```bash
gz sim nameOfYourFile.sdf
```


### Terminal 2 - bridging Twist messages

```bash
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
/model/FLIP/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```


### Terminal 3 - running teleop for steering

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/model/FLIP/cmd_vel
```
---


## 2.2) Running files for Lidar mapping using 4 terminals
### Terminal 1 — start Gazebo

```bash
cd /workspace/prototypes/3D-Lidar-Mapping/
gz sim lidarRoomScan.sdf
```

### Terminal 2 — start bridge + TF + 3D mapping

```bash
cd /workspace/prototypes/3D-Lidar-Mapping/
source /opt/ros/jazzy/setup.bash
ros2 launch /workspace/prototypes/3D-Lidar-Mapping/octomap_gazebo.launch.py
```

### Terminal 3 — start RViz

```bash
cd /workspace/prototypes/3D-Lidar-Mapping/
source /opt/ros/jazzy/setup.bash
rviz2
```

In RViz:

* set **Fixed Frame** on the left top of RVIZ to `odom`
![Fixed Frame odom](./Screenshots/fixedFrameOdom.png)

* **Add**:
![Odometry](./Screenshots/odometry.png)
  * `Map` with topic `/projected_map`  from under `By Topic`
  ![Map](./Screenshots/map.png)

  * `MarkerArray` with topic `/occupied_cells_vis_array`  from under `By Topic`
  ![MarkerArray](./Screenshots/markerArray.png)

  * `Odometry` with topic `/odom` from under `By Topic`
  ![Odometry](./Screenshots/odometry.png)

  * `PointCloud2` with topic `/points` from under `By Topic`
  ![PointCloud2](./Screenshots/pointcloud2.png)

  * `TF` from under `By display type`
  ![transfom(tf)](./Screenshots/tf.png)


### Terminal 4 — drive the robot

**If your bridge YAML includes `/cmd_vel` to `/model/FLIP/cmd_vel`:**

```bash
cd /workspace/prototypes/3D-Lidar-Mapping/
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

If instead you did not bridge to ROS topic `/cmd_vel` but directly to `/model/FLIP/cmd_vel`, then use:

```bash
cd /workspace/prototypes/3D-Lidar-Mapping/
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/model/FLIP/cmd_vel
```

---


# 3.) Options:
1. Octomap
2. LIO-SAM (Lidar Inertial Odometry via Smoothing And Mapping)
3. ~~LOAM (LiDAR Odometry and Mapping)~~
4. ~~LeGO-LOAM (A lightweight version of LOAM)~~
5. ~~RTAB-Map (Real-Time-Appearance-Based Mapping -> graph-based SLAM solution. Excellent for long-term mapping due to its memory management system.) + RGB-D SLAM~~
6. Cartographer (Google) - A versatile graph-based SLAM system that supports both 2D and 3D mapping using various sensors
7. Nav2