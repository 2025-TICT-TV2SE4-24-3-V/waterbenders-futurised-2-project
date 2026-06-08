# ChatGPT Links
- [Octomap](https://chatgpt.com/share/6a01811d-fc7c-83eb-bdb2-38881a21a26c)
- [RTAB-Map](https://chatgpt.com/share/6a01f66b-6abc-83eb-a51f-6d78ea842b00)
- [RTAB 3D mapping](https://chatgpt.com/share/6a148d3a-b9c4-83eb-b04a-9cbd8caceb15)

# 1.) Setup
**This setup needs the dependencies installed according the Setup guide in the [ReadME](../lidarSensorOmgeving/ReadME.md) of lidarSensorOmgeving.**

1. Install RTAB-Map for ROS 2 Jazzy and check installation
- install octomap server
    ```bash
    source /opt/ros/jazzy/setup.bash
    apt update
    apt install -y ros-jazzy-rtabmap-ros
    ```

- Check that it exists:
    ```bash
    ros2 pkg prefix rtabmap_launch
    ros2 pkg executables rtabmap_slam
    ros2 pkg executables rtabmap_odom
    ```
    

- You should see something like:
    ```bash
    /opt/ros/jazzy
    rtabmap_slam rtabmap
    rtabmap_odom icp_odometry
    rtabmap_odom rgbd_odometry
    rtabmap_odom stereo_odometry

    ```


---

# 2.) Running files in prototype:
## 2.1) Running files for Lidar mapping using 3 terminals
### Terminal 1 — start Gazebo

```bash
cd /workspace/prototypes/3D-Lidar-Mapping-RTAB/
gz sim lidarRoomScan.sdf
```

### Terminal 2 — start bridge + TF + 3D mapping

```bash
cd /workspace/prototypes/3D-Lidar-Mapping-RTAB/
source /opt/ros/jazzy/setup.bash
ros2 launch /workspace/prototypes/3D-Lidar-Mapping-RTAB/rtabmap_gazebo.launch.py db_name:=nameYouWishToGiveTheDbFile.db
```


### Terminal 3 — drive the robot

**If your bridge YAML includes `/cmd_vel` to `/model/FLIP/cmd_vel`:**

```bash
cd /workspace/prototypes/3D-Lidar-Mapping-RTAB/
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

If instead you did not bridge to ROS topic `/cmd_vel` but directly to `/model/FLIP/cmd_vel`, then use:

```bash
cd /workspace/prototypes/3D-Lidar-Mapping-RTAB/
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/model/FLIP/cmd_vel
```

---

## 2.2) Viewing saved .db file for interactivity with map
### Terminal 1 — Launch RTAB-Map + RViz with `viewMap.launch.py ` file which is a copy of rtabmap_gazebo.launch.py but with the '-d ' commented on line 100 so it doesn't delete/recreate the database, which can wipe the saved map
```bash
cd /workspace/prototypes/3D-Lidar-Mapping-RTAB
source /opt/ros/jazzy/setup.bash
ros2 launch viewMap.launch.py  db_name:=nameOfDbFileYouWishToView.db
```


### Terminal 2 — Publish the saved map
```bash
cd /workspace/prototypes/3D-Lidar-Mapping-RTAB
source /opt/ros/jazzy/setup.bash
ros2 service call /rtabmap/rtabmap/publish_map rtabmap_msgs/srv/PublishMap "{global_map: true, optimized: true, graph_only: false}" 
```



## 2.3) Rviz setup if config file doesn't work
| RViz display  | Topic / frame                              | Purpose                                     |
| ------------- | ------------------------------------------ | ------------------------------------------- |
| `TF`          | all frames                                 | Check `map`, `odom`, `chassis`, LiDAR frame |
| `MapCloud`    | `/rtabmap/mapData`                         | Main 3D RTAB-Map map                        |
| `PointCloud2` | `/points_colored`                          | Live colored LiDAR scan                     |
| `PointCloud2` | `/rtabmap/cloud_map` if available          | RTAB-Map assembled cloud                    |
| `Map`         | `/rtabmap/grid_map` if available           | 2D occupancy map                            |
| `Odometry`    | `/odom`                                    | Robot pose from Gazebo                      |
| `Path`        | `/rtabmap/mapPath` or similar if available | Estimated trajectory                        |


# 3.) Options:
1. Octomap
2. LIO-SAM (Lidar Inertial Odometry via Smoothing And Mapping)
3. LOAM (LiDAR Odometry and Mapping)
4. LeGO-LOAM (A lightweight version of LOAM)
5. Cartographer (Google) - A versatile graph-based SLAM system that supports both 2D and 3D mapping using various sensors
6. Nav2


# 4.) Running files in models folder:
## 4.1) Running files for Lidar mapping using 2 terminals
### Terminal 1 — start Gazebo
```bash
cd /workspace/models/gazebo
gz sim environment.sdf
```

### Terminal 2 — start bridge + TF + 3D mapping

```bash
cd /workspace/models/scripts
source /opt/ros/jazzy/setup.bash
ros2 launch /workspace/models/scripts/rtabmap_gazebo.launch.py db_name:=nameYouWishToGiveTheDbFile.db
```

### Use `key publisher` to drive the robot with the arrow keys

---

## 4.2) Viewing saved .db file for interactivity with map
### Terminal 1 — Launch RTAB-Map + RViz with `viewMap.launch.py ` file which is a copy of rtabmap_gazebo.launch.py but with the '-d ' commented on line 100 so it doesn't delete/recreate the database, which can wipe the saved map
```bash
cd /workspace/models/scripts
source /opt/ros/jazzy/setup.bash
ros2 launch viewMap.launch.py  db_name:=nameOfDbFileYouWishToView.db
```


### Terminal 2 — Publish the saved map
```bash
cd /workspace/models/scripts
source /opt/ros/jazzy/setup.bash
ros2 service call /rtabmap/rtabmap/publish_map rtabmap_msgs/srv/PublishMap "{global_map: true, optimized: true, graph_only: false}" 
```




## 5.) Problems
### Links
- [ChatGPT](https://chatgpt.com/c/6a189349-6be0-83eb-a72c-b77ecc5af33a)



### ROS-BRIDGE file
- When integrating 3D-lidarmapping into our workspace folder I encountered problems with the `rosBrigde.yaml` file. A bridge was missing for pointCloud data so that had to be added.
- Secondly the name of the .yaml file was different which caused errors so pay attention to file names when encountering errors.

### Wrong topic static_laser_tf 
- Since I had to make a link for the lidar sensor and attach that to flip in the models folder, the topic for `static_laser_tf` also had to be changed from `FLIP/lidar_link/gpu_lidar` to `FLIP/3D_LidarSensor/lidar_sensor_link/gpu_lidar` in both [rtabmap_gazebo.launch.py](../../models/scripts/rtabmap_gazebo.launch.py) and [viewMap.launch.py](../../models/scripts/viewMap.launch.py). So if your project is built around modularity for your model like ours. Please keep in mind to change the topics if your sensors have links that are joined to the main model!

### Odometry without IMU or RGB-D camera
To keep the simulation realistic, the robot has no IMU sensor and no RGB-D camera — matching the real hardware which only carries a 3D LiDAR. SLAM however requires a continuous pose estimate (odometry) to track the robot's position and orientation. The default source, Gazebo wheel odometry, drifts significantly due to wheel slip and simulation inaccuracies, making it unreliable for 3D mapping.

Two sensor-based alternatives exist for odometry, but both were ruled out on realism grounds:
- **IMU** — measures raw acceleration and angular velocity, not pose. It cannot produce a pose estimate on its own without double-integrating its signal, which accumulates drift rapidly. It always needs to be fused with another source, making it a supplement rather than a solution.
- **RGB-D camera (depth camera)** — can produce odometry via visual feature tracking across frames (RGB-D odometry), but the real robot has no depth camera, so adding one in simulation would misrepresent the actual sensor setup.

To solve this using only the LiDAR already on the robot, we use **ICP (Iterative Closest Point) odometry**. ICP replaces **wheel odometry** by aligning consecutive LiDAR point clouds to estimate the pose delta (translation + rotation) between frames directly from scan data. This fills the same odometry slot that wheel odometry, visual odometry, or RGB-D odometry would normally fill — without requiring any sensor beyond the LiDAR.