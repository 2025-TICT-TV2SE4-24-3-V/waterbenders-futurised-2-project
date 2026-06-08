# Autonomous exploration of robot car

## Algorithm sub parts

- **Occupancy grid map** (based on Lidar sensor data)
- **Frontier based exploration** for determining the frontier edge cell
- **Breadth First Search (BFS) algorithm** for pathfinding

### Steps (high overview)

1. **Calculate angles** (relative to the world) corresponding to the distances from the Lidar sensor per measurement:
    - Convert the IMU orientation data from quaternions to Euler angles.
    - Calculate the angles corresponding to each distance measurement, using the polar-to-Cartesian conversion (beam angle formula).
    - Match the Lidar measurement with the IMU orientation data and apply linear interpolation to accommodate for distortion caused by rotation or movement.
    <!-- Retrieve the exact angle of the IMU orientation data at the same timestamp the Lidar measure was taken -->
    - Calculate the angle relative to the world using the above values.
2. **Create an occupancy grid** using the distances collected from the Lidar sensor and the angles as calculated in the last step:
    - h
3. **Use frontier based exploration** to determine the frontier edge cell, i.e. the.. in combination with Breath First Search (BFS)
4. **Reconstruct shortest path** from the outcome of the Breadt First Search algorithm
5. **Calculate the exact velocities** which the robot has to move in order to reach the frontier edge cell as specified by the result of BFS
6. **Re-evaluate** after the robot has taken a certain amount of steps (probably 1) on the Occupancy grid.

<!-- ray_angle = angle_min + (i * angle_increment) -->
<!-- +/- the actual angle  -->

```python
import mathdef 
quat_to_yaw(x,y,z,w):    
    siny_cosp = 2.0 * (w * z + x * y)    
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)    
    return math.atan2(siny_cosp, cosy_cosp)
```

Sources:

1. https://awabot.com/en/autonomous-exploration-method-frontiers/
2. https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/yamauchi_frontiers.pdf
3. https://www.redblobgames.com/pathfinding/a-star/introduction.html
4. https://atsushisakai.github.io/PythonRobotics/modules/3_mapping/lidar_to_grid_map_tutorial/lidar_to_grid_map_tutorial.html
5. https://robotics.stackexchange.com/questions/98107/how-to-get-obstacle-distance-and-angle-by-using-lidar
6. https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
