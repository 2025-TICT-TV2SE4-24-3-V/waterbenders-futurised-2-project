## ToDo

1. Check for updates to unknown areas (-1) on the map
2. Use a costmap to accommodate for the robots size
    - apt install ros-jazzy-nav2-costmap-2d
    <!-- -   ``` -->
    <!--     ros2 lifecycle set /global_costmap configure -->
    <!--     ros2 lifecycle set /global_costmap activate -->
    <!--     ``` -->
3. Update `cmd_vel` and `pd_controller` for faster movement
4. ~~Implement cost calculation based on current orientation of the robot~~
5. Use sensor data to determine when to stop for an object
6. Reduce debug messages
7. Reorganize function definitions
8. Add documentation and helpful comments

## Notes

| Callback/Timer | Shared variables (marked) | Sub functions |
| --- | --- | --- |
| `map_callback()` | `self.map` | - |
| `costmap_callback()` | `self.costmap`, self.costmap_origin, self.costmap_resolution, self.costmap_width, self.costmap_height, self.costmap_updated | - |
| `pose_callback()` | self.robot_position, self.robot_orientation | quat_euler_angle |
| `frontier_exploration()` | `self.costmap`, self.costmap_updated, self.map, self.robot_position, self.world_path, self._grid_path, self._frontier_cells | find_frontier_cells, world_grid_coordinates, dijkstra, grid_world_coordinates |
| `publish_cmd_vel()` | `self.world_path`, self.robot_position, self.robot_orientation, self.prev_error, `self.object_nearby` | calc_distance_rotation, pd_controller |
| `lidar_callback()` | `self.object_nearby` | - |

<!-- | draw_map | self.map, self.costmap, self._frontier_cells, self.robot_position, self._grid_path | world_grid_coordinates | -->

Shared variables:
- self.costmap
- self.map
- self.object_nearby
- self.world_path
- self.robot_position

When concurrency becomes worthwhile

I would only add concurrency if:

Case 1

Planning takes a long time:

100 ms -> probably not
500 ms -> maybe
2 s -> probably yes

### The Core Principle

Lock whenever there's a possibility of concurrent access where at least one access is a write. This includes:

    Write + Write: Obviously unsafe
    Write + Read: Unsafe—reader might see torn/partial writes
    Read + Read: Safe (assuming primitive types like integers, though this depends on your platform)

### Callbacks

| Function | Shared variables | - | - |
| --- | --- | --- | --- |
| `map_callback()` | 
| `costmap_callback()` | 
| `pose_callback()` | 
| `lidar_callback()` |

### Timers

| Function | Sub functions | Shared variables | Race conditions |
| --- | --- | --- | --- |
| `publish_cmd_vel()` |
| `frontier_exploration()` | dijkstra(), | self.costmap | 
| `draw_map()` |

### Functions

| Function | Shared variables | Race conditions | - |
| --- | --- | --- | --- |
| `dijkstra()` | self.costmap |
- quat_euler_angle
- calc_distance_rotation
- pd_controller
- world_grid_coordinates
- grid_world_coordinates
- find_neighbouring_cells
- find_frontier_cells
- pd_controller

## Understanding Callbacks in Single Thread Execution

To determine which callbacks can run effectively in a single thread, consider the nature of the tasks they perform. Callbacks that are I/O-bound or involve waiting are best suited for single-thread execution in Python.

### I/O-Bound vs. CPU-Bound Tasks
| Task Type | Description | Suitability for Single Thread |
| --- | --- | --- |
| I/O-Bound | Tasks that involve waiting for external resources, such as network requests, file operations, or database calls.	| High |
| CPU-Bound | Tasks that require significant CPU processing, such as complex calculations or data processing. | Low |

## Understanding the difference between race conditions and deadlocks

https://stackoverflow.com/a/3130212
https://stackoverflow.com/a/3130111

## Solving race conditions in Python

There are several ways to solve race conditions in Python: 1) Using locks (threading.Lock()) to ensure that only one thread can modify a shared resource at a time, and 2) Using the Queue module (queue.Queue()) for thread-safe data structures. Additionally, multiprocessing can be used for true parallel execution, isolating processes to avoid race conditions.

```
# At the very beginning of publish_cmd_vel()
if self.path is not None and len(self.path) > 2:
    if self.waypoint_reached:  # Flag from previous callback
        self.path.pop(1)
        self.waypoint_reached = False
        self.prev_error = 0.0
        self.integral_error = 0.0

# Check whether path exists and is long enough
if self.path is None or len(self.path) < 2:
    return

# Extract the second coordinate from the path
x, y = self.path[1]

# Obtain the distance and angle (always fresh for current waypoint)
relative_distance = self.calc_relative_distance(x, y)
relative_angle = self.calc_relative_angle(x, y)

if relative_distance < self.DISTANCE_THRESHOLD:
    self.get_logger().info("Subgoal reached")
    self.waypoint_reached = True  # ← Set flag instead of popping

if len(self.path) <= 2:
    self.get_logger().info('Final goal reached')
    twist_msg.angular.z = 0.0
    twist_msg.linear.x = 0.0
    self.prev_error = 0.0
    self.integral_error = 0.0
    self.path = None
else:
    if abs(relative_angle) > 0.1:
        twist_msg.angular.z = self.pid_controller(relative_angle, Kp=1.5, Ki=0.05, Kd=0.3, dt=0.1)
        twist_msg.angular.z = max(-1.0, min(1.0, twist_msg.angular.z))
        twist_msg.linear.x = 0.0
    else:
        twist_msg.angular.z = 0.0
        twist_msg.linear.x = 0.4
```

## Old beadth_first_search

```python
    def breadth_first_search(self, grid, start: tuple):

        frontier_cells = self.find_frontier_cells(grid)
        self._frontier_cells = frontier_cells.copy()

        if len(frontier_cells) == 0:
            self.get_logger().warn("No frontier cells found")

        # Debug
        self.get_logger().info(f"Frontier cells found: {len(frontier_cells)}")

        frontier = Queue()
        frontier.put(start)
        reached = {}
        reached[start] = None

        while not frontier.empty():

            current = frontier.get()
            neighbours = self.find_neighbouring_cells(grid, current[0], current[1])

            # Exit the loop prematurely when a frontier 
            # edge cell has been reached and remove its value
            if current in frontier_cells:
                self.get_logger().info(f"Current: {current}, {current in frontier_cells}")
                frontier_cells.remove(current)
                break

            for ni in neighbours:
                if ni not in reached and grid[ni] != 100:
                    frontier.put(ni)
                    reached[ni] = current

        path = []
        while current != start:
            path.append(current)
            current = reached[current]
        path.append(start)
        path.reverse()

        # Test messages
        # print("Reached: ", reached)
        # print("Values of reached: ", [grid[ni] for ni, _ in reached.items()])

        return path

```

```
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import numpy as np
from tf2_ros import Buffer, TransformListener
import math
from queue import Queue, PriorityQueue
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

class MapAnalyzer(Node):

    # Constants
    DISTANCE_THRESHOLD = 0.75
    OBJECT_THRESHOLD = 1.0

    def __init__(self):
        super().__init__('path_finding')

        self.grid = None

        self.grid_origin = None
        self.grid_resolution = 0.0

        self.grid_width = 0
        self.grid_height = 0

        self.robot_position = None
        self.robot_orientation = 0.0

        self.world_path = None
        self._frontier_cells = None

        self.prev_error = 0.0

        self.object_nearby = False

        # Live mapping of path-finding data
        plt.ion() # Enable interactive plotting
        self.fig, self.ax = plt.subplots()
        self.cmap = ListedColormap(['white', 'white', 'black'])
        self.norm = BoundaryNorm([-1.5, -0.5, 0.5, 100.5], self.cmap.N)

        self.map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10 # Queue size
        )

        self.cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10
        )

        self.lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Timer to publish to cmd_vel topic(e.g., 10 Hz)
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

    def lidar_callback(self, msg: LaserScan):

        # self.get_logger().info(f"Lidar data samples amount: {len(msg.ranges)}")

        middle_index = int(len(msg.ranges) / 2)
        lidar_object_range = int((len(msg.ranges) / 360) * 120 / 2)

        for i in range(middle_index - lidar_object_range, middle_index + lidar_object_range):
            if msg.ranges[i] < self.OBJECT_THRESHOLD:
                self.get_logger().info(f"Minimal distance: {msg.ranges[i]}")
                self.object_nearby = True

    def update_plot(self):

        if self.grid is None:
            return

        self.ax.clear()
        # Source: https://matplotlib.org/stable/users/explain/artists/imshow_extent.html#imshow-extent
        self.ax.imshow(self.grid, cmap=self.cmap, norm=self.norm,
                       origin='lower') # Reflect the map to align with that of the Gazebo world
        self.ax.set_title("Occupancy Grid")

        # Plot all frontier cells
        if self._frontier_cells:
            y = [y for y, _ in self._frontier_cells]
            x = [x for _, x in self._frontier_cells]
            self.ax.scatter(x, y, s=50, c='red', alpha=0.5)

        if self.robot_position:
            y, x = self.world_grid_coordinates(self.robot_position[0], self.robot_position[1])
            self.ax.scatter(x, y, c='blue', s=50, alpha=0.5)

        if self.world_path:
            y, x = self.world_grid_coordinates(self.world_path[-1][0], self.world_path[-1][1])
            self.ax.scatter(x, y, c='orange', s=50, alpha=0.5)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def pose_callback(self, msg: PoseWithCovarianceStamped):

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.robot_position = (position.x, position.y)
        self.robot_orientation = self.quat_euler_angle(orientation.x, orientation.y, orientation.z, orientation.w)

        # Debug
        # self.get_logger().info(f'Received position: {position.x, position.y}')
        # self.get_logger().info(f'Received orientation: {self.quat_euler_angle(orientation.x, orientation.y, orientation.z, orientation.w)}')

    def quat_euler_angle(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)    
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z)    
        return math.atan2(siny_cosp, cosy_cosp)

    def calc_distance_rotation(self, target_x, target_y):

        robot_x, robot_y = self.robot_position

        # Calculate the distance between the robot and the target
        distance_x = target_x - robot_x
        distance_y = target_y - robot_y

        relative_distance = math.sqrt(distance_x**2 + distance_y**2)

        absolute_angle = math.atan2(distance_y, distance_x)
        relative_angle = absolute_angle - self.robot_orientation
        relative_angle = (relative_angle + math.pi) % (2 * math.pi) - math.pi # Normalize to [-π, π]

        return relative_distance, relative_angle

    def pd_controller(self, error, Kp, Kd, dt):
        derivative = (error - self.prev_error) / dt
        # Store previous error for derivative term
        self.prev_error = error

        # Output
        u = Kp * error + Kd * derivative

        return u

    def publish_cmd_vel(self):

        twist_msg = Twist()

        # Check whether world_path exists
        if self.world_path is None: # Using 'is None' is faster then checking with ==
            # self.get_logger().info('No path exists yet')
            return

        if len(self.world_path) <= 1:
            self.get_logger().info('Final goal reached')

            # twist_msg.angular.z = 0.0
            # twist_msg.linear.x = 0.0
            # self.cmd_vel.publish(twist_msg)

            self.world_path.clear()

            return

        # if self.object_nearby == True:
        #     self.get_logger().info("Object detected!")
        #     twist_msg.angular.z = 0.0
        #     twist_msg.linear.x = 0.0
        #     self.cmd_vel.publish(twist_msg)
        #
        #     return

        # Extract the second coordinate from the path (i.e. the first coordinate to move towards)
        x, y = self.world_path[1]

        # Test messages
        # self.get_logger().info(f"Current position: x={self.robot_position[0]}, y={self.robot_position[1]}")
        # self.get_logger().info(f"Current goal    : x={x}, y={y}")

        relative_distance, relative_angle = self.calc_distance_rotation(x, y)

        if relative_distance < self.DISTANCE_THRESHOLD: # Stop when below threshold
            self.world_path.pop(1) # Remove the coordinates when reached,
            # so that each function call it will move along the planned path
            twist_msg.angular.z = 0.1 # Stop turning completely (or test 0.1)
            twist_msg.linear.x = 0.4

            # Debug messages
            # self.get_logger().warn("Subgoal reached")

            self.prev_error = 0.0  # Reset PD controller error for the new path

        else:
            # dt is equal to 0.2, because /pose changes at 5hz
            twist_msg.angular.z = self.pd_controller(relative_angle, Kp=0.5, Kd=0.2, dt=0.2)
            twist_msg.angular.z = max(-1.0, min(1.0, twist_msg.angular.z)) # Clamp to [-0.5, 0.5]

            # twist_msg.linear.x = 0.2  # Fixed forward speed (deprecated)

            # Linear speed that decreases as the relative_distance changes (slows down as you approach)
            linear_speed = 0.4 * (1 - min(1.0, relative_distance / 0.5))  # Max speed at 0.5m away
            twist_msg.linear.x = max(0.2, linear_speed)  # Never go below 0.2 m/s

        self.cmd_vel.publish(twist_msg)

        # Debug
        # self.get_logger().info(f"Published cmd_vel: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}")

        return
        
    # Convert real-world coordinates to grid cell (x, y)
    def world_grid_coordinates(self, ix, iy):

        # Source: https://stackoverflow.com/questions/34952651/only-integers-slices-ellipsis-numpy-newaxis-none-and-intege
        ci = int((iy - self.grid_origin[1]) / self.grid_resolution)
        ri = int((ix - self.grid_origin[0]) / self.grid_resolution)

        # Calculate the grid x, y coordinates and invert the y-axis. 
        # The round() function is used to convert the decimal values to the nearest 
        # grid cell, because cells are always whole values and cannot be fractional indices
        # grid_x = round((ix - self.grid_origin[0]) / self.grid_resolution)
        # grid_y = self.grid_height - 1 - round((iy - self.grid_origin[1]) / self.grid_resolution)

        # Clamp to grid bounds to avoid the robots position being outside the grid. 
        # This happens when the map generated by the Lidar hasn't explored the region 
        # the robot is currently in (e.g. at the start where the Lidar only scans forward).
        # grid_x = max(0, min(self.grid_width - 1, grid_x))
        # grid_y = max(0, min(self.grid_height - 1, grid_y))

        return ci, ri

    # Convert a grid cell (x, y) to real-world coordinates
    def grid_world_coordinates(self, ci, ri): # y, x

        world_x = (ri * self.grid_resolution) + self.grid_origin[0]
        world_y = (ci * self.grid_resolution) + self.grid_origin[1]

        return world_x, world_y

    def find_neighbouring_cells(self, array, column, row, diagonals=False):

        height, width = array.shape

        neighbours = []

        # All neighbouring cells (including diagonals)
        directions = [
            (0, 1),
            (0, -1),
            (1, 0),
            (-1, 0),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1)
        ]

        for i, (x, y) in enumerate(directions):
            if diagonals == False and i > 3:
                break
            ri = row + x
            ci = column + y
            # Check whether each cell has a neighbour (isn't at the edge of the map)
            if 0 <= ri < width and 0 <= ci < height:
                index_pair = ci, ri 
                neighbours.append(index_pair)

        return neighbours

    # Scans the occupancy grid and identifies all free cells (0) adjacent to unknown cells (-1), resulting in frontier edge cells or candidate boundaries
    def find_frontier_cells(self, grid):

        height, width = grid.shape
        frontier_cells = []

        for ci in range(height): # iy
            for ri in range(width): # ix

                if grid[ci, ri] == 0:
                    index_pair = ci, ri

                    neighbours = self.find_neighbouring_cells(grid, ci, ri, diagonals=False)

                    for ni in neighbours:
                        # Check if neighbouring cell is unknown (-1) and coordinates 
                        # haven't already been added to 'candidate_boundaries' (avoids duplicates)
                        if grid[ni] == -1 and index_pair not in frontier_cells: # Redundant check?

                            world_x, world_y = self.grid_world_coordinates(ci, ri)
                            if self.calc_distance_rotation(world_x, world_y)[0] <= 0.95:
                                # self.get_logger().info(f"Distance to robot: {self.calc_distance_rotation(world_x, world_y)[0]}")
                                continue

                            frontier_cells.append(index_pair)
                            break

        return frontier_cells

    # Source: https://www.redblobgames.com/pathfinding/a-star/introduction.html
    # Scans grid and returns the first step in the direction of the closest valid edge cell (candidate boundary)
    def breadth_first_search(self, grid, start: tuple):

        frontier_cells = self.find_frontier_cells(grid)
        self._frontier_cells = frontier_cells.copy()

        if len(frontier_cells) == 0:
            self.get_logger().warn("No frontier cells found")

        # Debug
        self.get_logger().info(f"Frontier cells found: {len(frontier_cells)}")

        frontier = PriorityQueue()
        frontier.put((0, start))
        reached = {}
        costs = {}
        reached[start] = None
        costs[start] = 0

        while not frontier.empty():

            _, current = frontier.get()
            neighbours = self.find_neighbouring_cells(grid, current[0], current[1])

            # Exit the loop prematurely when a frontier 
            # edge cell has been reached and remove its value
            if current in frontier_cells:
                self.get_logger().info(f"Current: {current}, {current in frontier_cells}")
                frontier_cells.remove(current) # Redundant?
                break

            for ni in neighbours:

                world_x, world_y = self.grid_world_coordinates(ni[0], ni[1])
                rotations_cost = abs(self.calc_distance_rotation(world_x, world_y)[1])
                new_cost = costs[current] + rotations_cost

                if ni not in reached or new_cost < costs[ni]: 
                    if grid[ni] != 100:
                        costs[ni] = new_cost
                        frontier.put((new_cost, ni)) # Priority, value
                        reached[ni] = current

        path = []
        while current != start:
            path.append(current)
            current = reached[current]
        path.append(start)
        path.reverse()

        # Test messages
        # print("Reached: ", reached)
        # print("Values of reached: ", [grid[ni] for ni, _ in reached.items()])

        return path
        
    def map_callback(self, msg: OccupancyGrid):

        self.get_logger().info('Received map')

        # Debug
        # if self.world_path is not None:
        #     wx, wy = self.world_path[-1]
        #     self.get_logger().warn(f"Final goal grid coordinates (y, x): {self.world_grid_coordinates(wx, wy)}")

        self.grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        # self.grid = np.flip(self.grid, 0) # Reflect the np.array vertically (over the x-axis) to 
        # account for the origin being at the lower left corner (-x, -y)

        self.get_logger().info(f"{self.grid}")

        # The origin in the Gazebo simulation is always located in the bottom left corner, 
        # so where x, y are both minus (-x, -y). It changes as the robot moves
        # further into unknown territory
        self.grid_origin = (msg.info.origin.position.x, msg.info.origin.position.y) # (x, y)
        self.grid_resolution = msg.info.resolution
        self.grid_width = msg.info.width
        self.grid_height = msg.info.height

        self.update_plot()

        # Debug
        self.get_logger().info(f'Map origin: {self.grid_origin}')
        self.get_logger().info(f'Map resolution: {self.grid_resolution}')
        self.get_logger().info(f'Map width: {self.grid_width}')
        self.get_logger().info(f'Map height: {self.grid_height}')

        # Additional check to determine whether the robot hasn't moved (i.e. a /pose msg hasn't been sent out yet)
        if self.robot_position is None:
            self.get_logger().warn("Robot position unknown")
            return

        # rx, ry = self.world_grid_coordinates(self.robot_position[0], self.robot_position[1])
        #
        # self.grid[ry, rx] = 100

        # Skip if the robot is outside the current map bounds
        # robot_x, robot_y = self.robot_position
        # if (robot_x < self.grid_origin[0] or # Left of map
        #     robot_x > self.grid_origin[0] + self.grid_width * self.grid_resolution or # Right of map
        #     robot_y < self.grid_origin[1] or # Below map
        #     robot_y > self.grid_origin[1] + self.grid_height * self.grid_resolution): # Above map
        #     self.get_logger().warn("Robot is outside the current map bounds. Waiting for map update...")
        #     return

        # Skip if a path is still being executed
        if self.world_path is not None:
            if len(self.world_path) > 1:
                self.get_logger().warn("Current path hasn't been fully completed")
                return

        self.get_logger().warn("New path is being calculated")

        # Calculate a new path
        grid_position = self.world_grid_coordinates(self.robot_position[0], self.robot_position[1])
        grid_path = self.breadth_first_search(self.grid, grid_position)
        self.world_path = [self.grid_world_coordinates(ci, ri) for (ci, ri) in grid_path]

        # Debug
        self.get_logger().warn(f"Final grid goal (x, y): {grid_path[-1]}")
        self.get_logger().warn(f"Final goal (x, y): {self.world_path[-1]}")
        # self.get_logger().info(f"Grid path length: {len(grid_path)}, path: {grid_path[:5]}")

        return
 
def main(args=None):
    rclpy.init(args=args)
    node = MapAnalyzer()

    """
    TESTS
    """

    # Values meaning:
    # -1 = unknown
    # 0  = free
    # 100 = occupied

    # Width of 4 and height of 3
    arr = np.array([ 
        [0, 100, 100, -1], 
        [0, 0, 0, 100],
        [-1, 0, 0, -1]
    ])
    
    # node.get_logger().info(f"{node.find_neighbouring_cells(arr, 0, 0)}")

    assert node.find_neighbouring_cells(arr, 0, 0) == [(1, 0), (0, 1)], "4-way neighbours don't match"

    assert node.find_neighbouring_cells(arr, 0, 0, True) == [(1, 0), (0, 1), (1, 1)], "Diagonal neighbours don't match"

    # assert node.candidate_boundaries(arr) == [(1, 0), (2, 1), (2, 2)], "Candidate boundaries are invalid"

    # assert node.breadth_first_search(arr, (0, 0)) == [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (2, 3)], "Custom breadth first search invalid"

    try:
        rclpy.spin(node)  # Run until ctrl-c is pressed
    except KeyboardInterrupt:
        pass

    # After ctrl-c plot the latest map
    if node.grid is not None:

        plt.imshow(node.grid, cmap='gray')
        plt.title("Occupancy Grid")
        plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```


Yes. Looking at the overall architecture, I think you're trying to add concurrency fixes to a design that wasn't originally built around ownership of state.

The good news is that the algorithms themselves aren't the biggest issue. The main issue is that **every callback can read and write global state directly**.

---

# 1. Separate "state acquisition" from "computation"

Right now callbacks do this:

```python
costmap_callback()
    -> update costmap

frontier_exploration()
    -> read costmap
    -> read pose
    -> read map
    -> compute path
    -> write path

publish_cmd_vel()
    -> read pose
    -> read path
    -> modify path
```

Everything touches everything.

Instead think of your node as:

```text
Subscribers
    ↓
State Store
    ↓
Planner
    ↓
Path Store
    ↓
Controller
```

For example:

```python
class RobotState:
    map
    costmap
    pose
```

```python
class PlannerState:
    path
    frontier_cells
```

Subscribers only update `RobotState`.

Planner only reads `RobotState` snapshots and writes `PlannerState`.

Controller only reads `PlannerState`.

---

# 2. Stop mutating paths

This is the biggest design issue I see.

Currently:

```python
self.world_path.pop(1)
```

is modifying the actual path.

That makes path ownership unclear.

Instead:

```python
self.world_path = [...]
self.path_index = 1
```

Then:

```python
target = self.world_path[self.path_index]
```

and:

```python
self.path_index += 1
```

when a waypoint is reached.

Now the planner owns the path.

The controller only tracks progress through it.

Much cleaner.

---

# 3. Snapshot before computation

This is probably the single biggest improvement.

Right now:

```python
dijkstra()
```

uses:

```python
self.costmap
self.robot_position
self.robot_orientation
```

throughout execution.

A better approach:

```python
planner_input = {
    "costmap": self.costmap.copy(),
    "map": self.map.copy(),
    "robot_position": self.robot_position,
    "robot_orientation": self.robot_orientation,
}
```

Then:

```python
path = compute_path(planner_input)
```

The planner never touches node state again.

This eliminates most race conditions even before adding locks.

---

# 4. Make algorithms pure functions

For example:

Current:

```python
def dijkstra(self, start, frontier_cells):
```

internally reads:

```python
self.costmap
self.robot_position
self.robot_orientation
```

A better version:

```python
def dijkstra(
    costmap,
    origin,
    resolution,
    robot_position,
    robot_orientation,
    start,
    frontier_cells
):
```

Now:

* easy to test
* deterministic
* thread-safe
* reusable

No hidden dependencies.

---

# 5. Use immutable planner outputs

Instead of:

```python
self.world_path
self._grid_path
self._frontier_cells
```

being modified by multiple places:

Planner generates:

```python
PlanningResult(
    world_path,
    grid_path,
    frontier_cells
)
```

and publishes it atomically:

```python
self.current_plan = result
```

Nobody edits it afterward.

---

# 6. Avoid long-running timer callbacks

This:

```python
self.frontier_exploration_timer = self.create_timer(
    0.5,
    self.frontier_exploration
)
```

combined with:

```text
frontier_exploration() took 1.987s
```

is a warning sign.

The callback takes 4× longer than its period.

A common pattern:

```python
if self.planner_running:
    return
```

or:

```python
planner_thread.submit(...)
```

so only one planner runs at a time.

---

# 7. Consider splitting into multiple ROS nodes

This is what many navigation systems eventually do.

For example:

```text
Map Node
    publishes map

Frontier Detector Node
    subscribes map
    publishes frontier goal

Planner Node
    subscribes frontier goal
    publishes path

Controller Node
    subscribes path
    publishes cmd_vel
```

Advantages:

* less shared memory
* fewer race conditions
* easier debugging
* each component has a single responsibility

You don't need to do this immediately, but it's a common evolution once projects grow.

---

# 8. Remove hidden state from utility functions

For example:

```python
calc_distance_rotation()
```

uses:

```python
self.robot_position
self.robot_orientation
```

implicitly.

I would rather see:

```python
calc_distance_rotation(
    robot_position,
    robot_orientation,
    target_x,
    target_y
)
```

Now the function is deterministic.

---

# 9. Reduce the amount of mutable global state

Currently almost everything is:

```python
self.something
```

Examples:

```python
self.map
self.costmap
self.costmap_origin
self.costmap_resolution
self.robot_position
self.robot_orientation
self.world_path
self._grid_path
self._frontier_cells
```

That's a lot of shared mutable state.

A good rule:

> If a variable exists only for one computation, keep it local.

For example:

```python
frontier_cells
grid_path
```

should ideally be local variables until the final result is ready.

---

# If I were refactoring this project

I would aim for:

```text
ROS Callbacks
    ↓
Update shared state
    ↓
Planner copies state snapshot
    ↓
Planner computes path using local variables only
    ↓
Planner publishes immutable path result
    ↓
Controller follows path
```

The key idea is:

**callbacks should mostly move data around, not perform complex logic on shared state.**

Once you start treating `map`, `costmap`, and `pose` as snapshots that get copied into computations, most of the concurrency problems become much easier to reason about, and adding locks becomes a small implementation detail rather than the thing holding the architecture together.


Possibly, yes.

From what you've shown, I don't see a strong reason that this node *needs* concurrency.

Your architecture is essentially:

1. Receive map updates
2. Receive costmap updates
3. Receive pose updates
4. Occasionally compute a path
5. Follow the path

That's a very common robotics pattern that works perfectly well in a single-threaded event loop.

---

## The main question is: how long does planning take?

Your log showed:

```text
frontier_exploration() took 1.987s
```

That's actually the strongest argument *for* concurrency.

If path planning takes ~2 seconds, then during those 2 seconds a `SingleThreadedExecutor` cannot:

* process new costmaps
* process new poses
* process new maps
* publish control commands

So your robot is effectively "blind" while planning.

---

## But there's another solution

Instead of:

```text
Multiple callbacks
        ↓
All sharing state
        ↓
MultiThreadedExecutor
```

you can do:

```text
Callbacks update state
        ↓
Planner takes snapshot
        ↓
Planner computes
        ↓
Replace path
```

with a single-threaded executor.

For example:

```python
def frontier_exploration(self):

    if self.map is None:
        return

    map_snapshot = self.map.copy()
    costmap_snapshot = self.costmap.copy()
    robot_position = self.robot_position

    path = compute_path(
        map_snapshot,
        costmap_snapshot,
        robot_position
    )

    self.world_path = path
```

Now planning operates on a consistent snapshot.

---

## Another possibility: the planner is simply too slow

I noticed:

```python
for ci in range(height):
    for ri in range(width):
```

inside:

```python
find_frontier_cells()
```

and then:

```python
if index_pair not in frontier_cells:
```

which is an O(n) lookup inside nested loops.

That can become surprisingly expensive.

You may find that optimizing the planner gives more benefit than introducing threads.

---

## What I would do first

Run:

```python
from rclpy.executors import SingleThreadedExecutor

executor = SingleThreadedExecutor()
```

for a while.

If:

* the strange hangs disappear
* costmaps keep arriving
* navigation still works

then you've learned something important:

> The architecture doesn't actually need concurrency.

---

## When concurrency becomes worthwhile

I would only add concurrency if:

### Case 1

Planning takes a long time:

```text
100 ms -> probably not
500 ms -> maybe
2 s -> probably yes
```

### Case 2

You have CPU-heavy operations:

* frontier extraction
* path planning
* map processing
* image processing

that can run independently of control.

### Case 3

You split the planner into a worker thread that uses snapshots.

For example:

```text
ROS callbacks (main thread)
        ↓
update state
        ↓
submit planning job
        ↓
planner thread computes
        ↓
returns path
```

This is much safer than allowing multiple ROS callbacks to mutate shared state simultaneously.

---

## If this were my project

I would first refactor toward:

* `SingleThreadedExecutor`
* snapshot-based planning
* immutable paths (no `pop()`)

and get everything stable.

Only after that would I consider:

* a dedicated planning thread
* or a separate planner node

for performance.

In robotics, deterministic behavior is usually more valuable than squeezing out parallelism early. A stable single-threaded design is often easier to debug and extend than a multithreaded design built around shared mutable state.


In that case, concurrency may be justified, but I would still address the design first.

The key observation is that both of these functions are doing **heavy work**:

* `draw_map()` → Matplotlib rendering
* `frontier_exploration()` → frontier detection + Dijkstra planning

Neither of those should be competing with:

* `costmap_callback()`
* `map_callback()`
* `pose_callback()`
* `publish_cmd_vel()`

for execution time.

---

## What I would worry about most

Your planner is reading live state while callbacks are updating it:

```python
self.map
self.costmap
self.costmap_origin
self.costmap_resolution
self.robot_position
self.robot_orientation
```

For example:

```python
frontier_cells = self.find_frontier_cells(self.map)
```

takes potentially seconds.

Meanwhile:

```python
costmap_callback()
```

can execute and replace:

```python
self.costmap
self.costmap_origin
self.costmap_resolution
```

halfway through planning.

That creates inconsistent world views.

Not necessarily a crash.

Just weird behavior.

---

## Better architecture

Think of callbacks as producers and planners/controllers as consumers.

### Callbacks

Only update state:

```python
def costmap_callback(self, msg):
    self.latest_costmap = ...
```

Nothing else.

---

### Planner

When it starts:

```python
with self.state_lock:
    map_snapshot = self.map.copy()
    costmap_snapshot = self.costmap.copy()
    robot_position = self.robot_position
    robot_orientation = self.robot_orientation
```

Then release the lock.

Do all expensive work on the snapshots:

```python
frontier_cells = find_frontier_cells(map_snapshot)

path = dijkstra(
    costmap_snapshot,
    robot_position,
    frontier_cells
)
```

Now planning can take 10 seconds if necessary without blocking updates.

---

## I would actually separate draw_map

Matplotlib is notorious for blocking.

I'd treat visualization as completely separate.

Instead of:

```python
draw_map()
```

sharing live node state,

have it render snapshots:

```python
with self.state_lock:
    map_snapshot = self.map.copy()
    path_snapshot = list(self.world_path)
```

then release the lock.

Render only snapshots.

Never touch ROS state during rendering.

---

## Another thing that stands out

This is expensive:

```python
if index_pair not in frontier_cells:
```

because `frontier_cells` is a list.

For 500 frontier cells:

```text
500 checks
×
500 comparisons
=
250,000 operations
```

and that's inside nested loops.

Use a set:

```python
frontier_cells = set()
```

then:

```python
if index_pair not in frontier_cells:
```

becomes O(1).

---

## Another major bottleneck

This loop:

```python
for ci in range(height):
    for ri in range(width):
```

scans the entire occupancy grid every planning cycle.

If your map is:

```text
400 × 400
```

that's:

```text
160,000 cells
```

every 0.5 seconds.

Then for each cell:

```python
find_neighbouring_cells()
```

and distance calculations.

That's likely where most of your runtime is going.

---

## My preferred solution

I would structure it like this:

```text
ROS callbacks
    ↓
update latest state
    ↓
Planner thread (1 Hz)
    ↓
copies snapshots
    ↓
computes path
    ↓
stores new path
    ↓
Controller timer (10 Hz)
    ↓
follows path
```

Then:

```text
Map callback      fast
Costmap callback  fast
Pose callback     fast
Controller        fast

Planner           slow
Visualizer        slow
```

The slow jobs never interfere with the fast jobs.

---

## Before adding more threads

I'd profile first.

Your logs already show:

```text
frontier_exploration() took 1.987s
```

That's huge.

If you can reduce that to:

```text
100-200 ms
```

through algorithmic improvements (sets, avoiding full-map scans, NumPy operations, frontier clustering, etc.), you may discover you don't need much concurrency at all.

In robotics, a 10× algorithm improvement is usually more valuable than a 2× threading improvement.


- ros2 node info /global_costmap --verbose
