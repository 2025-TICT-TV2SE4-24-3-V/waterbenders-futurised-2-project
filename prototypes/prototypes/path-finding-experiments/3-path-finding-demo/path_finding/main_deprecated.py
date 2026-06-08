import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import numpy as np
from tf2_ros import Buffer, TransformListener
import math
from queue import Queue, PriorityQueue
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
import time
from threading import Lock

class MapAnalyzer(Node):

    # Constants
    DISTANCE_THRESHOLD = 0.75
    OBJECT_THRESHOLD = 1.0

    def __init__(self):
        super().__init__('path_finding')

        self.map = None

        self.costmap_lock = Lock()
        
        self.costmap = None

        self.costmap_origin = None
        self.costmap_resolution = 0.0

        self.costmap_width = 0
        self.costmap_height = 0

        self.costmap_updated = False

        self.robot_position = None
        self.robot_orientation = 0.0

        self.world_path = None

        self._grid_path = None
        self._frontier_cells = None

        self.prev_error = 0.0

        self.object_nearby = False

        # Live mapping of path-finding data
        plt.ion() # Enable interactive plotting
        self.fig, self.ax = plt.subplots()
        self.cmap = ListedColormap(['white', 'white', 'black'])
        self.norm = BoundaryNorm([-1.5, -0.5, 0.5, 100.5], self.cmap.N)

        self.subcriber_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.compute_cb_group = MutuallyExclusiveCallbackGroup()

        self.map_2d = self.create_subscription(
            OccupancyGrid,
            '/map_2d',
            self.map_callback,
            10, # Queue size
            callback_group=self.subcriber_cb_group 
        )

        self.global_costmap = self.create_subscription(
            OccupancyGrid,
            '/costmap',
            self.costmap_callback,
            10,
            callback_group=self.subcriber_cb_group
        )

        self.pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10,
            callback_group=self.subcriber_cb_group
        )

        # self.lidar = self.create_subscription(
        #     LaserScan,
        #     '/scan',
        #     self.lidar_callback,
        #     10,
        #     callback_group=self.subcriber_cb_group
        # )

        self.cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Timer to publish to cmd_vel topic(e.g., 10 Hz)
        self.cmd_vel_timer = self.create_timer(0.1, self.publish_cmd_vel, callback_group=self.timer_cb_group)

        # self.draw_map_timer = self.create_timer(0.2, self.draw_map, callback_group=self.timer_cb_group)

        self.frontier_exploration_timer = self.create_timer(0.5, self.frontier_exploration, callback_group=self.compute_cb_group)

    def draw_map(self):

        # TEST
        start = time.time()

        if self.map is None or self.costmap is None:
            return

        self.ax.clear()
        # Source: https://matplotlib.org/stable/users/explain/artists/imshow_extent.html#imshow-extent
        self.ax.imshow(self.map, cmap=self.cmap, norm=self.norm,
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

        if self._grid_path:
            y = [y for y, _ in self._grid_path]
            x = [x for _, x in self._grid_path]
            self.ax.scatter(x, y, s=50, c='green', alpha=0.5)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

        # TEST
        elapsed = time.time() - start    
        self.get_logger().info(f'draw_map took {elapsed:.3f}s')

    def map_callback(self, msg: OccupancyGrid):

        # Debug
        self.get_logger().info('Received map')

        self.map = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        return

    def costmap_callback(self, msg: OccupancyGrid):

        # Debug
        self.get_logger().info('Received costmap')

        self.costmap = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        # The origin in the Gazebo simulation is always located in the bottom left corner, 
        # so where x, y are both minus (-x, -y). It changes as the robot moves
        # further into unknown territory
        self.costmap_origin = (msg.info.origin.position.x, msg.info.origin.position.y) # (x, y)
        self.costmap_resolution = msg.info.resolution
        self.costmap_width = msg.info.width
        self.costmap_height = msg.info.height

        self.costmap_updated = True

        # # Check whether map exists before drawing the occupancy grid
        # if self.map is not None:
        #     self.draw_map()

        return

    def frontier_exploration(self):

        self.get_logger().info("frontier_exploration tick")

        # TEST
        start = time.time()

        # The below if-statements are safety checks to avoid concurrency problems

        # Costmap hasn't been updated or received 
        if not self.costmap_updated:
            return

        self.costmap_updated = False

        # Occupancy grid hasn't been published
        if self.map is None:
            self.get_logger().warn("Occupancy grid doesn't exist")
            return

        # Robot hasn't moved yet (/pose msg hasn't been sent out yet)
        if self.robot_position is None:
            self.get_logger().warn("Robot position unknown")
            return

        # if self.world_path is not None:
        #     # TODO: Use grid_path directly
        #     ci, ri = self.world_path[-1]
        #     if self.map[int(ci), int(ri)] == -1:
        #         self.get_logger().warn("Final path finding objective hasn't been explored yet")
        #         return

        # Path isn't finished completely
        if self.world_path is not None:
            if len(self.world_path) > 1:
                self.get_logger().warn("Current path hasn't been fully completed")
                return

        # Determine frontier cells
        self.get_logger().info("Determining frontier cells")

        frontier_cells = self.find_frontier_cells(self.map)
        self._frontier_cells = frontier_cells.copy()

        if len(frontier_cells) == 0:
            self.get_logger().warn("No frontier cells found")
            return

        # Debug
        self.get_logger().info(f"Frontier cells found: {len(frontier_cells)}")

        # Calculate a new path
        self.get_logger().info("New path is being calculated")

        starting_position = self.world_grid_coordinates(self.robot_position[0], self.robot_position[1])

        grid_path = self.dijkstra(starting_position, frontier_cells)
        self._grid_path = grid_path.copy()
        self.get_logger().info(f"Grid path: {grid_path}")

        self.world_path = [self.grid_world_coordinates(ci, ri) for (ci, ri) in grid_path]

        # Debug
        # self.get_logger().warn(f"Final grid goal (x, y): {grid_path[-1]}")
        # self.get_logger().warn(f"Final goal (x, y): {self.world_path[-1]}")

        # TEST
        elapsed = time.time() - start    
        self.get_logger().info(f'frontier_exploration() took {elapsed:.3f}s')

        return

    def lidar_callback(self, msg: LaserScan):

        # self.get_logger().info(f"Lidar data samples amount: {len(msg.ranges)}")

        middle_index = int(len(msg.ranges) / 2)
        lidar_object_range = int((len(msg.ranges) / 360) * 120 / 2)

        for i in range(middle_index - lidar_object_range, middle_index + lidar_object_range):
            if msg.ranges[i] < self.OBJECT_THRESHOLD:
                self.get_logger().info(f"Minimal distance: {msg.ranges[i]}")
                self.object_nearby = True

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

        self.get_logger().info("publish_cmd_vel tick")

        twist_msg = Twist()

        # Check whether world_path exists
        if self.world_path is None: # Using 'is None' is faster then checking with ==
            # self.get_logger().info('No path exists yet')
            return

        if len(self.world_path) <= 1:
            self.get_logger().info('Final goal reached')
            self.get_logger().info(f'Test: {len(self.world_path)}')

            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.cmd_vel.publish(twist_msg)

            self.world_path = None

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
            twist_msg.angular.z = 0.0 # Stop turning completely
            twist_msg.linear.x = 0.4

            # Debug messages
            self.get_logger().warn("Subgoal reached")

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
        ci = int((iy - self.costmap_origin[1]) / self.costmap_resolution)
        ri = int((ix - self.costmap_origin[0]) / self.costmap_resolution)

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

        # Don't use int() like in world_grid_coordinates, because else calculations in publish_cmd_vel() don't work anymore
        world_x = (ri * self.costmap_resolution) + self.costmap_origin[0]
        world_y = (ci * self.costmap_resolution) + self.costmap_origin[1]

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
    def dijkstra(self, start: tuple, frontier_cells: list):

        frontier = PriorityQueue()
        frontier.put((0, start))
        reached = {}
        costs = {}
        reached[start] = None
        costs[start] = 0

        while not frontier.empty():

            _, current = frontier.get()
            neighbours = self.find_neighbouring_cells(self.costmap, current[0], current[1])

            # Exit the loop prematurely when a frontier cell has been reached
            if current in frontier_cells:
                frontier_cells.remove(current) # Redundant?
                break

            for ni in neighbours:

                world_x, world_y = self.grid_world_coordinates(ni[0], ni[1])
                rotations_cost = abs(self.calc_distance_rotation(world_x, world_y)[1])
                new_cost = costs[current] + rotations_cost

                if ni not in reached or new_cost < costs[ni]: 
                    if self.costmap[ni] != 100:
                        costs[ni] = new_cost
                        frontier.put((new_cost, ni)) # Priority, value
                        reached[ni] = current

        path = []
        while current != start:
            path.append(current)
            current = reached[current]
        path.append(start)
        path.reverse()

        return path

def main(args=None):
    rclpy.init(args=args)
    node = MapAnalyzer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    """
    TESTS
    """

    # Values meaning:
    # -1 = unknown
    # 0  = free
    # 100 = occupied

    # Width of 4 and height of 3
    # arr = np.array([ 
    #     [0, 100, 100, -1], 
    #     [0, 0, 0, 100],
    #     [-1, 0, 0, -1]
    # ])
    
    # node.get_logger().info(f"{node.find_neighbouring_cells(arr, 0, 0)}")

    # assert node.find_neighbouring_cells(arr, 0, 0) == [(1, 0), (0, 1)], "4-way neighbours don't match"

    # assert node.find_neighbouring_cells(arr, 0, 0, True) == [(1, 0), (0, 1), (1, 1)], "Diagonal neighbours don't match"

    # assert node.candidate_boundaries(arr) == [(1, 0), (2, 1), (2, 2)], "Candidate boundaries are invalid"

    # assert node.breadth_first_search(arr, (0, 0)) == [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (2, 3)], "Custom breadth first search invalid"
