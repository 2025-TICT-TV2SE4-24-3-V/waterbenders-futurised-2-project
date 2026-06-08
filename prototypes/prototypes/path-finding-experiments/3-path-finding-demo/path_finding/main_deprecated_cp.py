from sensor_msgs.msg import LaserScan
import numpy as np
from tf2_ros import Buffer, TransformListener
import math
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


        # self.lidar = self.create_subscription(
        #     LaserScan,
        #     '/scan',
        #     self.lidar_callback,
        #     10,
        #     callback_group=self.subcriber_cb_group
        # )



        self.frontier_exploration_timer = self.create_timer(0.5, self.frontier_exploration, callback_group=self.compute_cb_group)

    def draw_map(self):

        # TEST
        start = time.time()

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



    def frontier_exploration(self):

        self.get_logger().info("frontier_exploration tick")

        # TEST
        start = time.time()

        # Costmap hasn't been updated or received 
        if not self.costmap_updated:
            return

        self.costmap_updated = False


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

    def publish_cmd_vel(self):

        if len(self.world_path) <= 1:
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


        return
        
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
