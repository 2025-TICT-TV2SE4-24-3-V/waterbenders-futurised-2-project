import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import numpy as np
from tf2_ros import Buffer, TransformListener
import math
from queue import Queue
import matplotlib.pyplot as plt

class MapAnalyzer(Node):
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

        # self.twist_msg = Twist()

        self.relative_distance = 0.0
        self.relative_angle = 0.0

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
            # 1 # Only latest command matters
        )

        self.pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10
        )

        # Timer to publish to cmd_vel topic(e.g., 10 Hz)
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)

    def pose_callback(self, msg: PoseWithCovarianceStamped):

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # self.get_logger().info(f'Received position: {position.x, position.y}')
        # self.get_logger().info(f'Received orientation: {self.quat_euler_angle(orientation.x, orientation.y, orientation.z, orientation.w)}')
        print(position.x, position.y)

        self.robot_position = (position.x, position.y)
        self.robot_orientation = self.quat_euler_angle(orientation.x, orientation.y, orientation.z, orientation.w)

    def quat_euler_angle(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)    
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z)    
        return math.atan2(siny_cosp, cosy_cosp)

    # def robot_position(self):
    #     return

    def p_controller(self, setpoint, Kp, value=0.0):

        error = setpoint - value

        return Kp * error

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


    def pd_controller(self, error, kp, kd, dt):
        # Store previous error for derivative term
        prev_error = getattr(self, "_prev_error", 0.0)
        derivative = (error - prev_error) / dt
        self._prev_error = error

        # PD output
        output = kp * error + kd * derivative

        # Optional: Clamp to avoid extreme values
        output = max(-0.5, min(0.5, output))
        return output

    def publish_cmd_vel(self):

        twist_msg = Twist()

        # Check whether world_path exists
        if self.world_path is None: # Using 'is None' is faster then checking with ==
            self.get_logger().info('No path exists yet')
            return

        if len(self.world_path) <= 1:
            self.get_logger().info('Path is too short or already completed')
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.cmd_vel.publish(twist_msg)
            return

        # Extract the second coordinate from the path 

        # and remove its 
        # value, so that each function call it will move along the planned path
        x, y = self.world_path[1]
        print(x, y)
        print("Final goal (x, y): ", self.world_path[-1])
        # self.world_path.pop(1)

        relative_distance, relative_angle = self.calc_distance_rotation(x, y)

        if relative_distance < 0.2: # Stop when below threshold
            self.world_path.pop(1) # Remove the coordinates when reached,
            # so that each function call it will move along the planned path
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
        else:
            # twist_msg.angular.z = self.p_controller(relative_angle, 0.5)
            # twist_msg.linear.x = self.p_controller(relative_distance, 1.0)

            # twist_msg.angular.z = self.p_controller(relative_angle, 0.3)
            twist_msg.angular.z = self.pd_controller(relative_angle, kp=0.3, kd=0.1, dt=0.2)
            twist_msg.angular.z = max(-0.5, min(0.5, twist_msg.angular.z)) # Clamp to [-0.5, 0.5]

            # twist_msg.linear.x = 0.2  # Fixed forward speed (adjust as needed)

            # Linear speed decreases as distance shrinks
            linear_speed = 0.2 * (1 - min(1.0, relative_distance / 0.5))  # Max speed at 0.5m away
            twist_msg.linear.x = max(0.05, linear_speed)  # Never go below 0.05 m/s

            # # Option 2: Proportional to remaining distance (slow down as you approach)
            # twist_msg.linear.x = 0.5 * (0.0 - relative_distance)  # Negative to move toward target
            # twist_msg.linear.x = max(0.0, min(0.5, twist_msg.linear.x))  # Clamp to [0.0, 0.5]

        self.cmd_vel.publish(twist_msg)
        # self.get_logger().info(f"Published cmd_vel: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}")

        return
        
    # Convert real-world coordinates to grid cell (x, y)
    def world_grid_coordinates(self, ix, iy):

        grid_x = round((ix - self.grid_origin[0]) / self.grid_resolution)
        grid_y = self.grid_height - 1 - round((iy - self.grid_origin[1]) / self.grid_resolution)

        # Clamp to grid bounds
        grid_x = max(0, min(self.grid_width - 1, grid_x))
        grid_y = max(0, min(self.grid_height - 1, grid_y))

        # grid_x = int((ix - self.grid_origin[0]) / self.grid_resolution)
        # grid_y = self.grid_height - 1 - int((iy - self.grid_origin[1]) / self.grid_resolution)

        return grid_y, grid_x

    # Convert a grid cell (x, y) to real-world coordinates
    def grid_world_coordinates(self, iy, ix):

        world_x = self.grid_origin[0] + ix * self.grid_resolution
        world_y = self.grid_origin[1] + (self.grid_height - 1 - iy) * self.grid_resolution

        return world_x, world_y

    def find_neighbouring_cells(self, array, iy, ix, diagonals=False):

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
            nx = ix + x
            ny = iy + y
            # Check whether each cell has a neighbour (isn't at the edge of the map)
            if 0 <= nx < width and 0 <= ny < height:
                index_pair = ny, nx # x and y are reversed so (iy, ix) is the correct way
                neighbours.append(index_pair)

        return neighbours

    # Scans the occupancy grid and identifies all free cells (0) adjacent to unknown cells (-1), resulting in frontier edge cells or candidate boundaries
    def candidate_boundaries(self, grid):

        height, width = grid.shape
        candidate_boundaries = []

        for iy in range(height):
            for ix in range(width):

                if grid[iy, ix] == 0:
                    neighbours = self.find_neighbouring_cells(grid, iy, ix, diagonals=False)

                    for ni in neighbours:
                        # Check if neighbouring cell is unknown (-1) and coordinates 
                        # haven't already been added to 'candidate_boundaries' (avoids duplicates)
                        if grid[ni] == -1 and ni not in candidate_boundaries:
                            candidate_boundaries.append(ni)

        return candidate_boundaries

    # Source: https://www.redblobgames.com/pathfinding/a-star/introduction.html
    # Scans grid and returns the first step in the direction of the closest valid edge cell (candidate boundary)
    def breadth_first_search(self, grid, start: tuple):

        # current = None

        # TODO: Simplify function (if needed) to only return the closest (not keep a dict of all pathways)

        edge_cells = self.candidate_boundaries(grid)

        if not edge_cells:
            self.get_logger().warn("No frontier cells found")

        frontier = Queue()
        frontier.put(start)
        reached = {}
        reached[start] = None

        while not frontier.empty():

            if not edge_cells:
                break

            current = frontier.get()
            neighbours = self.find_neighbouring_cells(grid, current[0], current[1])

            # Exit the loop prematurely when the closest frontier 
            # edge cell has been reached and return its value
            if current in edge_cells:
                edge_cells.remove(current)

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

        # print("Reached: ", reached)
        # print("Values of reached: ", [grid[ni] for ni, _ in reached.items()])

        # return path[-2]
        return path
        
    def map_callback(self, msg: OccupancyGrid):

        self.get_logger().info('Received map')
        # self.get_logger().info(f'Map origin: {msg.info.origin.position.x, msg.info.origin.position.y}')

        self.grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        # The origin of the map is located at the bottom left corner and changes as the robot moves further into unknown territory
        self.grid_origin = (msg.info.origin.position.x, msg.info.origin.position.y) # (x, y)
        self.grid_resolution = msg.info.resolution
        self.grid_width = msg.info.width
        self.grid_height = msg.info.height

        # print("Origin", self.grid_origin)
        # print("Resolution", self.grid_resolution)
        # print("Width", self.grid_width)
        # print("Height", self.grid_height)

        # Additional check to determine whether the robot hasn't moved (i.e. a /pose msg hasn't been sent out yet)
        if self.robot_position is None:
            self.get_logger().warn("Robot position unknown")
            return

       # Skip if the robot is outside the current map bounds
        # robot_x, robot_y = self.robot_position
        # if (robot_x < self.grid_origin[0] or # Left of map
        #     robot_x > self.grid_origin[0] + self.grid_width * self.grid_resolution or # Right of map
        #     robot_y < self.grid_origin[1] or # Below map
        #     robot_y > self.grid_origin[1] + self.grid_height * self.grid_resolution): # Above map
        #     self.get_logger().warn("Robot is outside the current map bounds. Waiting for map update...")
        #     return

        # Skip if a path is still being executed
        if self.world_path is not None and len(self.world_path) > 1:
            self.get_logger().warn("MAP CALLBACK: Old path hasn't been completed yet")
            return
        else:
            self.get_logger().warn("MAP CALLBACK: new path is being calculated")

        grid_position = self.world_grid_coordinates(self.robot_position[0], self.robot_position[1])

        # cell_y, cell_x = self.breadth_first_search(self.grid, grid_position)
        grid_path = self.breadth_first_search(self.grid, grid_position)

        self.get_logger().info(f"Grid path length: {len(grid_path)}, path: {grid_path[:5]}")

        self.world_path = [self.grid_world_coordinates(cell_y, cell_x) for (cell_y, cell_x) in grid_path]

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
    
    assert node.find_neighbouring_cells(arr, 0, 0) == [(1, 0), (0, 1)], "4-way neighbours don't match"
    assert node.find_neighbouring_cells(arr, 0, 0, True) == [(1, 0), (0, 1), (1, 1)], "Diagonal neighbours don't match"

    assert node.candidate_boundaries(arr) == [(2, 0), (2, 3)], "Candidate boundaries are invalid"

    assert node.breadth_first_search(arr, (0, 0)) == [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (2, 3)], "Custom breadth first search invalid"
    # assert node.breadth_first_search(arr, (0, 0)) == [(0, 0), (1, 0), (2, 0)], "Custom breadth first search invalid"

    try:
        rclpy.spin(node)  # Spin until Ctrl+C
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully

    # After Ctrl+C, plot the latest map
    if node.grid is not None:

        plt.imshow(node.grid, cmap='gray')
        # plt.plot(node.points[0], node.points[1], marker="^", color="r")
        plt.title("Occupancy Grid")
        plt.show()  # Blocking call to display the map

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""

    Convert a grid cell (x, y) to real-world coordinates:
    real_x = origin_x + x * resolution
    real_y = origin_y + y * resolution

    Convert real-world coordinates to grid cell (x, y)
    x = int((real_x - origin_x) / resolution)
    y = int((real_y - origin_y) / resolution)

"""

"""

    The origin in the Gazebo simulation is always located in the bottom left corner, so where x, y are both minus (-x, -y)

"""
#
# Is it because /pose returns a position of the robot that can be out of bounds , becuase the map is generated with the lidar that's on top meaning if the lidar hasn't explored aall regions the robot wiall always be out of bounds? If this is the case what can I do about it in the code?
