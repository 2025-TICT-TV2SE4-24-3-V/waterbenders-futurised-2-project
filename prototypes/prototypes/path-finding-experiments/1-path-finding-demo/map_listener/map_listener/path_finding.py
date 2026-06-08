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

        # Temporary variable
        self.points = ([], [])

        self.origin_x = 0.0
        self.origin_y = 0.0
        self.resolution = 0.0

        self.grid_height = 0
        self.grid_height = 0
        # Maybe change to 'data'?
        self.occupancy_grid = None

        self.twist_msg = Twist()
        self.previous_twist_msg = Twist

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) # Listen to /tf and /tf_static

        self.map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10 # Queue size
        )

        self.cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            1 # Only latest command matters
        )

        self.pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10
        )

        # Timer to publish commands (e.g., 10 Hz)
        # self.timer = self.create_timer(0.1, self.publish_cmd_vel)

        # Timer to check robots position
        # self.timer = self.create_timer(0.1, self.robot_position)

    def pose_callback(self, msg: PoseWithCovarianceStamped):

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.get_logger().info(f'Received position: {msg.pose.pose.position.x, msg.pose.pose.position.y}')
        # self.get_logger().info(f'Received orientation: {self.quat_euler_angle(orientation.x, orientation.y, orientation.z, orientation.w)}')

        # Convert real-world coordinates to grid cell (x, y)
        # x = int((real_x - origin_x) / resolution)
        # y = int((real_y - origin_y) / resolution)

        self.points[0].append(int((position.x - self.origin_x) / self.resolution))
        self.points[1].append(int((position.y - self.origin_y) / self.resolution))

        print(self.points[0][-1])
        print(self.points[1][-1])

    def quat_euler_angle(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)    
        cosy_cosp = 1.0 - 2.0 * (y*y + z*z)    
        return math.atan2(siny_cosp, cosy_cosp)

    def robot_position(self):

        # Get position and orientation (angle) of the 'chassis' frame relative to the 'map' frame
        transform = self.tf_buffer.lookup_transform(
                'map', # target_frame
                'chassis', # source_frame (frame whose pose to find relative to target_frame)
                rclpy.time.Time()
        )

        # Extract quaternions and convert to Euler angle
        q = transform.transform.rotation
        orientation = self.quat_euler_angle(q.x, q.y, q.z, q.w)

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        z = transform.transform.translation.z

        # print(x, y, orientation)
        # return x, y, orientation

    # def publish_cmd_vel(self):
    #     # Only publish if the command has changed
    #     if self.previous_twist_msg != self.twist_msg:
    #         self.cmd_vel_pub.publish(self.twist_msg)
    #         self.get_logger().debug(f"Published cmd_vel: linear.x={self.twist_msg.linear.x}, angular.z={self.twist_msg.angular.z}")
    #         self.previous_twist_msg = self.twist_msg  # Update ONLY after successful publish

    def publish_cmd_vel(self):

        self.get_logger().info(f"Published cmd_vel: linear.x={self.twist_msg.linear.x}, angular.z={self.twist_msg.angular.z}")
        self.cmd_vel.publish(self.twist_msg)

        # if self.previous_twist_msg != self.twist_msg:
        #     self.cmd_vel.publish(self.twist_msg)
        #     self.previous_twist_msg = self.twist_msg
        #     print("Succesfully published")

    def frontier_edge_cell(self, grid):

        height, width = grid.shape
        candidate_boundaries = []

        for i_y in range(height):
            for i_x in range(width):
                # Correct way:
                # print(occupancy_grid[i_y, i_x]) 
                current_cell_value = grid[i_y, i_x]
                neighbours = self.find_neighbouring_cells(grid, i_y, i_x)

                for index in neighbours:
                    if (current_cell_value == -1 and grid[index] == 0) or (current_cell_value == 0 and grid[index] == -1):
                        candidate_boundaries.append(index)

                # for value, index in neighbours:
                #     if (current_cell_value == -1 and value == 0) or (current_cell_value == 0 and value == -1):
                #         candidate_boundaries.append(index)

        return candidate_boundaries

        # Values meaning:
        # -1 = unknown
        # 0  = free
        # 100 = occupied

        # The algorithm scans the occupancy grid and identifies all free cells adjacent to unknown cells, resulting in candidate boundaries.

    def find_neighbouring_cells(self, array, i_y, i_x, diagonals=False):

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
            print(i, y, x)
            nx = i_x + x
            ny = i_y + y
            # First check whether each cell has a neighbour (isn't at the edge of the map)
            if 0 <= nx < width and 0 <= ny < height:
                index_pair = ny, nx
                neighbours.append(index_pair)

        return neighbours

        # if i_y > 0:
        #     index_pair = i_y - 1, i_x
        #     neighbours.append((array[index_pair], (index_pair)))
        # if i_y < (height - 1):
        #     index_pair = i_y + 1, i_x
        #     neighbours.append((array[index_pair], (index_pair)))
        # if i_x > 0:
        #     index_pair = i_y, i_x - 1
        #     neighbours.append((array[index_pair], (index_pair)))
        # if i_x < (width - 1):
        #     index_pair = i_y, i_x + 1
        #     neighbours.append((array[index_pair], (index_pair)))
        #
        # return neighbours

        # arr = np.array([
        #     [10, 20, 30, 40],   # Width of 4 and height of 3
        #     [50, 60, 70, 80],
        #     [90, 100, 110, 120]
        # ])

        # 100
        # [(60, (1, 1)), (90, (2, 0)), (110, (2, 2))]

    def breadth_first_search(self, occupancy_grid):

        # frontier = Queue()
        # frontier.put()
        # reached = set()

        return

    def map_callback(self, msg: OccupancyGrid):
        # self.get_logger().info('Received map')

        self.grid_width = msg.info.width
        self.grid_height = msg.info.height

        self.occupancy_grid = np.array(msg.data).reshape((self.grid_height, self.grid_width))

        arr = np.array([
            [10, 20, 30, 40],   # Width of 4 and height of 3
            [50, 60, 70, 80],
            [90, 100, 110, 120]
        ])

        # The origin of the map is located at the bottom left corner and changes as the robot moves further into unknown territory
        # print(msg.info.origin.position.x, msg.info.origin.position.y)
        # self.get_logger().info(f'Map origin: {msg.info.origin.position.x, msg.info.origin.position.y}')
        # print(msg.info.resolution)

        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution

        new = self.find_neighbouring_cells(self.occupancy_grid, 224, 166)

        # print(self.occupancy_grid.ndim)
        # print(self.occupancy_grid)
        # Values are inverted, so it becomes: y, x
        # print(arr[2, 1]) # Returns 100
        # # print(self.find_neighbouring_cells(arr, 2, 1))
        # new = self.find_neighbouring_cells(arr, 2, 1, True)
        print(new)
        for x in new:
            print(self.occupancy_grid[x])

        # print(self.robot_position())

        # self.twist_msg = 

        # twist_msg = Twist()
        # self.twist_msg.linear.x = 0.2
        #
        # self.cmd_vel.publish(twist_msg)

        # self.frontier_edge_cell(occupancy_grid=self.occupancy_grid)

def main(args=None):
    rclpy.init(args=args)
    node = MapAnalyzer()

    try:
        rclpy.spin(node)  # Spin until Ctrl+C
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully

    # After Ctrl+C, plot the latest map
    if node.occupancy_grid is not None:

        plt.imshow(node.occupancy_grid, cmap='gray')
        plt.plot(node.points[0], node.points[1], marker="^", color="r")
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
