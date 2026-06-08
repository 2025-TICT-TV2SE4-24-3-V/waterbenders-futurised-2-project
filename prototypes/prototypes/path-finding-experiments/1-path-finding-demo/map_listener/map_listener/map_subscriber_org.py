import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
import numpy as np

import matplotlib.pyplot as plt

class MapSubscriber(Node):

    def __init__(self):
        super().__init__('map_subscriber')

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info('Received map')

        # Convert flat array → 2D grid
        width = msg.info.width
        height = msg.info.height

        data = np.array(msg.data).reshape((height, width))

        # Metadata
        resolution = msg.info.resolution
        origin = msg.info.origin.position

        print("Map size:", width, "x", height)
        print("Resolution:", resolution)
        print("Origin:", origin.x, origin.y)

        # Example: inspect a cell
        print("Cell (0,0):", data[0][0])

        # Values meaning:
        # -1 = unknown
        # 0  = free
        # 100 = occupied

        plt.imshow(data, cmap='gray')
        plt.title("Occupancy Grid")
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)

    node = MapSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
