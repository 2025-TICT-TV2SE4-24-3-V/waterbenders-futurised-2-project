import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt

class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        self.latest_map = None  # Store the latest map data
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10 #
        )

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info('Received map')
        self.latest_map = msg  # Store the latest map

        # Values meaning:
        # -1 = unknown
        # 0  = free
        # 100 = occupied
    

def main(args=None):
    rclpy.init(args=args)
    node = MapSubscriber()

    try:
        rclpy.spin(node)  # Spin until Ctrl+C
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully

    # After Ctrl+C, plot the latest map
    if node.latest_map is not None:
        msg = node.latest_map
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))

        plt.imshow(data, cmap='gray')
        plt.title("Occupancy Grid")
        plt.show()  # Blocking call to display the map

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
