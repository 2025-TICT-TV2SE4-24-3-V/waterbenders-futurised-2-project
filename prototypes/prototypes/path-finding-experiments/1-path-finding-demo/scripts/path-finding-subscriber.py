from gz.transport import Node
from gz.msgs.twist_pb2 import Twist
from gz.msgs.laserscan_pb2 import LaserScan
# from gz.msgs.laser_scan_pb2 import LaserScan
import time

def lasermsg_cb(msg: LaserScan):
    print("Received Lidar sensor data: [{}]".format(msg.vertical_count))
 
# create a transport node
node = Node()
lidar_topic = "/lidar"
 
# subscribe to a topic by registering a callback
if node.subscribe(LaserScan, lidar_topic, lasermsg_cb):
    print("Subscribing to type {} on topic [{}]".format(
        LaserScan, lidar_topic))
else:
    print("Error subscribing to topic [{}]".format(lidar_topic))

# wait for shutdown
try:
  while True:
    time.sleep(0.001)
except KeyboardInterrupt:
  pass
print("Done")
