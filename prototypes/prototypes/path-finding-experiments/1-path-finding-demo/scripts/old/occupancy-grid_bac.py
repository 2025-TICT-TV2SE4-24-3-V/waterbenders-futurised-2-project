from gz.transport import Node
from gz.msgs.twist_pb2 import Twist
from gz.msgs.laserscan_pb2 import LaserScan
from gz.msgs.imu_pb2 import IMU
import time
import math

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, radians, pi

"""

TODO:
    - Skip float('inf') values

"""

flip_orientation_rad = 0

angles = []
distances = []

def quat_euler_angle(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)    
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)    
    return math.atan2(siny_cosp, cosy_cosp)

def lidar_angle(angle_min, measurement_index, angle_increment):
    # Source: https://robotics.stackexchange.com/questions/98107/how-to-get-obstacle-distance-and-angle-by-using-lidar
    return angle_min + (measurement_index * angle_increment) 

def imu_cb(_msg: IMU):
    global flip_orientation_rad
    flip_orientation_rad = quat_euler_angle(_msg.orientation.x, _msg.orientation.y, _msg.orientation.z, _msg.orientation.w)
    print("Orientation of Flip in radians: [{}]".format(flip_orientation_rad))

    # print("Current orientation in...: [{}]".format(quat_euler_angle(_msg.orientation.x, _msg.orientation.y, _msg.orientation.z, _msg.orientation.w)))
    # time.sleep(5) # Wait 5 seconds

def lasermsg_cb(_msg: LaserScan):

    # Debug proto message type
    # print(type(_msg.header.data[1].value[0]))
    # print(_msg.header.data[1].value[0])

    temp_dict = {}
    global angles, distances

    # '_msg.header.data[1].value' is the measurement number
    temp_dict[_msg.header.data[1].value[0]] = []
    # print(len(_msg.ranges)) = 360

    # Calculate each angle corresponding to a measurement (distance)
    for i, sample in enumerate(_msg.ranges):
        # Skip over distance measurements of 'inf'
        if sample == float('inf'):
            continue

        # print(lidar_angle(_msg.angle_min, i, _msg.angle_step))
        # print(lidar_angle(_msg.angle_min, i, _msg.angle_step), sample)

        # The '+' calculates the angle relative to the world, by taking into account the orientation of the robot
        world_angle = flip_orientation_rad + lidar_angle(_msg.angle_min, i, _msg.angle_step) 
        distances.append(float(sample))
        angles.append(float(world_angle))
        temp_dict[_msg.header.data[1].value[0]].append((sample, world_angle))

    median = (temp_dict[_msg.header.data[1].value[0]][179][1] + temp_dict[_msg.header.data[1].value[0]][180][1]) / 2
    print(median)
    print(temp_dict[_msg.header.data[1].value[0]][359][1])

# create a transport node
node = Node()
lidar_topic = "/lidar"
imu_topic = "/imu"

# subscribe to a topic by registering a callback
if node.subscribe(IMU, imu_topic, imu_cb):
    print("Subscribing to type {} on topic [{}]".format(
        LaserScan, lidar_topic))
else:
    print("Error subscribing to topic [{}]".format(lidar_topic))
 
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

angles = np.array(angles)
distances = np.array(distances)

ox = np.sin(angles) * distances
oy = np.cos(angles) * distances
plt.figure(figsize=(6,10))
plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-") # lines from 0,0 to the
plt.axis("equal")
bottom, top = plt.ylim()  # return the current ylim
plt.ylim((top, bottom)) # rescale y axis, to match the grid orientation
plt.grid(True)
plt.show()

print("Done")
