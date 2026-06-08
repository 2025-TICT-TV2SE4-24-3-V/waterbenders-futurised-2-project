from gz.transport import Node
from gz.msgs.twist_pb2 import Twist
from gz.msgs.laserscan_pb2 import LaserScan
from gz.msgs.imu_pb2 import IMU
import time
import math

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, radians, pi

IMU_UPDATE_RATE = 200
LIDAR_UPDATE_RATE = 10

angles = []
distances = []

imu_buffer = []

def quat_euler_angle(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)    
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)    
    return math.atan2(siny_cosp, cosy_cosp)

def lidar_angle(angle_min, measurement_index, angle_increment):
    # Source: https://robotics.stackexchange.com/questions/98107/how-to-get-obstacle-distance-and-angle-by-using-lidar
    return angle_min + (measurement_index * angle_increment) 

def imu_cb(_msg: IMU):

    print("ang_velo", _msg.angular_velocity.z)
    # print("", _msg.orientation)

    global imu_buffer
    orientation_rad = quat_euler_angle(_msg.orientation.x, _msg.orientation.y, _msg.orientation.z, _msg.orientation.w)

    # print("Orientation of Flip in radians: [{}]".format(flip_orientation_rad))

    # Convert the nanoseconds (i.e. 'nsec') to seconds using a factor of 0.000000001 or 1e-9 in scientific notation
    timestamp = _msg.header.stamp.sec + (_msg.header.stamp.nsec *  1e-9)

    imu_buffer.append((timestamp, orientation_rad, 0, _msg.angular_velocity.z))

    # Only store data of the last two seconds
    if len(imu_buffer) > (IMU_UPDATE_RATE * 3):
        imu_buffer.pop(0)

    # print("imu_buffer: ", imu_buffer)

def lasermsg_cb(_msg: LaserScan):

    # Debug proto message type
    # print(type(_msg.header.data[1].value[0]))
    # print(_msg.header.data[1].value[0])

    global angles, distances

    timestamp = _msg.header.stamp.sec + (_msg.header.stamp.nsec * 1e-9)

    """
            'beam_scantime' calculates the amount of time (in seconds) between two consecutive lidar measures.
            In our case the scantime equals 0.0125 seconds, because it takes one second to complete a single scan
            and each second 80 samples are takes, so: 1 / 80 = 0.0125s
    """
    scan_duration = 1 / LIDAR_UPDATE_RATE
    beam_scantime = scan_duration / len(_msg.ranges)

    # Avoid 'UnboundLocalError' error
    # world_angle = 0.0

    skip_samples = 0

    # Calculate each angle corresponding to a measurement (distance)
    for i, sample in enumerate(_msg.ranges):

        if skip_samples > 0:
            skip_samples -= 1
            continue

        # Skip over distance measurements of 'inf'
        if sample == float('inf'):
            continue

        lidar_beam_angle = lidar_angle(_msg.angle_min, i, _msg.angle_step)

        """
            The 'beam_timestamp' variable is simply an estimation of the time at which a scan was taken. 
            Note that '_msg.header.stamp' gives the time at which the scan ended, not began. Hence the minus 'scan_duration'.
        """
        beam_timestamp = (timestamp - scan_duration) + i * beam_scantime

        continue_ = False

        # Use the first value inside 'imu_buffer' to avoid an IndexError
        if len(imu_buffer) <= 1:
            # Small delay to avoid concurrency problems with imu_buffer
            # time.sleep(0.01)
            if not imu_buffer:
                world_angle = lidar_beam_angle
            else:
                world_angle = imu_buffer[0][1] + lidar_beam_angle
        else:
            for j in range(len(imu_buffer) - 1): # The -1 is needed, because of t1 being out of range
                # Yaw of an object is the rotation around the vertical axis (z-axis)
                t0, yaw0, _, ang_v0 = imu_buffer[j]
                t1, yaw1, _, ang_v1 = imu_buffer[j+1]

                # Check for abrupt rotations
                # if (ang_v0 * ang_v1) < 0.0:
                #     skip_samples = 20
                #     continue_ = True
                #     # print("ang_v0", ang_v0)
                #     # print("ang_v1", ang_v1)
                #     # print(ang_v1 * ang_v0)
                #     print("Abrupt rotation detected")
                #     break
                # elif abs(yaw1 - yaw0) >= 0.00135:
                #     continue_ = True
                #     break

                if t0 <= beam_timestamp <= t1:

                    """
                        Needed to observe the difference between two consecutive angles and measure their absolute values
                        Conclusion: normally the difference between two consecutive angles is anywhere between 0.0027 < x < 0.0049
                        unless a rotation occurs, which is when it drops to below 0.002

                        0.00045 < x < 0.00099

                        0.001220 < x < 0.002484
                    """
                    # print("abs_diff", abs(yaw1 - yaw0))
                    # Aply linear interpolation (interpoleren) to estimate the exact angle at 
                    # which the robot was looking at the time the Lidar measurement was taken
                    imu_angle = yaw0 + (((yaw1 - yaw0) / (t1 - t0)) * (beam_timestamp - t0))
                    world_angle = imu_angle + lidar_beam_angle
                    # print("imu_angle", imu_angle)
                    # print("lidar_angle", lidar_beam_angle)
                    # print("world_angle", world_angle)
                    break  # Exit the loop once the interval is found
                else:
                    # Use the last angle saved in imu_buffer
                    world_angle = imu_buffer[-1][1] + lidar_beam_angle

        if continue_ == True:
            continue

        distances.append(float(sample))
        angles.append(float(world_angle))

        # Check whether the Lidar scans from angle_min to angle_max
        # print(angles[0:79])
        # print(lidar_angle(_msg.angle_min, i, _msg.angle_step))

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
    # time.sleep(0.001)
    time.sleep(1)
except KeyboardInterrupt:
  pass

node.unsubscribe(imu_topic)
node.unsubscribe(lidar_topic)

angles = np.array(angles)
distances = np.array(distances)

# Plot the results
x = np.cos(angles) * distances
y = np.sin(angles) * distances

plt.figure(figsize=(6, 6))
plt.plot(x, y, "b.", markersize=1)
plt.axis("equal")
plt.grid(True)
plt.show()

print("Done")
