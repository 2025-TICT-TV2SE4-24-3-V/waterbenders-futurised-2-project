from gz.transport import Node
from gz.msgs.twist_pb2 import Twist
from gz.msgs.laserscan_pb2 import LaserScan
from gz.msgs.imu_pb2 import IMU

# Twist messages to obtain odometry data of the robots xyz values
from gz.msgs.odometry_pb2 import Odometry

import time
import math

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, radians, pi

IMU_UPDATE_RATE = 200
LIDAR_UPDATE_RATE = 10

MAX_ANGULAR_VELOCITY = 0.5 # Rad/s

angles = []
distances = []

imu_buffer = []

odometry_xy = ()
current_distance = 0.0

linear_x = 0.0

def quat_euler_angle(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)    
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)    
    return math.atan2(siny_cosp, cosy_cosp)

def lidar_angle(angle_min, measurement_index, angle_increment):
    # Source: https://robotics.stackexchange.com/questions/98107/how-to-get-obstacle-distance-and-angle-by-using-lidar
    return angle_min + (measurement_index * angle_increment) 

# Yaw of an object is the rotation around the vertical axis (z-axis)
def interpolate_imu_yaw(timestamp):

    """
        Needed to observe the difference between two consecutive angles and measure their absolute values
        Conclusion: normally the difference between two consecutive angles is anywhere between 0.0027 < x < 0.0049
        unless a rotation occurs, which is when it drops to below 0.002

        0.00045 < x < 0.00099

        0.001220 < x < 0.002484
    """

    # Use the first value inside 'imu_buffer' to avoid an IndexError
    if len(imu_buffer) <= 1:
        if not imu_buffer:
            return 0.0
        else:
            return imu_buffer[0][1]

    for i in range(len(imu_buffer) - 1): # The -1 is needed, because of t1 being out of range
        t0, yaw0, _, ang_v0 = imu_buffer[i]
        t1, yaw1, _, ang_v1 = imu_buffer[i+1]

        # Calculate the threshold based on the difference in time between two IMU 
        # measurements and the maximum angular velocity (in rad/s)
        angular_threshold = (MAX_ANGULAR_VELOCITY * (t1 - t0)) / 2

        # Dicard measurements when an abrupt change in rotation direction occured. Also check against linear_x, because ang_v can default to a minus value
        if (ang_v0 * ang_v1) < 0.0 and -0.4 < linear_x < 0.4:
            return "abrupt_rotation_detected"

        if t0 <= timestamp <= t1:

            # Check yaw difference only for the two IMU angles closest to the timestamp instead
            # of disregarding every angle in imu_buffer when one angle difference exceeds the threshold.
            if abs(yaw1 - yaw0) >= angular_threshold:
                return "rapid_rotation_detected"
            # print("diff", yaw1 - yaw0)

            # Aply linear interpolation (interpoleren) to estimate the exact angle at 
            # which the robot was looking at the time the Lidar measurement was taken
            imu_angle = yaw0 + (((yaw1 - yaw0) / (t1 - t0)) * (timestamp - t0))
            return imu_angle

    # Else use the last angle saved in imu_buffer
    # return imu_buffer[-1][1]

    return "no_angle_found"

def odom_cb(_msg: Odometry):

    global odometry_xy, current_distance, linear_x
    odometry_xy = (_msg.pose.position.x, _msg.pose.position.y)
    current_distance = math.sqrt(odometry_xy[0]**2 + odometry_xy[1]**2)
    robot_x, robot_y = odometry_xy

    linear_x = _msg.twist.linear.x
    # print("linear_x", linear_x)

def imu_cb(_msg: IMU):

    # print("IMU orientation of Flip (x, y, z):")
    # print(_msg.orientation.x) 
    # print(_msg.orientation.y)
    # print(_msg.orientation.z)

    global imu_buffer
    orientation_rad = quat_euler_angle(_msg.orientation.x, _msg.orientation.y, _msg.orientation.z, _msg.orientation.w)

    # print("Orientation of Flip in radians: [{}]".format(flip_orientation_rad))

    # Convert the nanoseconds (i.e. 'nsec') to seconds using a factor of 0.000000001 or 1e-9 in scientific notation
    timestamp = _msg.header.stamp.sec + (_msg.header.stamp.nsec *  1e-9)

    imu_buffer.append((timestamp, orientation_rad, 0.0, _msg.angular_velocity.z))

    # Only store data of the last three seconds
    if len(imu_buffer) > (IMU_UPDATE_RATE * 3):
        imu_buffer.pop(0)

def lidar_cb(_msg: LaserScan):

    global angles, distances

    timestamp = _msg.header.stamp.sec + (_msg.header.stamp.nsec * 1e-9)

    """
            'beam_scantime' calculates the amount of time (in seconds) between two consecutive lidar measures.
            In our case the scantime equals 0.0125 seconds, because it takes one second to complete a single scan
            and each second 80 samples are takes, so: 1 / 80 = 0.0125s
    """
    scan_duration = 1 / LIDAR_UPDATE_RATE
    beam_scantime = scan_duration / len(_msg.ranges)

    skip_samples = 0

    # Calculate each angle corresponding to a measurement (distance)
    for i, distance in enumerate(_msg.ranges):
        # Skip over distance measurements of 'inf'
        if distance == float('inf'):
            continue

        if skip_samples > 0:
            skip_samples -= 1
            continue

        """
            The 'beam_timestamp' variable is simply an estimation of the time at which a scan was taken. 
            Note that '_msg.header.stamp' gives the time at which the scan ended, not began. Hence the minus 'scan_duration'.
        """
        beam_timestamp = (timestamp - scan_duration) + i * beam_scantime
        lidar_beam_angle = lidar_angle(_msg.angle_min, i, _msg.angle_step)

        imu_angle = interpolate_imu_yaw(beam_timestamp)

        if imu_angle == "rapid_rotation_detected" or imu_angle == "no_angle_found":
            continue
        elif imu_angle == "abrupt_rotation_detected":
            skip_samples = 20
            continue

        """
            TESTS:
        """
        robot_x, robot_y = odometry_xy
        print("robot_x, robot_y", robot_x, robot_y)

        sample_distance_x = distance * math.cos(lidar_beam_angle)
        sample_distance_y = distance * math.sin(lidar_beam_angle)
        # print("test", distance, math.sqrt(sample_distance_x**2 + sample_distance_y**2))

        # Source: https://en.wikipedia.org/wiki/Rotation_matrix
        rotation_matrix_x = sample_distance_x * math.cos(imu_angle) - sample_distance_y * math.sin(imu_angle)
        rotation_matrix_y = sample_distance_x * math.sin(imu_angle) + sample_distance_y * math.cos(imu_angle)

        world_x = robot_x + rotation_matrix_x
        world_y = robot_y + rotation_matrix_y

        print("world_x, world_y", world_x, world_y)

        # The '+' calculates the angle relative to the world, by taking into account the orientation of the robot
        world_angle = imu_angle + lidar_beam_angle

        distances.append(float(distance))
        angles.append(float(world_angle))

        # distances.append(float(math.sqrt(world_x**2 + world_y**2)))
        # angles.append(float(math.atan2(world_y, world_x)))

# create a transport node
node = Node()
lidar_topic = "/lidar"
imu_topic = "/imu"
odometry_topic = "/model/FLIP/odometry"

# subscribe to a topic by registering a callback
if node.subscribe(IMU, imu_topic, imu_cb):
    print("Subscribing to type {} on topic [{}]".format(
        LaserScan, lidar_topic))
else:
    print("Error subscribing to topic [{}]".format(lidar_topic))
 
if node.subscribe(LaserScan, lidar_topic, lidar_cb):
    print("Subscribing to type {} on topic [{}]".format(
        LaserScan, lidar_topic))
else:
    print("Error subscribing to topic [{}]".format(lidar_topic))

if node.subscribe(Odometry, odometry_topic, odom_cb):
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
