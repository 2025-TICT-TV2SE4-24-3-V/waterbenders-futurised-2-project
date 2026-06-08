from gz.transport import Node
from gz.msgs.laserscan_pb2 import LaserScan
from gz.msgs.imu_pb2 import IMU
from gz.msgs.odometry_pb2 import Odometry

import time
import math
import numpy as np
import matplotlib.pyplot as plt

IMU_UPDATE_RATE = 200
LIDAR_UPDATE_RATE = 10

odometry_xy = ()

angles = []
distances = []

imu_buffer = []
twist_buffer = []

# ----------------------------
# Integrated pose (twist integration)
# ----------------------------
pose_x = 0.0
pose_y = 0.0
last_time = None

# ----------------------------
# Helpers
# ----------------------------

def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)

def angle_diff(a, b):
    d = a - b
    return (d + math.pi) % (2 * math.pi) - math.pi

def lidar_angle(angle_min, i, inc):
    return angle_min + i * inc

# ----------------------------
# IMU interpolation
# ----------------------------

def interpolate_yaw(t):

    if len(imu_buffer) < 2:
        return imu_buffer[-1][1] if imu_buffer else 0.0

    for i in range(len(imu_buffer) - 1):
        t0, y0 = imu_buffer[i]
        t1, y1 = imu_buffer[i+1]

        if t0 <= t <= t1:
            dt = t1 - t0
            if dt == 0:
                return y0

            alpha = (t - t0) / dt
            return y0 + alpha * angle_diff(y1, y0)

    return imu_buffer[-1][1]

# ----------------------------
# Twist-based motion integration
# ----------------------------

def integrate_pose(target_time):

    global pose_x, pose_y, last_time

    if not twist_buffer:
        return pose_x, pose_y

    if last_time is None:
        last_time = twist_buffer[0][0]
        return pose_x, pose_y

    for i in range(len(twist_buffer) - 1):

        t0, vx, wz = twist_buffer[i]
        t1, _, _ = twist_buffer[i+1]

        if t1 <= last_time:
            continue

        start = max(t0, last_time)
        end = min(t1, target_time)

        if end <= start:
            continue

        dt = end - start

        yaw = interpolate_yaw(start)

        pose_x += vx * math.cos(yaw) * dt
        pose_y += vx * math.sin(yaw) * dt

        last_time = end

        if end >= target_time:
            break

    return pose_x, pose_y

# ----------------------------
# Callbacks
# ----------------------------

def imu_cb(msg: IMU):

    yaw = quat_to_yaw(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )

    imu_time = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

    # 🔧 DEBUG
    print("IMU:", imu_time)

    imu_buffer.append((imu_time, yaw))

    if len(imu_buffer) > IMU_UPDATE_RATE * 3:
        imu_buffer.pop(0)


def odom_cb(msg: Odometry):

    global twist_buffer, odometry_xy

    odom_time = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

    # 🔧 DEBUG
    print("ODOM:", odom_time)

    vx = msg.twist.linear.x
    wz = msg.twist.angular.z

    twist_buffer.append((odom_time, vx, wz))

    if len(twist_buffer) > 2000:
        twist_buffer.pop(0)

    odometry_xy = (msg.pose.position.x, msg.pose.position.y)


def lidar_cb(msg: LaserScan):

    global angles, distances

    lidar_time = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

    # 🔧 DEBUG
    print("LIDAR:", lidar_time)

    scan_duration = 1 / LIDAR_UPDATE_RATE
    beam_dt = scan_duration / len(msg.ranges)

    for i, d in enumerate(msg.ranges):

        if d == float('inf'):
            continue

        beam_time = (lidar_time - scan_duration) + i * beam_dt

        yaw = interpolate_yaw(beam_time)
        robot_x, robot_y = integrate_pose(beam_time)

        angle = lidar_angle(msg.angle_min, i, msg.angle_step)

        lx = d * math.cos(angle)
        ly = d * math.sin(angle)

        wx = robot_x + (lx * math.cos(yaw) - ly * math.sin(yaw))
        wy = robot_y + (lx * math.sin(yaw) + ly * math.cos(yaw))

        distances.append(math.sqrt(wx**2 + wy**2))
        angles.append(math.atan2(wy, wx))

# ----------------------------
# Main
# ----------------------------

node = Node()

node.subscribe(IMU, "/imu", imu_cb)
node.subscribe(LaserScan, "/lidar", lidar_cb)
node.subscribe(Odometry, "/model/FLIP/odometry", odom_cb)

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass

node.unsubscribe("/imu")
node.unsubscribe("/lidar")

# ----------------------------
# Plot
# ----------------------------

angles = np.array(angles)
distances = np.array(distances)

x = np.cos(angles) * distances
y = np.sin(angles) * distances

plt.figure(figsize=(6, 6))
plt.plot(x, y, ".", markersize=1)
plt.axis("equal")
plt.grid(True)
plt.show()

print("Done")
