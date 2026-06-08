from gz.transport import Node
from gz.msgs.twist_pb2 import Twist
from gz.msgs.laserscan_pb2 import LaserScan
from gz.msgs.imu_pb2 import IMU
from gz.msgs.odometry_pb2 import Odometry

import time
import math
import numpy as np
import matplotlib.pyplot as plt

# Constants
IMU_UPDATE_RATE = 200
LIDAR_UPDATE_RATE = 10
MAX_ANGULAR_VELOCITY = 0.5  # Rad/s
SKIP_SAMPLES_ON_ABRUPT_ROTATION = 20

# Global state
angles = []
distances = []
imu_buffer = []
twist_buffer = []

# Integrated pose (from twist messages)
pose_x = 0.0
pose_y = 0.0
last_time = None

# Odometry (for linear_x, used in abrupt rotation detection)
linear_x = 0.0

# ----------------------------
# Helpers
# ----------------------------

def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def angle_diff(a, b):
    d = a - b
    return (d + math.pi) % (2 * math.pi) - math.pi

def lidar_angle(angle_min, i, inc):
    return angle_min + i * inc

# ----------------------------
# IMU Interpolation with Rotation Checks
# ----------------------------

def interpolate_yaw(timestamp):
    global linear_x

    if len(imu_buffer) < 2:
        return imu_buffer[-1][1] if imu_buffer else 0.0

    for i in range(len(imu_buffer) - 1):
        t0, yaw0, ang_v0 = imu_buffer[i]
        t1, yaw1, ang_v1 = imu_buffer[i + 1]

        # Check for abrupt rotation (sign change in angular velocity)
        if (ang_v0 * ang_v1) < 0.0 and -0.4 < linear_x < 0.4:
            return "abrupt_rotation_detected"

        if t0 <= timestamp <= t1:
            # Check for rapid rotation (exceeds MAX_ANGULAR_VELOCITY threshold)
            angular_threshold = (MAX_ANGULAR_VELOCITY * (t1 - t0)) / 2
            if abs(yaw1 - yaw0) >= angular_threshold:
                return "rapid_rotation_detected"

            # Linear interpolation
            dt = t1 - t0
            if dt == 0:
                return yaw0
            alpha = (timestamp - t0) / dt
            return yaw0 + alpha * angle_diff(yaw1, yaw0)

    return imu_buffer[-1][1] if imu_buffer else 0.0

# ----------------------------
# Twist-Based Pose Integration
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
        t1, _, _ = twist_buffer[i + 1]

        if t1 <= last_time:
            continue

        start = max(t0, last_time)
        end = min(t1, target_time)

        if end <= start:
            continue

        dt = end - start
        yaw = interpolate_yaw(start)

        # Skip if yaw interpolation fails (rapid/abrupt rotation)
        if yaw in ["rapid_rotation_detected", "abrupt_rotation_detected"]:
            return pose_x, pose_y

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
    imu_buffer.append((imu_time, yaw, msg.angular_velocity.z))

    if len(imu_buffer) > IMU_UPDATE_RATE * 3:
        imu_buffer.pop(0)

def odom_cb(msg: Odometry):
    global linear_x, twist_buffer
    odom_time = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    linear_x = msg.twist.linear.x
    twist_buffer.append((odom_time, msg.twist.linear.x, msg.twist.angular.z))

    if len(twist_buffer) > 2000:
        twist_buffer.pop(0)

def lidar_cb(msg: LaserScan):
    global angles, distances

    lidar_time = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    scan_duration = 1 / LIDAR_UPDATE_RATE
    beam_dt = scan_duration / len(msg.ranges)
    skip_samples = 0

    for i, d in enumerate(msg.ranges):
        if d == float('inf'):
            continue

        if skip_samples > 0:
            skip_samples -= 1
            continue

        beam_time = (lidar_time - scan_duration) + i * beam_dt
        yaw = interpolate_yaw(beam_time)

        if yaw == "rapid_rotation_detected":
            continue
        elif yaw == "abrupt_rotation_detected":
            skip_samples = SKIP_SAMPLES_ON_ABRUPT_ROTATION
            continue

        robot_x, robot_y = integrate_pose(beam_time)
        angle = lidar_angle(msg.angle_min, i, msg.angle_step)

        # Transform LiDAR point to world frame
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

# Plot
angles = np.array(angles)
distances = np.array(distances)
x = np.cos(angles) * distances
y = np.sin(angles) * distances

plt.figure(figsize=(6, 6))
plt.plot(x, y, "b.", markersize=1)
plt.axis("equal")
plt.grid(True)
plt.show()
print("Done")
