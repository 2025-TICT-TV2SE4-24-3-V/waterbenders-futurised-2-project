from gz.transport import Node
from gz.msgs.twist_pb2 import Twist
from gz.msgs.laserscan_pb2 import LaserScan
from gz.msgs.imu_pb2 import IMU
from gz.msgs.odometry_pb2 import Odometry

import time
import math
import numpy as np
import matplotlib.pyplot as plt

IMU_UPDATE_RATE = 200
LIDAR_UPDATE_RATE = 10
MAX_ANGULAR_VELOCITY = 0.5  # Rad/s

# ----------------------------
# Integrated pose
# ----------------------------
pose_x = 0.0
pose_y = 0.0
last_time = None

angles = []
distances = []

imu_buffer = []
odom_buffer = []

linear_x = 0.0


# ----------------------------
# Helpers
# ----------------------------
def quat_euler_angle(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)


def lidar_angle(angle_min, measurement_index, angle_increment):
    return angle_min + (measurement_index * angle_increment)


# ----------------------------
# IMU interpolation (UNCHANGED LOGIC)
# ----------------------------
def interpolate_imu_yaw(timestamp):

    if len(imu_buffer) <= 1:
        return imu_buffer[0][1] if imu_buffer else 0.0

    for i in range(len(imu_buffer) - 1):
        t0, yaw0, _, ang_v0 = imu_buffer[i]
        t1, yaw1, _, ang_v1 = imu_buffer[i + 1]

        angular_threshold = (MAX_ANGULAR_VELOCITY * (t1 - t0)) / 2
        print(angular_threshold)

        # KEEP abrupt rotation detection
        if (ang_v0 * ang_v1) < 0.0 and -0.4 < linear_x < 0.4:
            return "abrupt_rotation_detected"

        if t0 <= timestamp <= t1:

            # KEEP rapid rotation detection
            if abs(yaw1 - yaw0) >= 0.001:
                return "rapid_rotation_detected"

            dt = t1 - t0
            if dt <= 0:
                return yaw0

            alpha = (timestamp - t0) / dt
            return yaw0 + alpha * (yaw1 - yaw0)

    return "no_angle_found"


# ----------------------------
# Pose integration (FIXED, robust to flags)
# ----------------------------
def integrate_pose(target_time):

    global pose_x, pose_y, last_time

    if not odom_buffer:
        return pose_x, pose_y

    if last_time is None:
        last_time = odom_buffer[0][0]
        return pose_x, pose_y

    for i in range(len(odom_buffer) - 1):

        t0, vx0, _ = odom_buffer[i]
        t1, vx1, _ = odom_buffer[i + 1]

        if t1 <= last_time:
            continue

        start = max(t0, last_time)
        end = min(t1, target_time)

        if end <= start:
            continue

        dt = end - start
        mid_t = start + dt / 2.0

        yaw_mid = interpolate_imu_yaw(mid_t)

        # ✅ Instead of breaking everything, just skip bad segments
        if yaw_mid == "rapid_rotation_detected" or yaw_mid == "no_angle_found":
            last_time = end
            continue

        if yaw_mid == "abrupt_rotation_detected":
            last_time = end
            continue

        vx_mid = 0.5 * (vx0 + vx1)

        pose_x += vx_mid * math.cos(yaw_mid) * dt
        pose_y += vx_mid * math.sin(yaw_mid) * dt

        last_time = end

        if end >= target_time:
            break

    return pose_x, pose_y


# ----------------------------
# Callbacks
# ----------------------------
def odom_cb(_msg: Odometry):

    global linear_x, odom_buffer

    linear_x = _msg.twist.linear.x

    odom_time = _msg.header.stamp.sec + _msg.header.stamp.nsec * 1e-9

    vx = _msg.twist.linear.x
    wz = _msg.twist.angular.z

    odom_buffer.append((odom_time, vx, wz))

    if len(odom_buffer) > 500:
        odom_buffer.pop(0)


def imu_cb(_msg: IMU):

    global imu_buffer

    yaw = quat_euler_angle(
        _msg.orientation.x,
        _msg.orientation.y,
        _msg.orientation.z,
        _msg.orientation.w
    )

    timestamp = _msg.header.stamp.sec + (_msg.header.stamp.nsec * 1e-9)

    imu_buffer.append((timestamp, yaw, 0.0, _msg.angular_velocity.z))

    if len(imu_buffer) > (IMU_UPDATE_RATE * 3):
        imu_buffer.pop(0)


def lidar_cb(_msg: LaserScan):

    global angles, distances

    timestamp = _msg.header.stamp.sec + (_msg.header.stamp.nsec * 1e-9)

    robot_x, robot_y = integrate_pose(timestamp)

    scan_duration = 1 / LIDAR_UPDATE_RATE
    beam_scantime = scan_duration / len(_msg.ranges)

    skip_samples = 0

    for i, distance in enumerate(_msg.ranges):

        if distance == float('inf'):
            continue

        if skip_samples > 0:
            skip_samples -= 1
            continue

        beam_timestamp = (timestamp - scan_duration) + i * beam_scantime
        lidar_beam_angle = lidar_angle(_msg.angle_min, i, _msg.angle_step)

        imu_angle = interpolate_imu_yaw(beam_timestamp)

        # KEEP your filtering behavior
        if imu_angle == "rapid_rotation_detected" or imu_angle == "no_angle_found":
            continue
        elif imu_angle == "abrupt_rotation_detected":
            skip_samples = 20
            continue

        # Local → world transform
        local_x = distance * math.cos(lidar_beam_angle)
        local_y = distance * math.sin(lidar_beam_angle)

        world_dx = local_x * math.cos(imu_angle) - local_y * math.sin(imu_angle)
        world_dy = local_x * math.sin(imu_angle) + local_y * math.cos(imu_angle)

        world_x = robot_x + world_dx
        world_y = robot_y + world_dy

        world_angle = math.atan2(world_y, world_x)
        world_distance = math.sqrt(world_x**2 + world_y**2)

        angles.append(world_angle)
        distances.append(world_distance)


# ----------------------------
# Node setup
# ----------------------------
node = Node()

lidar_topic = "/lidar"
imu_topic = "/imu"
odometry_topic = "/model/FLIP/odometry"

node.subscribe(IMU, imu_topic, imu_cb)
node.subscribe(LaserScan, lidar_topic, lidar_cb)
node.subscribe(Odometry, odometry_topic, odom_cb)

# ----------------------------
# Main loop
# ----------------------------
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass

node.unsubscribe(imu_topic)
node.unsubscribe(lidar_topic)

# ----------------------------
# Plot
# ----------------------------
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
