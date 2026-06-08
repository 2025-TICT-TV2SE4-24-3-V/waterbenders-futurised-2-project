from gz.transport import Node
from gz.msgs.laserscan_pb2 import LaserScan
from gz.msgs.imu_pb2 import IMU

import time
import math
import numpy as np
import matplotlib.pyplot as plt


# ======================
# CONFIG (your setup)
# ======================
IMU_RATE = 200
LIDAR_RATE = 10

imu_buffer = []
angles = []
distances = []


# ======================
# ANGLE HELPERS
# ======================
def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

# Traceback (most recent call last):
#   File "/usr/lib/python3/dist-packages/gz/transport/__init__.py", line 115, in cb_deserialize
#     callback(deserialized_msg)
#   File "/workspace/models/scripts/occupancy-grid.py", line 110, in lasermsg_cb
#     angles.append(float(world_angle))
#                         ^^^^^^^^^^^
# UnboundLocalError: cannot access local variable 'world_angle' where it is not associated with a value

# ======================
# IMU CALLBACK (200 Hz)
# ======================
def imu_cb(msg: IMU):

    global imu_buffer

    yaw = quat_to_yaw(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )

    t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

    imu_buffer.append((t, yaw))

    # keep buffer ~3 seconds
    if len(imu_buffer) > IMU_RATE * 3:
        imu_buffer.pop(0)


# ======================
# FIND IMU ANGLE AT TIME
# ======================
def get_imu_angle(t):

    if len(imu_buffer) < 2:
        return imu_buffer[0][1] if imu_buffer else 0.0

    for i in range(len(imu_buffer) - 1):
        t0, y0 = imu_buffer[i]
        t1, y1 = imu_buffer[i + 1]

        if t0 <= t <= t1:

            # def wrap_angle(a):
            #     return (a + np.pi) % (2 * np.pi) - np.pi

            # shortest-path interpolation (CRITICAL FIX)
            dy = wrap_angle(y1 - y0)
            ratio = (t - t0) / (t1 - t0)

            return wrap_angle(y0 + dy * ratio)

    return imu_buffer[-1][1]
    # return print("Failure")


# ======================
# LIDAR CALLBACK (10 Hz)
# ======================
def lidar_cb(msg: LaserScan):

    global angles, distances

    t_scan = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    n = len(msg.ranges)

    # estimate scan duration from rate
    scan_duration = 1.0 / LIDAR_RATE
    dt = scan_duration / n

    angle_min = msg.angle_min
    angle_max = msg.angle_max

    # robust increment
    angle_inc = (angle_max - angle_min) / (n - 1)

    for i, r in enumerate(msg.ranges):

        if not np.isfinite(r):
            continue

        # beam time (scan starts at t_scan - duration)
        beam_t = (t_scan - scan_duration) + i * dt

        # IMU yaw at beam time
        yaw = get_imu_angle(beam_t)

        # LiDAR angle in sensor frame
        lidar_angle = angle_min + i * angle_inc

        # ======================
        # FRAME FUSION (KEY FIX)
        # ======================
        world_angle = wrap_angle(yaw + lidar_angle)

        angles.append(world_angle)
        distances.append(r)


# ======================
# NODE SETUP
# ======================
node = Node()

node.subscribe(IMU, "/imu", imu_cb)
node.subscribe(LaserScan, "/lidar", lidar_cb)


# ======================
# RUN
# ======================
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass


# ======================
# PLOT
# ======================
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
