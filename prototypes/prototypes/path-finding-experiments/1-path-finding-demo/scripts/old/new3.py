from gz.transport import Node
from gz.msgs.twist_pb2 import Twist
from gz.msgs.laserscan_pb2 import LaserScan
from gz.msgs.imu_pb2 import IMU
from gz.msgs.odometry_pb2 import Odometry
import time
import math
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
from filterpy.kalman import KalmanFilter

# Constants
IMU_UPDATE_RATE = 200
LIDAR_UPDATE_RATE = 10
MAX_ANGULAR_VELOCITY = 0.5  # Rad/s

# Global variables
angles = []
distances = []
imu_buffer = []
odometry_xy = (0.0, 0.0)
current_distance = 0.0
linear_x = 0.0
previous_scan_points = np.array([])  # For ICP

# NEW: Initialize Kalman Filter
def init_kalman_filter():
    kf = KalmanFilter(dim_x=3, dim_z=3)
    kf.x = np.array([0., 0., 0.])  # Initial state [x, y, theta]
    kf.F = np.eye(3)  # State transition matrix
    kf.H = np.eye(3)  # Measurement function
    kf.P *= 1000  # Initial uncertainty
    kf.R = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.01]])  # Measurement noise
    kf.Q = np.array([[0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.001]])  # Process noise
    return kf

kalman_filter = init_kalman_filter()  # Global Kalman Filter

# NEW: ICP alignment using Open3D
def icp_align(source_points, target_points):
    """
    Aligns source_points to target_points using Open3D's ICP.
    Returns the aligned source_points as a numpy array.
    """
    if len(source_points) < 3 or len(target_points) < 3:
        return np.array(source_points)

    # Convert 2D points to 3D (z=0)
    source_3d = np.hstack([source_points, np.zeros((len(source_points), 1))])
    target_3d = np.hstack([target_points, np.zeros((len(target_points), 1))])

    # Create Open3D point clouds
    source_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(source_3d)

    target_pcd = o3d.geometry.PointCloud()
    target_pcd.points = o3d.utility.Vector3dVector(target_3d)

    # Run ICP
    threshold = 1.0  # Distance threshold for correspondence
    result = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, threshold, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )

    # Apply the transformation and return as 2D
    aligned_source = np.asarray(source_pcd.transform(result.transformation).points)[:, :2]
    return aligned_source

def quat_euler_angle(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)

def lidar_angle(angle_min, measurement_index, angle_step):
    return angle_min + (measurement_index * angle_step)

def interpolate_imu_yaw(timestamp):
    if len(imu_buffer) <= 1:
        return imu_buffer[0][1] if imu_buffer else 0.0

    for i in range(len(imu_buffer) - 1):
        t0, yaw0, _, ang_v0 = imu_buffer[i]
        t1, yaw1, _, ang_v1 = imu_buffer[i+1]
        angular_threshold = (MAX_ANGULAR_VELOCITY * (t1 - t0)) / 2

        if (ang_v0 * ang_v1) < 0.0 and -0.4 < linear_x < 0.4:
            return "abrupt_rotation_detected"
        if t0 <= timestamp <= t1:
            if abs(yaw1 - yaw0) >= angular_threshold:
                return "rapid_rotation_detected"
            imu_angle = yaw0 + (((yaw1 - yaw0) / (t1 - t0)) * (timestamp - t0))
            return imu_angle

    return "no_angle_found"

def odom_cb(_msg: Odometry):
    global odometry_xy, current_distance, linear_x, kalman_filter

    x = _msg.pose.position.x
    y = _msg.pose.position.y
    theta = quat_euler_angle(
        _msg.pose.orientation.x,
        _msg.pose.orientation.y,
        _msg.pose.orientation.z,
        _msg.pose.orientation.w
    )

    # Update Kalman Filter
    kalman_filter.predict()
    kalman_filter.update(np.array([x, y, theta]))
    smoothed_x, smoothed_y, smoothed_theta = kalman_filter.x

    odometry_xy = (smoothed_x, smoothed_y)
    current_distance = math.sqrt(smoothed_x**2 + smoothed_y**2)
    linear_x = _msg.twist.linear.x

def imu_cb(_msg: IMU):
    global imu_buffer
    orientation_rad = quat_euler_angle(_msg.orientation.x, _msg.orientation.y, _msg.orientation.z, _msg.orientation.w)
    timestamp = _msg.header.stamp.sec + (_msg.header.stamp.nsec * 1e-9)
    imu_buffer.append((timestamp, orientation_rad, 0.0, _msg.angular_velocity.z))
    if len(imu_buffer) > (IMU_UPDATE_RATE * 3):
        imu_buffer.pop(0)

def lidar_cb(_msg: LaserScan):
    global angles, distances, previous_scan_points

    timestamp = _msg.header.stamp.sec + (_msg.header.stamp.nsec * 1e-9)
    scan_duration = 1 / LIDAR_UPDATE_RATE
    beam_scantime = scan_duration / len(_msg.ranges)
    skip_samples = 0
    current_scan_points = []

    for i, distance in enumerate(_msg.ranges):
        if distance == float('inf'):
            continue
        if skip_samples > 0:
            skip_samples -= 1
            continue

        beam_timestamp = (timestamp - scan_duration) + i * beam_scantime
        lidar_beam_angle = lidar_angle(_msg.angle_min, i, _msg.angle_step)
        imu_angle = interpolate_imu_yaw(beam_timestamp)

        if imu_angle in ["rapid_rotation_detected", "no_angle_found"]:
            continue
        elif imu_angle == "abrupt_rotation_detected":
            skip_samples = 20
            continue

        robot_x, robot_y = odometry_xy
        sample_distance_x = distance * math.cos(lidar_beam_angle)
        sample_distance_y = distance * math.sin(lidar_beam_angle)
        rotation_matrix_x = sample_distance_x * math.cos(imu_angle) - sample_distance_y * math.sin(imu_angle)
        rotation_matrix_y = sample_distance_x * math.sin(imu_angle) + sample_distance_y * math.cos(imu_angle)
        world_x = robot_x + rotation_matrix_x
        world_y = robot_y + rotation_matrix_y

        current_scan_points.append([world_x, world_y])
        distances.append(float(distance))
        angles.append(float(imu_angle + lidar_beam_angle))

    # Apply ICP if there are previous points
    if len(current_scan_points) > 0 and len(previous_scan_points) > 0:
        aligned_points = icp_align(current_scan_points, previous_scan_points)
        current_scan_points = aligned_points.tolist()

    # Store the current scan for the next iteration
    previous_scan_points = np.array(current_scan_points)

# Initialize node and subscribe to topics
node = Node()
lidar_topic = "/lidar"
imu_topic = "/imu"
odometry_topic = "/model/FLIP/odometry"

node.subscribe(IMU, imu_topic, imu_cb)
node.subscribe(LaserScan, lidar_topic, lidar_cb)
node.subscribe(Odometry, odometry_topic, odom_cb)

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass

node.unsubscribe(imu_topic)
node.unsubscribe(lidar_topic)

# Plot the results
x = np.cos(angles) * distances
y = np.sin(angles) * distances

plt.figure(figsize=(6, 6))
plt.plot(x, y, "b.", markersize=1)
plt.axis("equal")
plt.grid(True)
plt.show()
print("Done")
