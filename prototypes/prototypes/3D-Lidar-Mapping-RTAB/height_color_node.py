#!/usr/bin/env python3
"""
height_color_node.py
--------------------
Subscribes to /points (raw XYZ PointCloud2 from Gazebo LiDAR bridge) and
republishes /points_colored with per-point RGB packed correctly for RViz.

Colour gradient (Z-based):
  z <= z_min  →  blue   (0, 0, 255)
  z == mid    →  green  (0, 255, 0)
  z >= z_max  →  red    (255, 0, 0)

Place this file in your WORKSPACE_DIR and it is launched by rtabmap_gazebo.launch.py.

Parameters:
  z_min  (float, default  0.0)   Z value → blue
  z_max  (float, default  3.5)   Z value → red
"""

import struct
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField


# ── Colour math ───────────────────────────────────────────────────────────────

def z_to_rgb_bytes(z: float, z_min: float, z_max: float):
    """Return (r, g, b) uint8 for a Z value via blue→green→red gradient."""
    t = float(np.clip((z - z_min) / max(z_max - z_min, 1e-6), 0.0, 1.0))
    if t < 0.5:
        s = t * 2.0
        return 0, int(255 * s), int(255 * (1.0 - s))   # blue → green
    else:
        s = (t - 0.5) * 2.0
        return int(255 * s), int(255 * (1.0 - s)), 0   # green → red


def pack_rgb(r: int, g: int, b: int) -> bytes:
    """
    Pack R,G,B into 4 bytes in the layout RViz expects for a FLOAT32 'rgb' field.

    RViz reads the rgb field as a little-endian uint32 = 0x00RRGGBB, then
    interprets the raw bytes as a float32. We write the 4 bytes as:
        byte0 = B, byte1 = G, byte2 = R, byte3 = 0x00
    which is exactly what ros_visualisation's PointCloud2Display expects.
    """
    return struct.pack('BBBB', b, g, r, 0)


# ── Vectorised cloud conversion ───────────────────────────────────────────────

def colorize_cloud(msg: PointCloud2, z_min: float, z_max: float) -> PointCloud2:
    """
    Read an XYZ PointCloud2 and return an XYZRGB PointCloud2.
    Uses numpy for the inner loop — fast enough for 16×640 = 10 240 pts @ 10 Hz.
    """
    # ── Locate x/y/z byte offsets in the incoming message ────────────────────
    field_map = {f.name: f for f in msg.fields}
    x_off = field_map['x'].offset
    y_off = field_map['y'].offset
    z_off = field_map['z'].offset
    src_step = msg.point_step
    n = msg.width * msg.height

    raw = np.frombuffer(msg.data, dtype=np.uint8)

    # Extract x, y, z as float32 arrays using strided views
    xs = np.frombuffer(raw[x_off::src_step].tobytes()[:n*4], dtype=np.float32)
    ys = np.frombuffer(raw[y_off::src_step].tobytes()[:n*4], dtype=np.float32)
    zs = np.frombuffer(raw[z_off::src_step].tobytes()[:n*4], dtype=np.float32)

    # ── Compute RGB for every point in one vectorised pass ───────────────────
    t = np.clip((zs - z_min) / max(z_max - z_min, 1e-6), 0.0, 1.0)

    r = np.where(t >= 0.5, ((t - 0.5) * 2.0 * 255).astype(np.uint8), np.uint8(0))
    g = np.where(
        t < 0.5,
        (t * 2.0 * 255).astype(np.uint8),
        ((1.0 - (t - 0.5) * 2.0) * 255).astype(np.uint8)
    )
    b = np.where(t < 0.5, ((1.0 - t * 2.0) * 255).astype(np.uint8), np.uint8(0))

    # ── Build output buffer: x(4) y(4) z(4) rgb(4) = 16 bytes/point ─────────
    dst_step = 16
    out_buf = np.zeros(n * dst_step, dtype=np.uint8)

    # Copy x, y, z floats
    out_buf[0::dst_step], out_buf[1::dst_step] = \
        raw[x_off::src_step][:n], raw[x_off+1::src_step][:n]
    out_buf[2::dst_step], out_buf[3::dst_step] = \
        raw[x_off+2::src_step][:n], raw[x_off+3::src_step][:n]

    out_buf[4::dst_step], out_buf[5::dst_step] = \
        raw[y_off::src_step][:n], raw[y_off+1::src_step][:n]
    out_buf[6::dst_step], out_buf[7::dst_step] = \
        raw[y_off+2::src_step][:n], raw[y_off+3::src_step][:n]

    out_buf[8::dst_step],  out_buf[9::dst_step]  = \
        raw[z_off::src_step][:n], raw[z_off+1::src_step][:n]
    out_buf[10::dst_step], out_buf[11::dst_step] = \
        raw[z_off+2::src_step][:n], raw[z_off+3::src_step][:n]

    # Pack RGB into bytes 12-15: layout = [B, G, R, 0x00] (little-endian 0x00RRGGBB)
    out_buf[12::dst_step] = b   # byte 0 → Blue
    out_buf[13::dst_step] = g   # byte 1 → Green
    out_buf[14::dst_step] = r   # byte 2 → Red
    out_buf[15::dst_step] = 0   # byte 3 → padding

    # ── Assemble output message ───────────────────────────────────────────────
    out = PointCloud2()
    out.header       = msg.header
    out.height       = msg.height
    out.width        = msg.width
    out.is_bigendian = False
    out.point_step   = dst_step
    out.row_step     = dst_step * msg.width
    out.is_dense     = msg.is_dense
    out.fields       = [
        PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    out.data = out_buf.tobytes()
    return out


# ── ROS 2 node ────────────────────────────────────────────────────────────────

class HeightColorNode(Node):
    def __init__(self):
        super().__init__('height_color_node')
        self.declare_parameter('z_min', 0.0)
        self.declare_parameter('z_max', 3.5)
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.get_logger().info(
            f'Height coloriser started  z_min={self.z_min}  z_max={self.z_max}\n'
            f'  Subscribe : /points\n'
            f'  Publish   : /points_colored\n'
            f'  In RViz   : PointCloud2 → /points_colored → Color Transformer: RGB8'
        )

        # Match Gazebo bridge QoS (Best Effort)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub = self.create_publisher(PointCloud2, '/points_colored', qos)
        self.sub = self.create_subscription(
            PointCloud2, '/points', self._cb, qos
        )

    def _cb(self, msg: PointCloud2):
        self.pub.publish(colorize_cloud(msg, self.z_min, self.z_max))


def main(args=None):
    rclpy.init(args=args)
    node = HeightColorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()