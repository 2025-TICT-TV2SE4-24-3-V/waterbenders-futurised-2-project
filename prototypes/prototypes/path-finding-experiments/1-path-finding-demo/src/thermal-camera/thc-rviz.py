import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import numpy as np
import cv2
import time

from cv_bridge import CvBridge
from ultralytics import YOLO


class ObjectTemperature(Node):

    def __init__(self):
        super().__init__('object_temperature')

        # -------------------------
        # SUBSCRIPTIONS (NO SYNC FILTERS)
        # -------------------------
        self.sub_rgb = self.create_subscription(
            Image,
            '/camera/image',
            self.rgb_cb,
            10
        )

        self.sub_thermal = self.create_subscription(
            Image,
            '/FLIP/thermal_camera',
            self.thermal_cb,
            10
        )

        # -------------------------
        # PUBLISHER (RVIZ OUTPUT)
        # -------------------------
        self.pub_heatmap = self.create_publisher(
            Image,
            '/thermal/heatmap_image',
            10
        )

        self.bridge = CvBridge()

        # YOLO
        self.model = YOLO("yolov8n.pt")

        self.min_temp = 0.0
        self.max_temp = 200.0

        # latest frames
        self.rgb = None
        self.thermal = None

        # FPS control
        self.last_pub_time = 0
        self.min_interval = 0.1  # 10 Hz

        self.get_logger().info("ObjectTemperature node started")

    # -------------------------
    # CALLBACKS
    # -------------------------
    def rgb_cb(self, msg):
        self.rgb = msg
        self.try_process()

    def thermal_cb(self, msg):
        self.thermal = msg
        self.try_process()

    # -------------------------
    # CORE PIPELINE
    # -------------------------
    def try_process(self):

        if self.rgb is None or self.thermal is None:
            return

        now = time.time()
        if now - self.last_pub_time < self.min_interval:
            return

        self.last_pub_time = now

        # -------------------------
        # RGB
        # -------------------------
        rgb = np.frombuffer(self.rgb.data, dtype=np.uint8).reshape(
            (self.rgb.height, self.rgb.width, 3)
        )

        # -------------------------
        # THERMAL (mono16 FIXED)
        # -------------------------
        thermal_raw = np.frombuffer(self.thermal.data, dtype=np.uint16).reshape(
            (self.thermal.height, self.thermal.width)
        )

        # resize thermal → RGB
        thermal_resized = cv2.resize(
            thermal_raw,
            (rgb.shape[1], rgb.shape[0]),
            interpolation=cv2.INTER_NEAREST
        )

        # normalize
        min_val = np.min(thermal_resized)
        max_val = np.max(thermal_resized)

        norm = (thermal_resized.astype(np.float32) - min_val) / (max_val - min_val + 1e-6)

        temp = self.min_temp + norm * (self.max_temp - self.min_temp)

        # -------------------------
        # YOLO
        # -------------------------
        results = self.model(rgb, verbose=False)[0]

        heatmap = cv2.applyColorMap(
            (norm * 255).astype(np.uint8),
            cv2.COLORMAP_INFERNO
        )

        # -------------------------
        # DRAW BOXES
        # -------------------------
        if results.boxes is not None:

            for box in results.boxes:

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0])
                label = self.model.names[cls]

                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(temp.shape[1], x2), min(temp.shape[0], y2)

                region = temp[y1:y2, x1:x2]

                if region.size == 0:
                    continue

                avg_temp = np.mean(region)

                cv2.rectangle(heatmap, (x1, y1), (x2, y2), (255, 255, 255), 2)

                cv2.putText(
                    heatmap,
                    f"{label}: {avg_temp:.1f}C",
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1
                )

        # -------------------------
        # PUBLISH TO RVIZ
        # -------------------------
        heatmap = cv2.cvtColor(heatmap, cv2.COLOR_BGR2RGB)

        msg = self.bridge.cv2_to_imgmsg(heatmap.astype(np.uint8), encoding='rgb8')

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"

        self.pub_heatmap.publish(msg)

        self.get_logger().info(
            f"Published | avg temp: {np.mean(temp):.2f}C | max: {np.max(temp):.2f}C"
        )


# -------------------------
# MAIN
# -------------------------
def main(args=None):
    rclpy.init(args=args)

    node = ObjectTemperature()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()