#!/usr/bin/env python3

import math
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image


def yaw_from_ros_quat(q):
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)

    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class StateMonitorGUI(Node):
    """
    Backend-independent realtime GUI monitor.

    Subscribes:
      /uav/odom
      /uav/imu
      /uav/camera/depth/image
      /uav/camera/rgb/image

    Notes:
      - Requires a display. Over SSH, use X forwarding or run locally.
      - For headless SSH without display, use `state_monitor` instead.
    """

    def __init__(self):
        super().__init__("uav_state_monitor_gui")

        self.declare_parameter("odom_topic", "/uav/odom")
        self.declare_parameter("imu_topic", "/uav/imu")
        self.declare_parameter("depth_topic", "/uav/camera/depth/image")
        self.declare_parameter("rgb_topic", "/uav/camera/rgb/image")

        self.declare_parameter("show_rgb", True)
        self.declare_parameter("show_depth", True)
        self.declare_parameter("depth_clip_m", 20.0)
        self.declare_parameter("window_name", "UAV State Monitor")

        self.odom_topic = self.get_parameter("odom_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.rgb_topic = self.get_parameter("rgb_topic").value

        self.show_rgb = bool(self.get_parameter("show_rgb").value)
        self.show_depth = bool(self.get_parameter("show_depth").value)
        self.depth_clip_m = float(self.get_parameter("depth_clip_m").value)
        self.window_name = self.get_parameter("window_name").value

        self.bridge = CvBridge()

        self.odom = None
        self.imu = None
        self.depth_msg = None
        self.rgb_msg = None

        self.odom_count = 0
        self.imu_count = 0
        self.depth_count = 0
        self.rgb_count = 0

        self.last_rate_time = time.time()
        self.odom_hz = 0
        self.imu_hz = 0
        self.depth_hz = 0
        self.rgb_hz = 0

        qos = qos_profile_sensor_data

        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, qos)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, qos)

        if self.show_rgb:
            self.create_subscription(Image, self.rgb_topic, self.rgb_cb, qos)

        self.timer = self.create_timer(1.0 / 20.0, self.render)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.get_logger().info("UAV state GUI monitor started.")
        self.get_logger().info(f"odom : {self.odom_topic}")
        self.get_logger().info(f"imu  : {self.imu_topic}")
        self.get_logger().info(f"depth: {self.depth_topic}")
        self.get_logger().info(f"rgb  : {self.rgb_topic}")

    def odom_cb(self, msg):
        self.odom = msg
        self.odom_count += 1

    def imu_cb(self, msg):
        self.imu = msg
        self.imu_count += 1

    def depth_cb(self, msg):
        self.depth_msg = msg
        self.depth_count += 1

    def rgb_cb(self, msg):
        self.rgb_msg = msg
        self.rgb_count += 1

    def update_rates(self):
        now = time.time()
        dt = now - self.last_rate_time

        if dt >= 1.0:
            self.odom_hz = self.odom_count / dt
            self.imu_hz = self.imu_count / dt
            self.depth_hz = self.depth_count / dt
            self.rgb_hz = self.rgb_count / dt

            self.odom_count = 0
            self.imu_count = 0
            self.depth_count = 0
            self.rgb_count = 0
            self.last_rate_time = now

    def make_blank(self, title, width=640, height=360):
        img = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(
            img,
            title,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return img

    def depth_to_bgr(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert depth image: {exc}")
            return self.make_blank("Depth conversion failed")

        depth = np.asarray(depth)

        if depth.ndim == 3:
            depth = depth[:, :, 0]

        depth = depth.astype(np.float32)

        # 16UC1 depth is often in millimeters. Convert to meters.
        if msg.encoding.upper() in ["16UC1", "MONO16"]:
            depth = depth / 1000.0

        depth[~np.isfinite(depth)] = 0.0
        depth = np.clip(depth, 0.0, self.depth_clip_m)

        if self.depth_clip_m <= 1e-6:
            self.depth_clip_m = 20.0

        norm = (depth / self.depth_clip_m * 255.0).astype(np.uint8)
        color = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)

        return color

    def rgb_to_bgr(self, msg):
        try:
            if msg.encoding in ["rgb8", "rgba8"]:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert RGB image: {exc}")
            return self.make_blank("RGB conversion failed")

        return img

    def overlay_text(self, img):
        lines = []

        lines.append(
            f"Hz odom={self.odom_hz:5.1f} imu={self.imu_hz:5.1f} "
            f"depth={self.depth_hz:5.1f} rgb={self.rgb_hz:5.1f}"
        )

        if self.odom is not None:
            p = self.odom.pose.pose.position
            v = self.odom.twist.twist.linear
            yaw = yaw_from_ros_quat(self.odom.pose.pose.orientation)

            lines.append(
                f"pos ENU: x={p.x: .2f} y={p.y: .2f} z={p.z: .2f} "
                f"yaw={math.degrees(yaw): .1f} deg"
            )
            lines.append(
                f"vel body FLU: forward={v.x: .2f} left={v.y: .2f} up={v.z: .2f}"
            )
        else:
            lines.append("odom: no data")

        if self.imu is not None:
            g = self.imu.angular_velocity
            a = self.imu.linear_acceleration
            lines.append(
                f"gyro: [{g.x: .2f}, {g.y: .2f}, {g.z: .2f}] rad/s"
            )
            lines.append(
                f"acc : [{a.x: .2f}, {a.y: .2f}, {a.z: .2f}] m/s^2"
            )
        else:
            lines.append("imu: no data")

        y = 28
        for line in lines:
            cv2.putText(
                img,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 0),
                4,
                cv2.LINE_AA,
            )
            cv2.putText(
                img,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            y += 24

        return img

    def render(self):
        self.update_rates()

        panels = []

        if self.show_depth:
            if self.depth_msg is not None:
                depth_img = self.depth_to_bgr(self.depth_msg)
            else:
                depth_img = self.make_blank("No depth image")
            cv2.putText(
                depth_img,
                "Depth",
                (12, depth_img.shape[0] - 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            panels.append(depth_img)

        if self.show_rgb:
            if self.rgb_msg is not None:
                rgb_img = self.rgb_to_bgr(self.rgb_msg)
            else:
                rgb_img = self.make_blank("No RGB image")
            cv2.putText(
                rgb_img,
                "RGB",
                (12, rgb_img.shape[0] - 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            panels.append(rgb_img)

        if not panels:
            canvas = self.make_blank("No image panels enabled")
        else:
            target_h = 360
            resized = []
            for p in panels:
                h, w = p.shape[:2]
                scale = target_h / max(h, 1)
                new_w = int(w * scale)
                resized.append(cv2.resize(p, (new_w, target_h)))
            canvas = np.hstack(resized)

        canvas = self.overlay_text(canvas)

        cv2.imshow(self.window_name, canvas)

        key = cv2.waitKey(1) & 0xFF
        if key in [ord("q"), 27]:
            self.get_logger().info("GUI requested shutdown.")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = StateMonitorGUI()

    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()


if __name__ == "__main__":
    main()
