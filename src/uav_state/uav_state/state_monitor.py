#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Bool, Float32, String


def yaw_from_ros_quat(q):
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)

    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class StateMonitor(Node):
    """
    Backend-independent UAV state monitor.

    Subscribes only to normalized /uav topics:

      /uav/odom
      /uav/imu
      /uav/camera/depth/image
      /uav/crash
    """

    def __init__(self):
        super().__init__("uav_state_monitor")

        self.declare_parameter("odom_topic", "/uav/odom")
        self.declare_parameter("imu_topic", "/uav/imu")
        self.declare_parameter("depth_topic", "/uav/camera/depth/image")
        self.declare_parameter("crash_topic", "/uav/crash")
        self.declare_parameter("crash_reason_topic", "/uav/crash_reason")
        self.declare_parameter("contact_force_topic", "/uav/contact_force_n")
        self.declare_parameter("contact_depth_topic", "/uav/contact_depth_m")

        self.odom_topic = self.get_parameter("odom_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.crash_topic = self.get_parameter("crash_topic").value
        self.crash_reason_topic = self.get_parameter(
            "crash_reason_topic"
        ).value
        self.contact_force_topic = self.get_parameter(
            "contact_force_topic"
        ).value
        self.contact_depth_topic = self.get_parameter(
            "contact_depth_topic"
        ).value

        self.odom = None
        self.imu = None
        self.depth = None
        self.crash = None
        self.crash_reason = ""
        self.contact_force_n = None
        self.contact_depth_m = None

        self.odom_count = 0
        self.imu_count = 0
        self.depth_count = 0
        self.crash_count = 0
        self.contact_count = 0

        qos = qos_profile_sensor_data

        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, qos)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, qos)
        self.create_subscription(Bool, self.crash_topic, self.crash_cb, 10)
        self.create_subscription(
            String,
            self.crash_reason_topic,
            self.crash_reason_cb,
            10,
        )
        self.create_subscription(
            Float32,
            self.contact_force_topic,
            self.contact_force_cb,
            10,
        )
        self.create_subscription(
            Float32,
            self.contact_depth_topic,
            self.contact_depth_cb,
            10,
        )

        self.timer = self.create_timer(1.0, self.print_status)

        self.get_logger().info("Generic UAV state monitor started.")
        self.get_logger().info(f"odom : {self.odom_topic}")
        self.get_logger().info(f"imu  : {self.imu_topic}")
        self.get_logger().info(f"depth: {self.depth_topic}")
        self.get_logger().info(f"crash: {self.crash_topic}")
        self.get_logger().info(f"contact force: {self.contact_force_topic}")
        self.get_logger().info(f"contact depth: {self.contact_depth_topic}")

    def odom_cb(self, msg):
        self.odom = msg
        self.odom_count += 1

    def imu_cb(self, msg):
        self.imu = msg
        self.imu_count += 1

    def depth_cb(self, msg):
        self.depth = msg
        self.depth_count += 1

    def crash_cb(self, msg):
        self.crash = msg
        self.crash_count += 1

    def crash_reason_cb(self, msg):
        self.crash_reason = msg.data

    def contact_force_cb(self, msg):
        self.contact_force_n = float(msg.data)
        self.contact_count += 1

    def contact_depth_cb(self, msg):
        self.contact_depth_m = float(msg.data)

    def print_status(self):
        lines = []
        lines.append("")
        lines.append("===== UAV STATE MONITOR =====")
        lines.append(
            f"rates: odom={self.odom_count:3d} Hz | "
            f"imu={self.imu_count:3d} Hz | "
            f"depth={self.depth_count:3d} Hz | "
            f"crash={self.crash_count:3d} Hz | "
            f"contact={self.contact_count:3d} Hz"
        )

        if self.odom is not None:
            p = self.odom.pose.pose.position
            v = self.odom.twist.twist.linear
            yaw = yaw_from_ros_quat(self.odom.pose.pose.orientation)

            lines.append(
                f"pose ENU: "
                f"x={p.x: .2f}, y={p.y: .2f}, z={p.z: .2f} | "
                f"vel body FLU: "
                f"forward={v.x: .2f}, left={v.y: .2f}, up={v.z: .2f} | "
                f"yaw={math.degrees(yaw): .1f} deg"
            )
        else:
            lines.append(f"Odometry: no data yet on {self.odom_topic}")

        if self.imu is not None:
            g = self.imu.angular_velocity
            a = self.imu.linear_acceleration

            lines.append(
                f"IMU gyro(rad/s): "
                f"[{g.x: .3f}, {g.y: .3f}, {g.z: .3f}] | "
                f"accel(m/s^2): "
                f"[{a.x: .3f}, {a.y: .3f}, {a.z: .3f}]"
            )
        else:
            lines.append(f"IMU: no data yet on {self.imu_topic}")

        if self.depth is not None:
            lines.append(
                f"Depth image: "
                f"{self.depth.width}x{self.depth.height}, "
                f"encoding={self.depth.encoding}"
            )
        else:
            lines.append(f"Depth image: no data yet on {self.depth_topic}")

        if self.contact_force_n is not None:
            depth = (
                self.contact_depth_m
                if self.contact_depth_m is not None
                else 0.0
            )
            lines.append(
                f"Contact: force={self.contact_force_n: .1f} N | "
                f"depth={depth: .4f} m"
            )
        else:
            lines.append(
                f"Contact: no data yet on {self.contact_force_topic}"
            )

        if self.crash is not None:
            state = "CRASH" if self.crash.data else "ok"
            reason = self.crash_reason or "n/a"
            lines.append(f"Crash state: {state} | reason={reason}")
        else:
            lines.append(f"Crash state: no data yet on {self.crash_topic}")

        self.get_logger().info("\n".join(lines))

        self.odom_count = 0
        self.imu_count = 0
        self.depth_count = 0
        self.crash_count = 0


def main():
    rclpy.init()
    node = StateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
