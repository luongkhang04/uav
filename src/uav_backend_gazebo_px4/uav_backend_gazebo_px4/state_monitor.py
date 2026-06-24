#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleOdometry, SensorCombined


def yaw_from_quat_wxyz(q):
    if q is None or len(q) < 4:
        return 0.0

    w = float(q[0])
    x = float(q[1])
    y = float(q[2])
    z = float(q[3])

    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class StateMonitor(Node):
    def __init__(self):
        super().__init__("uav_state_monitor")

        self.declare_parameter("depth_topic", "/uav/camera/depth/image")
        self.depth_topic = self.get_parameter("depth_topic").value

        self.odom = None
        self.imu = None
        self.depth = None

        self.odom_count = 0
        self.imu_count = 0
        self.depth_count = 0

        qos = qos_profile_sensor_data

        self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odom_cb,
            qos,
        )

        self.create_subscription(
            SensorCombined,
            "/fmu/out/sensor_combined",
            self.imu_cb,
            qos,
        )

        self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_cb,
            qos,
        )

        self.timer = self.create_timer(1.0, self.print_status)

        self.get_logger().info("State monitor started.")
        self.get_logger().info("Subscribing with sensor_data QoS:")
        self.get_logger().info("  /fmu/out/vehicle_odometry")
        self.get_logger().info("  /fmu/out/sensor_combined")
        self.get_logger().info(f"  {self.depth_topic}")

    def odom_cb(self, msg):
        self.odom = msg
        self.odom_count += 1

    def imu_cb(self, msg):
        self.imu = msg
        self.imu_count += 1

    def depth_cb(self, msg):
        self.depth = msg
        self.depth_count += 1

    def print_status(self):
        lines = []
        lines.append("")
        lines.append("===== UAV STATE MONITOR =====")
        lines.append(
            f"rates: odom={self.odom_count:3d} Hz | "
            f"imu={self.imu_count:3d} Hz | "
            f"depth={self.depth_count:3d} Hz"
        )

        if self.odom is not None:
            p = self.odom.position
            v = self.odom.velocity
            yaw = yaw_from_quat_wxyz(self.odom.q)

            lines.append(
                f"PX4 NED pos: "
                f"N={p[0]: .2f}, E={p[1]: .2f}, D={p[2]: .2f} | "
                f"vel: vN={v[0]: .2f}, vE={v[1]: .2f}, vD={v[2]: .2f} | "
                f"yaw={math.degrees(yaw): .1f} deg"
            )
        else:
            lines.append("PX4 odometry: no data yet")

        if self.imu is not None:
            gyro = self.imu.gyro_rad
            accel = self.imu.accelerometer_m_s2

            lines.append(
                f"IMU gyro(rad/s): "
                f"[{gyro[0]: .3f}, {gyro[1]: .3f}, {gyro[2]: .3f}] | "
                f"accel(m/s^2): "
                f"[{accel[0]: .3f}, {accel[1]: .3f}, {accel[2]: .3f}]"
            )
        else:
            lines.append("IMU: no data yet")

        if self.depth is not None:
            lines.append(
                f"Depth image: "
                f"{self.depth.width}x{self.depth.height}, "
                f"encoding={self.depth.encoding}"
            )
        else:
            lines.append(f"Depth image: no data yet on {self.depth_topic}")

        self.get_logger().info("\n".join(lines))

        self.odom_count = 0
        self.imu_count = 0
        self.depth_count = 0


def main():
    rclpy.init()
    node = StateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
