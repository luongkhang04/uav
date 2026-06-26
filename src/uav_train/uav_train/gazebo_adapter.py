from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from std_srvs.srv import Trigger

from .config import ProjectConfig
from .observation import depth_image_to_meters, yaw_from_ros_quat


class RosGazeboAdapter(Node):
    """ROS adapter for the backend-independent `/uav/*` interface."""

    def __init__(self, config: ProjectConfig) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)
        super().__init__("xai_sac_gazebo_adapter")

        self.config = config
        self.ros_cfg = config.ros
        self.env_cfg = config.environment

        self.latest_odom: Odometry | None = None
        self.latest_imu: Imu | None = None
        self.latest_depth: Image | None = None

        qos = qos_profile_sensor_data
        self.create_subscription(
            Odometry,
            self.ros_cfg.odom_topic,
            self._odom_cb,
            qos,
        )
        self.create_subscription(
            Imu,
            self.ros_cfg.imu_topic,
            self._imu_cb,
            qos,
        )
        self.create_subscription(
            Image,
            self.ros_cfg.depth_topic,
            self._depth_cb,
            qos,
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.ros_cfg.cmd_topic,
            10,
        )
        self.arm_client = self.create_client(
            Trigger,
            self.ros_cfg.offboard_arm_service,
        )
        self.land_client = self.create_client(
            Trigger,
            self.ros_cfg.land_service,
        )
        self.disarm_client = self.create_client(
            Trigger,
            self.ros_cfg.disarm_service,
        )

        self.get_logger().info("XAI SAC Gazebo adapter started.")
        self.get_logger().info(f"odom : {self.ros_cfg.odom_topic}")
        self.get_logger().info(f"imu  : {self.ros_cfg.imu_topic}")
        self.get_logger().info(f"depth: {self.ros_cfg.depth_topic}")
        self.get_logger().info(f"cmd  : {self.ros_cfg.cmd_topic}")

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _imu_cb(self, msg: Imu) -> None:
        self.latest_imu = msg

    def _depth_cb(self, msg: Image) -> None:
        self.latest_depth = msg

    def wait_until_ready(self, timeout_sec: float | None = None) -> None:
        timeout = self.ros_cfg.data_timeout_sec
        if timeout_sec is not None:
            timeout = timeout_sec

        deadline = time.monotonic() + float(timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            missing = self.missing_inputs()
            if not missing:
                return
            rclpy.spin_once(self, timeout_sec=0.05)

        missing = ", ".join(self.missing_inputs())
        raise RuntimeError(f"Timed out waiting for ROS inputs: {missing}")

    def missing_inputs(self) -> list[str]:
        missing: list[str] = []
        if self.latest_odom is None:
            missing.append(self.ros_cfg.odom_topic)
        if self.latest_imu is None:
            missing.append(self.ros_cfg.imu_topic)
        if self.latest_depth is None:
            missing.append(self.ros_cfg.depth_topic)
        return missing

    def arm_offboard(self) -> bool:
        return self._call_trigger(
            self.arm_client,
            self.ros_cfg.offboard_arm_service,
        )

    def land(self) -> bool:
        return self._call_trigger(self.land_client, self.ros_cfg.land_service)

    def disarm(self) -> bool:
        return self._call_trigger(
            self.disarm_client,
            self.ros_cfg.disarm_service,
        )

    def _call_trigger(self, client: object, service_name: str) -> bool:
        timeout = float(self.ros_cfg.service_timeout_sec)
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f"Service not available: {service_name}")
            return False

        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                self.get_logger().warn(
                    f"Service call timed out: {service_name}"
                )
                return False
            rclpy.spin_once(self, timeout_sec=0.05)

        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f"Service call failed {service_name}: {exc}"
            )
            return False

        if not bool(result.success):
            self.get_logger().warn(
                f"Service returned failure {service_name}: {result.message}"
            )
            return False
        return True

    def step(self, action: np.ndarray, duration_sec: float) -> None:
        action = np.asarray(action, dtype=np.float32)
        period = 1.0 / max(float(self.ros_cfg.control_rate_hz), 1e-6)
        deadline = time.monotonic() + max(float(duration_sec), 0.0)

        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_action(action)
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self, timeout_sec=min(period, remaining))

    def publish_action(self, action: np.ndarray) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(action[0])
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = float(action[1])
        msg.twist.angular.z = float(action[2])
        self.cmd_pub.publish(msg)

    def publish_zero(self, duration_sec: float = 0.2) -> None:
        zero = np.zeros(3, dtype=np.float32)
        self.step(zero, duration_sec)

    def get_pose(self) -> tuple[np.ndarray, float]:
        self.wait_until_ready()
        odom = self.latest_odom
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        return (
            np.array([position.x, position.y, position.z], dtype=np.float32),
            float(yaw_from_ros_quat(orientation)),
        )

    def get_velocity(self) -> np.ndarray:
        self.wait_until_ready()
        odom = self.latest_odom
        imu = self.latest_imu
        linear = odom.twist.twist.linear
        angular = imu.angular_velocity
        v_xy = float(np.hypot(float(linear.x), float(linear.y)))
        return np.array([v_xy, linear.z, angular.z], dtype=np.float32)

    def get_depth_image(self) -> np.ndarray:
        self.wait_until_ready()
        return depth_image_to_meters(
            self.latest_depth,
            max_depth_m=self.env_cfg.max_depth_meters,
            depth_scale=self.ros_cfg.depth_scale,
        )

    def close(self) -> None:
        if self.ros_cfg.stop_on_close:
            self.publish_zero(duration_sec=0.2)
        self.destroy_node()
