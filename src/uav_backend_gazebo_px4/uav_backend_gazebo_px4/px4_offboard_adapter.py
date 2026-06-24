#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def yaw_from_quat_wxyz(q):
    if q is None or len(q) < 4:
        return 0.0
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class PX4OffboardAdapter(Node):
    """
    Input:
      /uav/cmd_vel_body: geometry_msgs/TwistStamped, frame_id=base_link, ROS FLU
        x forward, y left, z up, yaw positive left

    Output:
      PX4 Offboard velocity setpoint in local NED.
    """

    def __init__(self):
        super().__init__("px4_offboard_adapter")

        self.declare_parameter("cmd_topic", "/uav/cmd_vel_body")
        self.declare_parameter("max_xy_speed", 2.0)
        self.declare_parameter("max_z_speed", 1.0)
        self.declare_parameter("max_yaw_rate", 1.0)

        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.max_xy_speed = float(self.get_parameter("max_xy_speed").value)
        self.max_z_speed = float(self.get_parameter("max_z_speed").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)

        self.latest_cmd = TwistStamped()
        self.latest_cmd.header.frame_id = "base_link"
        self.current_yaw_ned = 0.0

        self.create_subscription(
            TwistStamped,
            self.cmd_topic,
            self.cmd_callback,
            10,
        )

        self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            10,
        )

        self.traj_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            10,
        )

        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10,
        )

        self.create_service(Trigger, "/uav/offboard_arm", self.offboard_arm_cb)
        self.create_service(Trigger, "/uav/disarm", self.disarm_cb)
        self.create_service(Trigger, "/uav/land", self.land_cb)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        self.get_logger().info("PX4 Offboard adapter started.")
        self.get_logger().info('Arm/offboard: ros2 service call /uav/offboard_arm std_srvs/srv/Trigger "{}"')
        self.get_logger().info('Land:         ros2 service call /uav/land std_srvs/srv/Trigger "{}"')

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def cmd_callback(self, msg):
        self.latest_cmd = msg

    def odom_callback(self, msg):
        self.current_yaw_ned = yaw_from_quat_wxyz(msg.q)

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_velocity_setpoint()

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.now_us()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        # Newer px4_msgs may include these fields.
        if hasattr(msg, "thrust_and_torque"):
            msg.thrust_and_torque = False
        if hasattr(msg, "direct_actuator"):
            msg.direct_actuator = False

        self.offboard_pub.publish(msg)

    def body_flu_cmd_to_px4_ned(self):
        cmd = self.latest_cmd.twist

        # ROS body FLU command.
        forward = float(cmd.linear.x)
        left = float(cmd.linear.y)
        up = float(cmd.linear.z)
        yaw_left = float(cmd.angular.z)

        # Limit horizontal velocity.
        xy_norm = math.hypot(forward, left)
        if xy_norm > self.max_xy_speed and xy_norm > 1e-6:
            scale = self.max_xy_speed / xy_norm
            forward *= scale
            left *= scale

        up = clamp(up, -self.max_z_speed, self.max_z_speed)
        yaw_left = clamp(yaw_left, -self.max_yaw_rate, self.max_yaw_rate)

        # ROS body FLU -> PX4 body FRD.
        body_forward = forward
        body_right = -left
        body_down = -up

        # Rotate body FRD horizontal velocity into local NED.
        yaw = self.current_yaw_ned
        c = math.cos(yaw)
        s = math.sin(yaw)

        v_n = c * body_forward - s * body_right
        v_e = s * body_forward + c * body_right
        v_d = body_down

        # ROS yaw-left positive -> PX4 NED yawspeed sign.
        yawspeed_ned = -yaw_left

        return v_n, v_e, v_d, yawspeed_ned

    def publish_velocity_setpoint(self):
        v_n, v_e, v_d, yawspeed = self.body_flu_cmd_to_px4_ned()

        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [float(v_n), float(v_e), float(v_d)]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = math.nan
        msg.yawspeed = float(yawspeed)

        self.traj_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)

    def set_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
        )

    def land(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
        )

    def offboard_arm_cb(self, request, response):
        self.set_offboard_mode()
        self.arm()
        response.success = True
        response.message = "Sent Offboard mode and arm commands."
        return response

    def disarm_cb(self, request, response):
        self.disarm()
        response.success = True
        response.message = "Sent disarm command."
        return response

    def land_cb(self, request, response):
        self.land()
        response.success = True
        response.message = "Sent land command."
        return response


def main():
    rclpy.init()
    node = PX4OffboardAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
