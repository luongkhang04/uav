#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
    SensorCombined,
)

PX4_MAX_XY_SPEED = 12.0
PX4_MAX_Z_SPEED_UP = 3.0
PX4_MAX_Z_SPEED_DOWN = 1.5


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_px4_quat_wxyz(q):
    """
    PX4 quaternion array order: [w, x, y, z].
    This extracts yaw in PX4 local NED convention.
    """
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


def ros_quat_from_yaw(yaw):
    """
    ROS quaternion fields: x, y, z, w.
    This adapter publishes yaw-only orientation for the normalized ROS odom topic.
    """
    qz = math.sin(0.5 * yaw)
    qw = math.cos(0.5 * yaw)
    return 0.0, 0.0, qz, qw


class PX4BackendAdapter(Node):
    """
    PX4 backend adapter.

    Backend-specific input/output:
      PX4 in:
        /fmu/in/offboard_control_mode
        /fmu/in/trajectory_setpoint
        /fmu/in/vehicle_command

      PX4 out:
        /fmu/out/vehicle_odometry
        /fmu/out/sensor_combined

    Backend-independent interface:
      Control input:
        /uav/cmd_vel_body      geometry_msgs/TwistStamped, ROS body FLU

      State output:
        /uav/odom              nav_msgs/Odometry, pose in ROS ENU, twist in body FLU
        /uav/imu               sensor_msgs/Imu, ROS FLU-like

      Services:
        /uav/offboard_arm
        /uav/disarm
        /uav/land
    """

    def __init__(self):
        super().__init__("px4_backend_adapter")

        self.declare_parameter("cmd_topic", "/uav/cmd_vel_body")
        self.declare_parameter("odom_topic", "/uav/odom")
        self.declare_parameter("imu_topic", "/uav/imu")

        self.declare_parameter('max_xy_speed', PX4_MAX_XY_SPEED)
        self.declare_parameter('max_z_speed_up', PX4_MAX_Z_SPEED_UP)
        self.declare_parameter('max_z_speed_down', PX4_MAX_Z_SPEED_DOWN)
        self.declare_parameter('max_z_speed', -1.0)
        self.declare_parameter("max_yaw_rate", 1.0)

        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value

        self.max_xy_speed = float(self.get_parameter('max_xy_speed').value)
        self.max_z_speed_up = float(self.get_parameter('max_z_speed_up').value)
        self.max_z_speed_down = float(
            self.get_parameter('max_z_speed_down').value
        )
        max_z_speed = float(self.get_parameter('max_z_speed').value)
        if max_z_speed > 0.0:
            self.max_z_speed_up = max_z_speed
            self.max_z_speed_down = max_z_speed
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)

        self.latest_cmd = TwistStamped()
        self.latest_cmd.header.frame_id = "base_link"

        self.current_yaw_ned = 0.0

        # Command input from keyboard/RL/planner.
        self.create_subscription(
            TwistStamped,
            self.cmd_topic,
            self.cmd_callback,
            10,
        )

        # PX4 state input. PX4 output topics require sensor_data QoS.
        self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            SensorCombined,
            "/fmu/out/sensor_combined",
            self.imu_callback,
            qos_profile_sensor_data,
        )

        # PX4 Offboard output.
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

        # Normalized backend-independent state output.
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)

        # Services.
        self.create_service(Trigger, "/uav/offboard_arm", self.offboard_arm_cb)
        self.create_service(Trigger, "/uav/disarm", self.disarm_cb)
        self.create_service(Trigger, "/uav/land", self.land_cb)

        # PX4 Offboard setpoints must be streamed continuously.
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        self.get_logger().info("PX4 backend adapter started.")
        self.get_logger().info(f"Control input : {self.cmd_topic}")
        self.get_logger().info(f"State output  : {self.odom_topic}, {self.imu_topic}")
        self.get_logger().info('Arm/offboard : ros2 service call /uav/offboard_arm std_srvs/srv/Trigger "{}"')
        self.get_logger().info('Land         : ros2 service call /uav/land std_srvs/srv/Trigger "{}"')
        self.get_logger().info('Disarm       : ros2 service call /uav/disarm std_srvs/srv/Trigger "{}"')

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def cmd_callback(self, msg):
        self.latest_cmd = msg

    def odom_callback(self, msg):
        """
        PX4 VehicleOdometry -> normalized ROS nav_msgs/Odometry.

        PX4 local frame:
          NED: x=N, y=E, z=D

        ROS normalized pose frame:
          ENU: x=E, y=N, z=U=-D

        ROS normalized twist frame:
          body FLU: x=forward, y=left, z=up
        """
        self.current_yaw_ned = yaw_from_px4_quat_wxyz(msg.q)

        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "odom"
        out.child_frame_id = "base_link"

        n = float(msg.position[0])
        e = float(msg.position[1])
        d = float(msg.position[2])

        out.pose.pose.position.x = e
        out.pose.pose.position.y = n
        out.pose.pose.position.z = -d

        vn = float(msg.velocity[0])
        ve = float(msg.velocity[1])
        vd = float(msg.velocity[2])

        # PX4 velocity is expressed in local NED:
        #   vn = North velocity
        #   ve = East velocity
        #   vd = Down velocity
        #
        # Convert horizontal velocity from local NED to body FRD.
        # Earlier, command conversion used:
        #   v_n = cos(yaw) * forward - sin(yaw) * right
        #   v_e = sin(yaw) * forward + cos(yaw) * right
        #
        # Inverse:
        #   forward = cos(yaw) * v_n + sin(yaw) * v_e
        #   right   = -sin(yaw) * v_n + cos(yaw) * v_e
        #
        # Then convert body FRD -> ROS body FLU:
        #   forward = forward
        #   left    = -right
        #   up      = -down
        yaw_ned = self.current_yaw_ned
        c = math.cos(yaw_ned)
        s = math.sin(yaw_ned)

        body_forward = c * vn + s * ve
        body_right = -s * vn + c * ve
        body_down = vd

        out.twist.twist.linear.x = body_forward
        out.twist.twist.linear.y = -body_right
        out.twist.twist.linear.z = -body_down

        # Yaw-only NED -> ENU conversion:
        # PX4 NED yaw: 0 = North, +90 deg = East.
        # ROS ENU yaw: 0 = East, +90 deg = North.
        yaw_enu = wrap_pi(math.pi / 2.0 - self.current_yaw_ned)
        qx, qy, qz, qw = ros_quat_from_yaw(yaw_enu)

        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw

        self.odom_pub.publish(out)

    def imu_callback(self, msg):
        """
        PX4 SensorCombined -> normalized ROS sensor_msgs/Imu.

        Approximate body frame conversion:
          PX4 body FRD: x forward, y right, z down
          ROS body FLU: x forward, y left,  z up
        """
        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"

        gyro = msg.gyro_rad
        accel = msg.accelerometer_m_s2

        out.angular_velocity.x = float(gyro[0])
        out.angular_velocity.y = -float(gyro[1])
        out.angular_velocity.z = -float(gyro[2])

        out.linear_acceleration.x = float(accel[0])
        out.linear_acceleration.y = -float(accel[1])
        out.linear_acceleration.z = -float(accel[2])

        # SensorCombined does not provide orientation.
        out.orientation_covariance[0] = -1.0

        self.imu_pub.publish(out)

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

        up = clamp(up, -self.max_z_speed_down, self.max_z_speed_up)
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
    node = PX4BackendAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
