#!/usr/bin/env python3

import math
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from ros_gz_interfaces.msg import Contacts

from px4_msgs.msg import (
    FailureDetectorStatus,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleOdometry,
    VehicleStatus,
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

    This adapter publishes yaw-only orientation for normalized ROS odom.
    """
    qz = math.sin(0.5 * yaw)
    qw = math.cos(0.5 * yaw)
    return 0.0, 0.0, qz, qw


def depth_image_near_m(
    msg,
    max_depth_m,
    depth_scale=1.0,
    min_valid_m=0.05,
    percentile=0.5,
):
    encoding = msg.encoding.upper()
    channels = channels_from_encoding(encoding)

    if encoding.startswith("32FC"):
        dtype = np.dtype(np.float32)
        unit_scale = 1.0
    elif encoding in {"16UC1", "MONO16"} or encoding.startswith("16UC"):
        dtype = np.dtype(np.uint16)
        unit_scale = 0.001
    elif encoding in {"8UC1", "MONO8"} or encoding.startswith("8UC"):
        dtype = np.dtype(np.uint8)
        unit_scale = float(max_depth_m) / 255.0
    else:
        raise RuntimeError(f"Unsupported depth image encoding: {encoding}")

    if msg.is_bigendian:
        dtype = dtype.newbyteorder(">")

    width = int(msg.width)
    height = int(msg.height)
    if width <= 0 or height <= 0:
        raise RuntimeError("Depth image has invalid dimensions.")

    raw = np.frombuffer(msg.data, dtype=dtype)
    row_values = int(msg.step // dtype.itemsize) if msg.step else width * channels
    min_row_values = width * channels

    if row_values >= min_row_values and raw.size >= row_values * height:
        image = raw[:row_values * height].reshape(height, row_values)
        image = image[:, :min_row_values].reshape(height, width, channels)
    else:
        expected = width * height * channels
        if raw.size < expected:
            raise RuntimeError("Depth image data is smaller than expected.")
        image = raw[:expected].reshape(height, width, channels)

    depth = image[:, :, 0].astype(np.float32)
    depth *= float(unit_scale) * float(depth_scale)
    valid = depth[np.isfinite(depth)]
    valid = valid[
        (valid >= float(min_valid_m))
        & (valid <= float(max_depth_m))
    ]
    if valid.size == 0:
        return None, 0.0

    percentile = clamp(float(percentile), 0.0, 100.0)
    near_depth_m = float(np.percentile(valid, percentile))
    valid_ratio = float(valid.size) / float(max(width * height, 1))
    return near_depth_m, valid_ratio


def channels_from_encoding(encoding):
    if encoding.endswith("C4"):
        return 4
    if encoding.endswith("C3"):
        return 3
    if encoding.endswith("C2"):
        return 2
    return 1


def parse_name_list(value):
    if isinstance(value, str):
        raw_items = value.replace(";", ",").split(",")
    else:
        raw_items = value
    return [
        str(item).strip().lower()
        for item in raw_items
        if str(item).strip()
    ]


def vector3_norm(vector):
    return math.sqrt(
        float(vector.x) * float(vector.x)
        + float(vector.y) * float(vector.y)
        + float(vector.z) * float(vector.z)
    )


def wrench_force_norm(wrench):
    return max(
        vector3_norm(wrench.body_1_wrench.force),
        vector3_norm(wrench.body_2_wrench.force),
    )


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
        /uav/odom              nav_msgs/Odometry, ROS ENU pose, body FLU twist
        /uav/imu               sensor_msgs/Imu, ROS FLU-like
        /uav/crash             std_msgs/Bool, Gazebo/PX4 crash state
        /uav/crash_reason      std_msgs/String, human-readable crash source

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
        self.declare_parameter("crash_topic", "/uav/crash")
        self.declare_parameter("crash_reason_topic", "/uav/crash_reason")
        self.declare_parameter("contact_topic", "/uav/gazebo/contacts")
        self.declare_parameter("contact_force_topic", "/uav/contact_force_n")
        self.declare_parameter("contact_depth_topic", "/uav/contact_depth_m")
        self.declare_parameter("contact_timeout_sec", 0.5)
        self.declare_parameter(
            "allowed_contact_names",
            "ground,ground_plane,landing_pad,landingpad,pad,floor",
        )
        self.declare_parameter("max_contact_force_n", 200.0)
        self.declare_parameter("max_contact_depth_m", 0.05)
        self.declare_parameter("gazebo_world", "default")
        self.declare_parameter("gazebo_model_name", "x500_depth_0")
        self.declare_parameter("reset_x", 0.0)
        self.declare_parameter("reset_y", 0.0)
        self.declare_parameter("reset_z", 0.0)
        self.declare_parameter("reset_roll", 0.0)
        self.declare_parameter("reset_pitch", 0.0)
        self.declare_parameter("reset_yaw", 0.0)
        self.declare_parameter("reset_pause", False)
        self.declare_parameter("reset_settle_sec", 0.5)
        self.declare_parameter("gz_service_timeout_ms", 10000)
        self.declare_parameter("depth_topic", "/uav/camera/depth/image")
        self.declare_parameter("use_depth_crash_fallback", True)
        self.declare_parameter("depth_crash_distance", 1.5)
        self.declare_parameter("depth_max_meters", 15.0)
        self.declare_parameter("depth_scale", 1.0)
        self.declare_parameter("depth_timeout_sec", 1.0)
        self.declare_parameter("depth_min_valid_m", 0.05)
        self.declare_parameter("depth_crash_percentile", 0.5)
        self.declare_parameter("depth_crash_confirmations", 3)
        self.declare_parameter("depth_ignore_when_landed", True)
        self.declare_parameter("depth_min_airborne_altitude", 0.75)
        self.declare_parameter("land_detect_timeout_sec", 1.0)

        self.declare_parameter('max_xy_speed', PX4_MAX_XY_SPEED)
        self.declare_parameter('max_z_speed_up', PX4_MAX_Z_SPEED_UP)
        self.declare_parameter('max_z_speed_down', PX4_MAX_Z_SPEED_DOWN)
        self.declare_parameter('max_z_speed', -1.0)
        self.declare_parameter("max_yaw_rate", 1.0)

        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.crash_topic = self.get_parameter("crash_topic").value
        self.crash_reason_topic = self.get_parameter(
            "crash_reason_topic"
        ).value
        self.contact_topic = self.get_parameter("contact_topic").value
        self.contact_force_topic = self.get_parameter(
            "contact_force_topic"
        ).value
        self.contact_depth_topic = self.get_parameter(
            "contact_depth_topic"
        ).value
        self.contact_timeout_sec = float(
            self.get_parameter("contact_timeout_sec").value
        )
        self.allowed_contact_names = parse_name_list(
            self.get_parameter("allowed_contact_names").value
        )
        self.max_contact_force_n = float(
            self.get_parameter("max_contact_force_n").value
        )
        self.max_contact_depth_m = float(
            self.get_parameter("max_contact_depth_m").value
        )
        self.gazebo_world = self.get_parameter("gazebo_world").value
        self.gazebo_model_name = self.get_parameter("gazebo_model_name").value
        self.reset_pose = (
            float(self.get_parameter("reset_x").value),
            float(self.get_parameter("reset_y").value),
            float(self.get_parameter("reset_z").value),
            float(self.get_parameter("reset_roll").value),
            float(self.get_parameter("reset_pitch").value),
            float(self.get_parameter("reset_yaw").value),
        )
        self.reset_pause = bool(self.get_parameter("reset_pause").value)
        self.reset_settle_sec = float(
            self.get_parameter("reset_settle_sec").value
        )
        self.gz_service_timeout_ms = int(
            self.get_parameter("gz_service_timeout_ms").value
        )
        self.depth_topic = self.get_parameter("depth_topic").value
        self.use_depth_crash_fallback = bool(
            self.get_parameter("use_depth_crash_fallback").value
        )
        self.depth_crash_distance = float(
            self.get_parameter("depth_crash_distance").value
        )
        self.depth_max_meters = float(
            self.get_parameter("depth_max_meters").value
        )
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.depth_timeout_sec = float(
            self.get_parameter("depth_timeout_sec").value
        )
        self.depth_min_valid_m = float(
            self.get_parameter("depth_min_valid_m").value
        )
        self.depth_crash_percentile = float(
            self.get_parameter("depth_crash_percentile").value
        )
        self.depth_crash_confirmations = max(
            1,
            int(self.get_parameter("depth_crash_confirmations").value),
        )
        self.depth_ignore_when_landed = bool(
            self.get_parameter("depth_ignore_when_landed").value
        )
        self.depth_min_airborne_altitude = float(
            self.get_parameter("depth_min_airborne_altitude").value
        )
        self.land_detect_timeout_sec = float(
            self.get_parameter("land_detect_timeout_sec").value
        )

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
        self.current_altitude_m = None
        self.last_contact_time = 0.0
        self.last_contact_count = 0
        self.last_contact_pair = ""
        self.last_contact_force_n = 0.0
        self.last_contact_depth_m = 0.0
        self.last_contact_allowed = False
        self.last_contact_crash = False
        self.last_contact_reason = ""
        self.px4_failure_flags = []
        self.px4_nav_termination = False
        self.last_land_detect_time = 0.0
        self.px4_ground_contact = False
        self.px4_maybe_landed = False
        self.px4_landed = False
        self.latest_min_depth_m = None
        self.latest_depth_valid_ratio = 0.0
        self.latest_depth_time = 0.0
        self.latest_depth_error = ""
        self.depth_crash_streak = 0

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

        self.create_subscription(
            FailureDetectorStatus,
            "/fmu/out/failure_detector_status",
            self.failure_detector_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            VehicleLandDetected,
            "/fmu/out/vehicle_land_detected",
            self.vehicle_land_detected_callback,
            qos_profile_sensor_data,
        )

        if self.contact_topic:
            self.create_subscription(
                Contacts,
                self.contact_topic,
                self.contact_callback,
                qos_profile_sensor_data,
            )

        if self.use_depth_crash_fallback:
            self.create_subscription(
                Image,
                self.depth_topic,
                self.depth_callback,
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
        self.crash_pub = self.create_publisher(Bool, self.crash_topic, 10)
        self.crash_reason_pub = self.create_publisher(
            String,
            self.crash_reason_topic,
            10,
        )
        self.contact_force_pub = self.create_publisher(
            Float32,
            self.contact_force_topic,
            10,
        )
        self.contact_depth_pub = self.create_publisher(
            Float32,
            self.contact_depth_topic,
            10,
        )

        # Services.
        self.create_service(Trigger, "/uav/offboard_arm", self.offboard_arm_cb)
        self.create_service(Trigger, "/uav/disarm", self.disarm_cb)
        self.create_service(Trigger, "/uav/land", self.land_cb)
        self.create_service(Trigger, "/uav/reset_sim", self.reset_sim_cb)

        # PX4 Offboard setpoints must be streamed continuously.
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        self.get_logger().info("PX4 backend adapter started.")
        self.get_logger().info(f"Control input : {self.cmd_topic}")
        self.get_logger().info(
            f"State output  : {self.odom_topic}, {self.imu_topic}, "
            f"{self.crash_topic}"
        )
        self.get_logger().info(f"Crash reason  : {self.crash_reason_topic}")
        self.get_logger().info(
            f"Gazebo contact: {self.contact_topic or 'disabled'}"
        )
        self.get_logger().info(
            f"Contact limits: allowed={self.allowed_contact_names} "
            f"force<={self.max_contact_force_n:.1f} N "
            f"depth<={self.max_contact_depth_m:.3f} m"
        )
        self.get_logger().info(
            f"Depth fallback: {self.depth_topic} "
            f"near p{self.depth_crash_percentile:g} "
            f"< {self.depth_crash_distance:.2f} m for "
            f"{self.depth_crash_confirmations} frames, ignored below "
            f"{self.depth_min_airborne_altitude:.2f} m"
        )
        self.get_logger().info(
            'Arm/offboard : ros2 service call /uav/offboard_arm '
            'std_srvs/srv/Trigger "{}"'
        )
        self.get_logger().info(
            'Land         : ros2 service call /uav/land '
            'std_srvs/srv/Trigger "{}"'
        )
        self.get_logger().info(
            'Disarm       : ros2 service call /uav/disarm '
            'std_srvs/srv/Trigger "{}"'
        )
        self.get_logger().info(
            'Reset sim    : ros2 service call /uav/reset_sim '
            'std_srvs/srv/Trigger "{}"'
        )

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def cmd_callback(self, msg):
        self.latest_cmd = msg

    def contact_callback(self, msg):
        self.last_contact_count = len(msg.contacts)
        if self.last_contact_count <= 0:
            self.clear_contact_state()
            return

        self.last_contact_time = time.monotonic()
        max_force_n = 0.0
        max_depth_m = 0.0
        max_pair = ""
        all_allowed = True
        crash_reason = ""

        for contact in msg.contacts:
            pair = self.contact_pair_name(contact)
            force_n = self.contact_force_n(contact)
            depth_m = self.contact_depth_m(contact)
            allowed = self.contact_pair_allowed(pair)

            if force_n > max_force_n or depth_m > max_depth_m:
                max_pair = pair
            max_force_n = max(max_force_n, force_n)
            max_depth_m = max(max_depth_m, depth_m)

            if not allowed:
                all_allowed = False
                if not crash_reason:
                    crash_reason = (
                        "gazebo_contact:unallowed "
                        f"pair={pair} force_n={force_n:.1f} "
                        f"depth_m={depth_m:.4f}"
                    )

            if (
                self.max_contact_force_n > 0.0
                and force_n > self.max_contact_force_n
                and not crash_reason
            ):
                crash_reason = (
                    "gazebo_contact:force "
                    f"force_n={force_n:.1f}>"
                    f"max_contact_force_n="
                    f"{self.max_contact_force_n:.1f} "
                    f"pair={pair}"
                )

            if (
                self.max_contact_depth_m > 0.0
                and depth_m > self.max_contact_depth_m
                and not crash_reason
            ):
                crash_reason = (
                    "gazebo_contact:depth "
                    f"depth_m={depth_m:.4f}>"
                    f"max_contact_depth_m="
                    f"{self.max_contact_depth_m:.4f} "
                    f"pair={pair}"
                )

        self.last_contact_pair = max_pair or self.contact_pair_name(
            msg.contacts[0]
        )
        self.last_contact_force_n = max_force_n
        self.last_contact_depth_m = max_depth_m
        self.last_contact_allowed = all_allowed
        self.last_contact_crash = bool(crash_reason)
        self.last_contact_reason = crash_reason

    def clear_contact_state(self):
        self.last_contact_count = 0
        self.last_contact_pair = ""
        self.last_contact_force_n = 0.0
        self.last_contact_depth_m = 0.0
        self.last_contact_allowed = False
        self.last_contact_crash = False
        self.last_contact_reason = ""

    def contact_pair_name(self, contact):
        name1 = contact.collision1.name or str(contact.collision1.id)
        name2 = contact.collision2.name or str(contact.collision2.id)
        return f"{name1} <-> {name2}"

    def contact_pair_allowed(self, pair):
        pair_lower = pair.lower()
        return any(name in pair_lower for name in self.allowed_contact_names)

    def contact_force_n(self, contact):
        if not contact.wrenches:
            return 0.0
        return max(wrench_force_norm(wrench) for wrench in contact.wrenches)

    def contact_depth_m(self, contact):
        if not contact.depths:
            return 0.0
        return max(abs(float(depth)) for depth in contact.depths)

    def depth_callback(self, msg):
        self.latest_depth_time = time.monotonic()
        try:
            near_depth_m, valid_ratio = depth_image_near_m(
                msg,
                self.depth_max_meters,
                self.depth_scale,
                self.depth_min_valid_m,
                self.depth_crash_percentile,
            )
            self.latest_min_depth_m = near_depth_m
            self.latest_depth_valid_ratio = valid_ratio
            self.latest_depth_error = (
                "" if near_depth_m is not None else "no_valid_depth"
            )
        except RuntimeError as exc:
            self.latest_min_depth_m = None
            self.latest_depth_valid_ratio = 0.0
            self.latest_depth_error = str(exc)

        if (
            self.latest_min_depth_m is not None
            and self.latest_min_depth_m < self.depth_crash_distance
            and not self.depth_fallback_suppression_reason()
        ):
            self.depth_crash_streak += 1
        else:
            self.depth_crash_streak = 0

    def failure_detector_callback(self, msg):
        flags = []
        for name in [
            "fd_roll",
            "fd_pitch",
            "fd_alt",
            "fd_ext",
            "fd_arm_escs",
            "fd_battery",
            "fd_imbalanced_prop",
            "fd_motor",
        ]:
            if bool(getattr(msg, name, False)):
                flags.append(name.removeprefix("fd_"))
        self.px4_failure_flags = flags

    def vehicle_status_callback(self, msg):
        self.px4_nav_termination = (
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_TERMINATION
        )

    def vehicle_land_detected_callback(self, msg):
        self.last_land_detect_time = time.monotonic()
        self.px4_ground_contact = bool(msg.ground_contact)
        self.px4_maybe_landed = bool(msg.maybe_landed)
        self.px4_landed = bool(msg.landed)

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
        self.current_altitude_m = -d

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
        self.publish_crash_state()
        self.publish_contact_metrics()

    def publish_contact_metrics(self):
        force_msg = Float32()
        depth_msg = Float32()
        if self.contact_is_recent():
            force_msg.data = float(self.last_contact_force_n)
            depth_msg.data = float(self.last_contact_depth_m)
        else:
            force_msg.data = 0.0
            depth_msg.data = 0.0
        self.contact_force_pub.publish(force_msg)
        self.contact_depth_pub.publish(depth_msg)

    def publish_crash_state(self):
        reasons = self.crash_reasons()

        crash_msg = Bool()
        crash_msg.data = bool(reasons)
        self.crash_pub.publish(crash_msg)

        reason_msg = String()
        reason_msg.data = "; ".join(reasons) if reasons else self.ok_reason()
        self.crash_reason_pub.publish(reason_msg)

    def crash_reasons(self):
        reasons = []

        if self.contact_is_recent() and self.last_contact_crash:
            reasons.append(self.last_contact_reason)

        if self.px4_failure_flags:
            reasons.append(
                "px4_failure_detector:"
                + ",".join(self.px4_failure_flags)
            )

        if self.px4_nav_termination:
            reasons.append("px4_nav_state:termination")

        if self.depth_fallback_crashed():
            reasons.append(
                "depth_fallback:"
                f"near_depth_m={self.latest_min_depth_m:.3f}<"
                f"crash_distance_m={self.depth_crash_distance:.3f} "
                f"streak={self.depth_crash_streak}/"
                f"{self.depth_crash_confirmations}"
            )

        return reasons

    def contact_is_recent(self):
        return (
            self.last_contact_count > 0
            and time.monotonic() - self.last_contact_time
            <= self.contact_timeout_sec
        )

    def contact_ok_reason(self):
        if not self.contact_is_recent():
            return ""
        allowed = "allowed" if self.last_contact_allowed else "unallowed"
        return (
            f"ok contact_{allowed}:count={self.last_contact_count} "
            f"force_n={self.last_contact_force_n:.1f} "
            f"depth_m={self.last_contact_depth_m:.4f} "
            f"pair={self.last_contact_pair}"
        )

    def depth_fallback_crashed(self):
        if not self.depth_is_recent() or self.latest_min_depth_m is None:
            return False
        if self.depth_fallback_suppression_reason():
            return False
        return (
            self.latest_min_depth_m < self.depth_crash_distance
            and self.depth_crash_streak >= self.depth_crash_confirmations
        )

    def depth_is_recent(self):
        if not self.use_depth_crash_fallback or self.latest_depth_time <= 0.0:
            return False
        return time.monotonic() - self.latest_depth_time <= self.depth_timeout_sec

    def land_detect_is_recent(self):
        if self.last_land_detect_time <= 0.0:
            return False
        return (
            time.monotonic() - self.last_land_detect_time
            <= self.land_detect_timeout_sec
        )

    def depth_fallback_suppression_reason(self):
        if not self.use_depth_crash_fallback:
            return ""

        low_altitude_reason = self.low_altitude_suppression_reason()
        if low_altitude_reason:
            return low_altitude_reason

        # If odometry altitude is unavailable, PX4 land detection is the only
        # landing signal left. Once altitude is known, do not let "landed"
        # hide a touchdown on a high obstacle/platform.
        if (
            self.depth_ignore_when_landed
            and self.current_altitude_m is None
            and self.land_detect_is_recent()
        ):
            return self.px4_landed_suppression_reason()

        return ""

    def low_altitude_suppression_reason(self):
        if (
            self.depth_min_airborne_altitude > 0.0
            and self.current_altitude_m is not None
            and self.current_altitude_m < self.depth_min_airborne_altitude
        ):
            return (
                f"low_altitude:altitude_m={self.current_altitude_m:.3f}<"
                f"min_airborne_altitude_m="
                f"{self.depth_min_airborne_altitude:.3f}"
            )
        return ""

    def px4_landed_suppression_reason(self):
        if self.depth_ignore_when_landed and self.land_detect_is_recent():
            landed_flags = []
            if self.px4_landed:
                landed_flags.append("landed")
            if self.px4_maybe_landed:
                landed_flags.append("maybe_landed")
            if self.px4_ground_contact:
                landed_flags.append("ground_contact")
            if landed_flags:
                return "px4_land_detected:" + ",".join(landed_flags)
        return ""

    def ok_reason(self):
        contact_reason = self.contact_ok_reason()
        if contact_reason:
            return contact_reason
        if self.depth_is_recent() and self.latest_min_depth_m is not None:
            suppression = self.depth_fallback_suppression_reason()
            if suppression:
                return (
                    "ok depth_suppressed:"
                    f"{suppression} "
                    f"near_depth_m={self.latest_min_depth_m:.3f}"
                )
            if self.latest_min_depth_m < self.depth_crash_distance:
                return (
                    f"ok depth_pending:near_depth_m="
                    f"{self.latest_min_depth_m:.3f} "
                    f"streak={self.depth_crash_streak}/"
                    f"{self.depth_crash_confirmations}"
                )
            return (
                f"ok depth_near_m={self.latest_min_depth_m:.3f} "
                f"valid={100.0 * self.latest_depth_valid_ratio:.1f}%"
            )
        if self.latest_depth_error:
            return f"ok depth_error={self.latest_depth_error}"
        if self.use_depth_crash_fallback:
            return "ok depth_fallback=no_depth"
        return "ok"

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

    def force_disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
            param2=21196.0,
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

    def reset_sim_cb(self, request, response):
        self.latest_cmd = TwistStamped()
        self.latest_cmd.header.frame_id = "base_link"

        # PX4 can reject a normal disarm while airborne; force-disarm is used
        # here because this service is only intended for simulation resets.
        for _ in range(5):
            self.force_disarm()
            self.publish_offboard_control_mode()
            self.publish_velocity_setpoint()
            time.sleep(0.05)

        paused = False
        if self.reset_pause:
            paused = self._gz_control("pause: true")
            if not paused:
                self.get_logger().warn(
                    "Failed to pause Gazebo before reset; "
                    "continuing with set_pose."
                )

        try:
            if not self._gz_set_pose():
                response.success = False
                response.message = "Failed to set Gazebo model pose."
                return response
        finally:
            if paused and not self._gz_control("pause: false"):
                self.get_logger().warn("Failed to unpause Gazebo after reset.")

        self.clear_contact_state()
        self.px4_failure_flags = []
        self.px4_nav_termination = False
        self.latest_min_depth_m = None
        self.latest_depth_error = ""
        if self.reset_settle_sec > 0.0:
            time.sleep(self.reset_settle_sec)

        response.success = True
        response.message = (
            f"Reset {self.gazebo_model_name} in world {self.gazebo_world}."
        )
        return response

    def _gz_control(self, request_text):
        return self._call_gz_service(
            f"/world/{self.gazebo_world}/control",
            "gz.msgs.WorldControl",
            request_text,
        )

    def _gz_set_pose(self):
        x, y, z, roll, pitch, yaw = self.reset_pose
        request_text = (
            f'name: "{self.gazebo_model_name}", '
            f"position {{x: {x}, y: {y}, z: {z}}}, "
            f"orientation {self._quat_request(roll, pitch, yaw)}"
        )
        return self._call_gz_service(
            f"/world/{self.gazebo_world}/set_pose",
            "gz.msgs.Pose",
            request_text,
        )

    def _call_gz_service(self, service, request_type, request_text):
        command = [
            "gz",
            "service",
            "-s",
            service,
            "--reqtype",
            request_type,
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(self.gz_service_timeout_ms),
            "--req",
            request_text,
        ]
        try:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
            )
        except OSError as exc:
            self.get_logger().warn(f"Failed to run gz service: {exc}")
            return False

        if result.returncode != 0:
            self.get_logger().warn(
                f"gz service failed ({service}): "
                f"{result.stderr.strip() or result.stdout.strip()}"
            )
            return False
        return True

    def _quat_request(self, roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return f"{{w: {w}, x: {x}, y: {y}, z: {z}}}"


def main():
    rclpy.init()
    node = PX4BackendAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
