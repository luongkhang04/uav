#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    home = os.path.expanduser("~")

    px4_dir = LaunchConfiguration("px4_dir")
    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")

    # Default: GUI mode.
    # Set headless:=true only when running over SSH/headless server.
    headless = LaunchConfiguration("headless")

    software_render = LaunchConfiguration("software_render")

    start_agent = LaunchConfiguration("start_agent")
    start_gcs = LaunchConfiguration("start_gcs")
    start_bridge = LaunchConfiguration("start_bridge")
    start_contact_bridge = LaunchConfiguration("start_contact_bridge")
    start_adapter = LaunchConfiguration("start_adapter")

    depth_gz_topic = LaunchConfiguration("depth_gz_topic")
    depth_ros_topic = LaunchConfiguration("depth_ros_topic")
    contact_gz_topic = LaunchConfiguration("contact_gz_topic")
    contact_ros_topic = LaunchConfiguration("contact_ros_topic")
    contact_force_topic = LaunchConfiguration("contact_force_topic")
    contact_depth_topic = LaunchConfiguration("contact_depth_topic")
    allowed_contact_names = LaunchConfiguration("allowed_contact_names")
    max_contact_force_n = LaunchConfiguration("max_contact_force_n")
    max_contact_depth_m = LaunchConfiguration("max_contact_depth_m")
    crash_topic = LaunchConfiguration("crash_topic")
    crash_reason_topic = LaunchConfiguration("crash_reason_topic")
    gazebo_model_name = LaunchConfiguration("gazebo_model_name")
    reset_x = LaunchConfiguration("reset_x")
    reset_y = LaunchConfiguration("reset_y")
    reset_z = LaunchConfiguration("reset_z")
    reset_roll = LaunchConfiguration("reset_roll")
    reset_pitch = LaunchConfiguration("reset_pitch")
    reset_yaw = LaunchConfiguration("reset_yaw")
    reset_pause = LaunchConfiguration("reset_pause")
    depth_min_airborne_altitude = LaunchConfiguration(
        "depth_min_airborne_altitude"
    )
    depth_min_valid_m = LaunchConfiguration("depth_min_valid_m")
    depth_crash_percentile = LaunchConfiguration(
        "depth_crash_percentile"
    )
    depth_crash_confirmations = LaunchConfiguration(
        "depth_crash_confirmations"
    )

    agent = ExecuteProcess(
        cmd=[
            "micro-xrce-dds-agent",
            "udp4",
            "-p",
            "8888",
        ],
        name="micro_xrce_dds_agent",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_agent),
    )

    px4 = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            [
                "cd ", os.path.join(home, "uav"), " && ",
                "source config/uav_env.sh && ",
                "source ~/miniconda3/etc/profile.d/conda.sh && ",
                'conda activate "$PX4_CONDA_ENV" && ',
                "export PYTHONNOUSERSITE=1 && ",
                "unset PYTHONPATH && ",
                "cd ", px4_dir, " && ",

                # Optional software rendering.
                'if [ "', software_render, '" = "true" ]; then ',
                "export LIBGL_ALWAYS_SOFTWARE=1; ",
                "export GALLIUM_DRIVER=llvmpipe; ",
                "export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe; ",
                "fi; ",

                # GUI by default. Only export HEADLESS=1 when requested.
                'if [ "', headless, '" = "true" ] || [ "', headless,
                '" = "1" ]; then ',
                "export HEADLESS=1; ",
                "else ",
                "unset HEADLESS; ",
                "fi; ",

                "PX4_GZ_WORLD=", world, " make px4_sitl ", model,
            ],
        ],
        name="px4_gazebo",
        output="screen",
        emulate_tty=True,
    )

    gcs = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            [
                "cd ", os.path.join(home, "uav"), " && ",
                "source config/uav_env.sh && ",
                "source ~/miniconda3/etc/profile.d/conda.sh && ",
                'conda activate "$PX4_CONDA_ENV" && ',
                "export PYTHONNOUSERSITE=1 && ",
                "unset PYTHONPATH && ",
                "mavproxy.py ",
                "--master=udpin:0.0.0.0:14550 ",
                "--aircraft uav_headless",
            ],
        ],
        name="mavproxy_gcs",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_gcs),
    )

    depth_bridge = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            [
                "ros2 run ros_gz_bridge parameter_bridge ",
                depth_gz_topic,
                "@sensor_msgs/msg/Image@gz.msgs.Image ",
                "--ros-args -r ",
                depth_gz_topic,
                ":=",
                depth_ros_topic,
            ],
        ],
        name="depth_bridge",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_bridge),
    )

    contact_bridge = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            [
                "ros2 run ros_gz_bridge parameter_bridge ",
                contact_gz_topic,
                "@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts ",
                "--ros-args -r ",
                contact_gz_topic,
                ":=",
                contact_ros_topic,
            ],
        ],
        name="contact_bridge",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_contact_bridge),
    )

    adapter = Node(
        package="uav_backend_gazebo_px4",
        executable="px4_backend_adapter",
        name="px4_backend_adapter",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "contact_topic": contact_ros_topic,
            "contact_force_topic": contact_force_topic,
            "contact_depth_topic": contact_depth_topic,
            "allowed_contact_names": allowed_contact_names,
            "max_contact_force_n": max_contact_force_n,
            "max_contact_depth_m": max_contact_depth_m,
            "depth_topic": depth_ros_topic,
            "crash_topic": crash_topic,
            "crash_reason_topic": crash_reason_topic,
            "gazebo_world": world,
            "gazebo_model_name": gazebo_model_name,
            "reset_x": reset_x,
            "reset_y": reset_y,
            "reset_z": reset_z,
            "reset_roll": reset_roll,
            "reset_pitch": reset_pitch,
            "reset_yaw": reset_yaw,
            "reset_pause": reset_pause,
            "depth_min_airborne_altitude": depth_min_airborne_altitude,
            "depth_min_valid_m": depth_min_valid_m,
            "depth_crash_percentile": depth_crash_percentile,
            "depth_crash_confirmations": depth_crash_confirmations,
        }],
        condition=IfCondition(start_adapter),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "px4_dir",
            default_value=os.path.join(home, "uav/external/PX4-Autopilot"),
        ),
        DeclareLaunchArgument(
            "model",
            default_value="gz_x500_depth",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="default",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo without GUI. Default false.",
        ),
        DeclareLaunchArgument(
            "software_render",
            default_value="false",
            description="Use Mesa llvmpipe software rendering.",
        ),
        DeclareLaunchArgument(
            "start_agent",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "start_gcs",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "start_bridge",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "start_contact_bridge",
            default_value="false",
            description="Bridge a Gazebo Contacts topic into /uav/crash.",
        ),
        DeclareLaunchArgument(
            "start_adapter",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "depth_gz_topic",
            default_value="/depth_camera",
        ),
        DeclareLaunchArgument(
            "depth_ros_topic",
            default_value="/uav/camera/depth/image",
        ),
        DeclareLaunchArgument(
            "contact_gz_topic",
            default_value="/contact",
            description=(
                "Gazebo Contacts topic to bridge when "
                "start_contact_bridge is true."
            ),
        ),
        DeclareLaunchArgument(
            "contact_ros_topic",
            default_value="/uav/gazebo/contacts",
        ),
        DeclareLaunchArgument(
            "contact_force_topic",
            default_value="/uav/contact_force_n",
        ),
        DeclareLaunchArgument(
            "contact_depth_topic",
            default_value="/uav/contact_depth_m",
        ),
        DeclareLaunchArgument(
            "allowed_contact_names",
            default_value="ground,ground_plane,landing_pad,landingpad,pad,floor",
            description=(
                "Comma-separated contact-name substrings that are allowed "
                "unless force/depth limits are exceeded."
            ),
        ),
        DeclareLaunchArgument(
            "max_contact_force_n",
            default_value="200.0",
            description=(
                "Allowed contacts above this force become crashes. "
                "Use <=0 to disable the force limit."
            ),
        ),
        DeclareLaunchArgument(
            "max_contact_depth_m",
            default_value="0.05",
            description=(
                "Allowed contacts above this penetration depth become "
                "crashes. Use <=0 to disable the depth limit."
            ),
        ),
        DeclareLaunchArgument(
            "crash_topic",
            default_value="/uav/crash",
        ),
        DeclareLaunchArgument(
            "crash_reason_topic",
            default_value="/uav/crash_reason",
        ),
        DeclareLaunchArgument(
            "gazebo_model_name",
            default_value="x500_depth_0",
            description="Gazebo model entity name to move on /uav/reset_sim.",
        ),
        DeclareLaunchArgument(
            "reset_x",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "reset_y",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "reset_z",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "reset_roll",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "reset_pitch",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "reset_yaw",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "reset_pause",
            default_value="false",
            description=(
                "Pause Gazebo during /uav/reset_sim. Default false "
                "because some PX4/Gazebo worlds time out on "
                "/world/*/control."
            ),
        ),
        DeclareLaunchArgument(
            "depth_min_airborne_altitude",
            default_value="0.75",
            description=(
                "Suppress depth-fallback crash detection below this ENU "
                "altitude so landing/ground does not count as a crash."
            ),
        ),
        DeclareLaunchArgument(
            "depth_min_valid_m",
            default_value="0.05",
            description="Ignore depth values below this as invalid dropouts.",
        ),
        DeclareLaunchArgument(
            "depth_crash_percentile",
            default_value="0.5",
            description=(
                "Use this valid-depth percentile for depth fallback instead "
                "of the single minimum pixel."
            ),
        ),
        DeclareLaunchArgument(
            "depth_crash_confirmations",
            default_value="3",
            description=(
                "Require this many consecutive close depth frames before "
                "publishing a depth-fallback crash."
            ),
        ),

        agent,
        TimerAction(period=1.0, actions=[px4]),
        TimerAction(period=8.0, actions=[gcs]),
        TimerAction(period=12.0, actions=[depth_bridge]),
        TimerAction(period=12.5, actions=[contact_bridge]),
        TimerAction(period=14.0, actions=[adapter]),
    ])
