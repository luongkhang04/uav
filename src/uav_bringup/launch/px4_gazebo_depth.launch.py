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
    start_adapter = LaunchConfiguration("start_adapter")

    depth_gz_topic = LaunchConfiguration("depth_gz_topic")
    depth_ros_topic = LaunchConfiguration("depth_ros_topic")

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
                'if [ "', headless, '" = "true" ] || [ "', headless, '" = "1" ]; then ',
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

    adapter = Node(
        package="uav_backend_gazebo_px4",
        executable="px4_backend_adapter",
        name="px4_backend_adapter",
        output="screen",
        emulate_tty=True,
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

        agent,
        TimerAction(period=1.0, actions=[px4]),
        TimerAction(period=8.0, actions=[gcs]),
        TimerAction(period=12.0, actions=[depth_bridge]),
        TimerAction(period=14.0, actions=[adapter]),
    ])
