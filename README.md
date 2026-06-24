# UAV: ROS 2 + PX4 + Gazebo UAV Control Stack

This repository provides a modular ROS 2 workspace for UAV simulation, control, monitoring, and future learning/XAI extensions. The current implementation focuses on PX4 + Gazebo SITL with ROS 2 Offboard velocity control, keyboard teleoperation, state monitoring, and depth camera bridging.

Repository:

```bash
git clone --recursive https://github.com/luongkhang04/uav.git
```

## Current Goal

The short-term goal is to build a clean UAV software stack that can:

* run PX4 SITL with Gazebo;
* expose PX4 state to ROS 2;
* control the UAV through PX4 Offboard mode;
* support keyboard control for manual testing;
* monitor odometry, IMU, and depth camera streams;
* provide a stable interface for future RL and XAI modules.

The long-term goal is to support multiple backends:

```text
Gazebo + PX4 SITL
AirSim
Real PX4 UAV
```

while keeping the high-level controller, RL policy, and XAI modules independent of the backend.

## Architecture

The intended architecture is:

```text
keyboard / RL policy / planner
        |
        v
/uav/cmd_vel_body
        |
        v
px4_offboard_adapter
        |
        v
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
        |
        v
PX4
        |
        v
Gazebo UAV or real UAV
```

State and sensor flow:

```text
PX4 uXRCE-DDS topics
        |
        v
/fmu/out/vehicle_odometry
/fmu/out/sensor_combined
/fmu/out/vehicle_status_v4

Gazebo sensor topics
        |
        v
ros_gz_bridge
        |
        v
/uav/camera/depth/image
/uav/camera/depth/points
/uav/camera/rgb/image
```

## Packages

Current package layout:

```text
uav/
├── external/
│   └── PX4-Autopilot/              # PX4 submodule, built separately
├── src/
│   ├── px4_msgs/                   # PX4 ROS 2 message definitions
│   ├── uav_control/                # Keyboard/manual control nodes
│   ├── uav_backend_gazebo_px4/     # PX4/Gazebo backend adapter and monitor
│   └── uav_bringup/                # Launch files
└── docs/
    ├── INSTALL.md
    └── PX4_GAZEBO_KEYBOARD.md
```

## Implemented

The current implementation includes:

* PX4-Autopilot as an external submodule.
* `px4_msgs` as a ROS 2 package.
* `uav_control/keyboard_cmd_vel.py`

  * publishes `/uav/cmd_vel_body`;
  * supports manual velocity control;
  * can call arm/offboard, land, and disarm services.
* `uav_backend_gazebo_px4/px4_offboard_adapter.py`

  * subscribes `/uav/cmd_vel_body`;
  * publishes PX4 Offboard setpoints;
  * exposes `/uav/offboard_arm`, `/uav/land`, and `/uav/disarm`;
  * converts ROS body-frame velocity commands into PX4 NED velocity setpoints.
* `uav_backend_gazebo_px4/state_monitor.py`

  * monitors PX4 odometry;
  * monitors IMU;
  * monitors depth image if the Gazebo depth bridge is running.
* PX4 + Gazebo SITL workflow.
* Headless GCS through MAVProxy.
* Depth camera bridge through `ros_gz_bridge`.

## Main ROS Topics

Command topic:

```text
/uav/cmd_vel_body
```

Message type:

```text
geometry_msgs/msg/TwistStamped
```

Control convention:

```text
linear.x   forward
linear.y   left
linear.z   up
angular.z  yaw-left rate
```

PX4 input topics:

```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
```

PX4 output topics:

```text
/fmu/out/vehicle_odometry
/fmu/out/sensor_combined
/fmu/out/vehicle_status_v4
/fmu/out/vehicle_command_ack_v1
```

Gazebo/ROS camera topics:

```text
/uav/camera/depth/image
/uav/camera/depth/points
/uav/camera/rgb/image
```

## Quick Start

Install dependencies and build the workspace:

```text
docs/INSTALL.md
```

Run PX4 + Gazebo + keyboard control:

```text
docs/PX4_GAZEBO_KEYBOARD.md
```

Typical run flow:

```bash
ros2 launch uav_bringup px4_gazebo_depth.launch.py
```

Then in another terminal:

```bash
ros2 run uav_control keyboard_cmd_vel
```

Keyboard controls:

```text
o      Offboard + Arm
r      Up / takeoff velocity
f      Down
w/s    Forward / backward
a/d    Left / right
j/l    Yaw left / yaw right
space  Stop
p      Land
k      Disarm
x      Exit keyboard
```

## Current Status

Working:

* PX4 SITL starts with Gazebo.
* Micro XRCE-DDS Agent connects to PX4.
* PX4 publishes odometry and IMU to ROS 2.
* Offboard adapter publishes velocity setpoints to PX4.
* Keyboard control can command body-frame velocity.
* UAV can arm and take off in simulation.
* State monitor receives odometry and IMU.
* `x500_depth` model exposes Gazebo camera/depth topics.

In progress:

* Stable one-command launch for the whole PX4 backend.
* Robust depth bridge configuration.
* Cleaner run mode with fewer manual terminals.
* RL environment wrapper.
* XAI observation/action logging.

## Roadmap

Near-term:

* Add and validate launch files for:

  * PX4 + Gazebo;
  * Micro XRCE-DDS Agent;
  * MAVProxy GCS;
  * `ros_gz_bridge`;
  * PX4 Offboard adapter.
* Add a minimal keyboard workflow with arm, land, and disarm keys.
* Add a clean monitor command for odometry, IMU, depth image, and PX4 status.
* Add config files for topic names, speed limits, and backend selection.

Mid-term:

* Add AirSim backend adapter.
* Add real PX4 backend adapter.
* Add unified backend interface:

```text
/uav/odom
/uav/imu
/uav/status
/uav/camera/depth/image
/uav/cmd_vel_body
```

* Add RL training environment using the same command/state interface.
* Add evaluation scripts for trained policies.

Long-term:

* Add XAI modules for explaining UAV decisions.
* Add policy action attribution from depth/state inputs.
* Add safety monitor and failsafe layer.
* Add experiment logging and replay.
* Add support for real UAV deployment with strict safety checks.

## Safety Note

This repository is currently intended for simulation. Do not run the same Offboard control stack on a real UAV without proper safety checks, manual override, geofence, kill switch, tested failsafe behavior, and supervision.
