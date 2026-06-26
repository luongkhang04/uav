# UAV: ROS 2 + PX4 + Gazebo UAV Control Stack

This repository provides a modular ROS 2 workspace for UAV simulation, control,
state monitoring, and future learning/XAI extensions. The current implementation
uses PX4 + Gazebo SITL as the backend and exposes a backend-independent ROS 2
interface for control and state.

## Current Goal

The short-term goal is to keep the UAV stack cleanly separated into:

* control clients that publish backend-independent velocity commands;
* backend adapters that translate between `/uav/*` topics and a specific
  simulator or vehicle backend;
* state tools that consume normalized odometry, IMU, and camera topics;
* launch and documentation for repeatable PX4/Gazebo development.

The long-term goal is to support multiple backends:

```text
Gazebo + PX4 SITL
AirSim
Real PX4 UAV
```

while keeping keyboard control, RL policies, planners, state monitors, and XAI
modules independent of the selected backend.

## Architecture

The active backend boundary is `px4_backend_adapter.py`. It adapts both control
and state between the backend-independent `/uav/*` interface and PX4 uXRCE-DDS
topics.

Control flow:

```text
keyboard / RL policy / planner
        |
        v
/uav/cmd_vel_body                 geometry_msgs/msg/TwistStamped
        |                         frame: base_link, ROS body FLU
        v
px4_backend_adapter
        |
        v
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
        |
        v
PX4 Offboard control
        |
        v
Gazebo x500 / x500_depth
```

State flow:

```text
PX4 uXRCE-DDS output topics
        |
        v
/fmu/out/vehicle_odometry
/fmu/out/sensor_combined
        |
        v
px4_backend_adapter
        |
        v
/uav/odom                        nav_msgs/msg/Odometry
/uav/imu                         sensor_msgs/msg/Imu
        |
        v
state_monitor / state_monitor_gui / RL / XAI
```

Camera flow:

```text
Gazebo depth camera topic
        |
        v
ros_gz_bridge
        |
        v
/uav/camera/depth/image          sensor_msgs/msg/Image
        |
        v
state_monitor / state_monitor_gui / perception / RL / XAI
```

Frame conventions:

```text
/uav/cmd_vel_body   ROS body FLU: x forward, y left, z up, yaw-left positive
/uav/odom pose      ROS ENU: x east, y north, z up
/uav/odom twist     ROS body FLU
/uav/imu            ROS body FLU-like angular velocity and acceleration
PX4 backend         PX4 NED / body FRD, hidden behind px4_backend_adapter
```

## Packages

Current package layout:

```text
uav/
├── external/
│   └── PX4-Autopilot/              # PX4 submodule, built with make
├── src/
│   ├── px4_msgs/                   # PX4 ROS 2 message definitions
│   ├── uav_control/                # Keyboard/manual control client
│   ├── uav_backend_gazebo_px4/     # PX4/Gazebo control + state adapter
│   ├── uav_state/                  # Backend-independent state monitors
│   ├── uav_train/                  # XAI SAC training env and trainer
│   ├── uav_evaluate/               # XAI SAC policy evaluation runner
│   └── uav_bringup/                # Launch files for PX4/Gazebo workflow
└── docs/
    ├── INSTALL.md
    ├── PX4_GAZEBO_KEYBOARD.md
    └── XAI_SAC_GAZEBO.md
```

Package responsibilities:

* `uav_control`
  * provides `keyboard_cmd_vel`;
  * publishes `/uav/cmd_vel_body`;
  * calls `/uav/offboard_arm`, `/uav/land`, and `/uav/disarm`.
* `uav_backend_gazebo_px4`
  * provides `px4_backend_adapter`;
  * converts `/uav/cmd_vel_body` from ROS body FLU to PX4 NED velocity
    setpoints;
  * publishes PX4 Offboard setpoint and vehicle command topics;
  * converts PX4 odometry and IMU data into normalized `/uav/odom` and
    `/uav/imu`;
  * keeps manual fallback backend startup instructions in its package README.
* `uav_state`
  * provides `state_monitor` for terminal status output;
  * provides `state_monitor_gui` for local visual monitoring;
  * subscribes only to backend-independent `/uav/*` state and camera topics.
* `uav_train`
  * provides `train_xai_sac`;
  * mirrors the `XAI_SAC_AirSim_UAV` SB3 SAC training loop;
  * builds the same 31-value observation and 3-value action interface from
    normalized ROS topics.
* `uav_evaluate`
  * provides `evaluate_xai_sac`;
  * loads a trained SAC `.zip` and publishes policy commands to
    `/uav/cmd_vel_body`.
* `uav_bringup`
  * provides `px4_gazebo_depth.launch.py`;
  * starts Micro XRCE-DDS Agent, PX4 + Gazebo, MAVProxy, the depth bridge, and
    `px4_backend_adapter`.
* `px4_msgs`
  * provides the PX4 message types used by the adapter.

## Implemented

The current implementation includes:

* PX4-Autopilot as an external submodule.
* `px4_msgs` as a ROS 2 package.
* Keyboard hold/release control through `uav_control/keyboard_cmd_vel.py`.
* A PX4/Gazebo backend adapter in
  `uav_backend_gazebo_px4/px4_backend_adapter.py`.
* Backend-independent state monitors in `uav_state`.
* One-command PX4/Gazebo launch through `uav_bringup`.
* Headless GCS through MAVProxy.
* Depth camera bridge through `ros_gz_bridge`.
* XAI SAC training and evaluation packages using the same SB3 model shape as
  `XAI_SAC_AirSim_UAV`.
* A copied evaluation checkpoint at `models/xai_sac/model_final.zip`.
* Manual backend fallback instructions in
  `src/uav_backend_gazebo_px4/README.md`.

`px4_backend_adapter.py` currently:

* subscribes to `/uav/cmd_vel_body`;
* streams `/fmu/in/offboard_control_mode` at 20 Hz;
* publishes `/fmu/in/trajectory_setpoint` velocity setpoints;
* publishes `/fmu/in/vehicle_command` for mode, arm, disarm, and land;
* exposes `/uav/offboard_arm`, `/uav/land`, and `/uav/disarm`;
* subscribes to `/fmu/out/vehicle_odometry` and `/fmu/out/sensor_combined`;
* publishes normalized `/uav/odom` and `/uav/imu`.

## ROS Interface

Backend-independent control topic:

```text
/uav/cmd_vel_body                 geometry_msgs/msg/TwistStamped
```

Control convention:

```text
linear.x   forward
linear.y   left
linear.z   up
angular.z  yaw-left rate
```

Backend-independent state topics:

```text
/uav/odom                         nav_msgs/msg/Odometry
/uav/imu                          sensor_msgs/msg/Imu
/uav/camera/depth/image           sensor_msgs/msg/Image
/uav/camera/depth/points          sensor_msgs/msg/PointCloud2, optional
/uav/camera/rgb/image             sensor_msgs/msg/Image, optional
```

Backend-independent services:

```text
/uav/offboard_arm                 std_srvs/srv/Trigger
/uav/land                         std_srvs/srv/Trigger
/uav/disarm                       std_srvs/srv/Trigger
```

PX4 backend topics used by `px4_backend_adapter`:

```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
/fmu/out/vehicle_odometry
/fmu/out/sensor_combined
```

Gazebo bridge topics:

```text
/depth_camera                     Gazebo image topic
/uav/camera/depth/image           ROS 2 bridged image topic
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

Run XAI SAC training or evaluation:

```text
docs/XAI_SAC_GAZEBO.md
```

Typical run flow:

```bash
cd ~/uav
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 launch uav_bringup px4_gazebo_depth.launch.py
```

Then in another interactive terminal:

```bash
cd ~/uav
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 run uav_control keyboard_cmd_vel
```

Optional state monitor:

```bash
ros2 run uav_state state_monitor_gui
```

Evaluate the copied XAI SAC checkpoint after PX4/Gazebo is running:

```bash
ros2 run uav_evaluate evaluate_xai_sac \
  --model ~/uav/models/xai_sac/model_final.zip
```

## Current Status

Working:

* PX4 SITL starts with Gazebo from the launch file.
* Micro XRCE-DDS Agent connects PX4 to ROS 2.
* MAVProxy provides a headless GCS connection.
* `px4_backend_adapter` handles both control and state adaptation.
* Keyboard control publishes backend-independent body-frame commands.
* UAV can enter Offboard mode, arm, move, land, and disarm in simulation.
* `/uav/odom` and `/uav/imu` expose normalized backend-independent state.
* `state_monitor` and `state_monitor_gui` consume normalized state topics.
* `gz_x500_depth` exposes a depth camera that can be bridged to ROS 2.
* `uav_train` and `uav_evaluate` expose the XAI SAC train/eval flow.
* `models/xai_sac/model_final.zip` contains the copied trained checkpoint.

In progress:

* Broader perception topic support beyond the current depth image bridge.
* Cleaner reset/episode handling for future RL workflows.
* Additional backend adapters for AirSim and real PX4 vehicles.
