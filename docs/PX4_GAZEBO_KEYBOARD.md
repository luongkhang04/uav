# PX4 + Gazebo + Keyboard Control

This guide explains how to run the PX4/Gazebo backend and control the simulated UAV using the keyboard.

The PX4 backend includes:

```text
PX4 + Gazebo
Micro XRCE-DDS Agent
MAVProxy headless GCS
ros_gz_bridge depth camera bridge
px4_offboard_adapter
```

Keyboard control runs separately because it needs an interactive terminal.

## 1. Clean Old Processes

Before starting a fresh simulation:

```bash
pkill -9 -f "px4" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "mavproxy.py" 2>/dev/null || true
pkill -9 -f "micro-xrce-dds-agent" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
```

## 2. One-Command PX4 Backend Launch

If the launch file is installed:

```bash
cd ~/uav
deactivate 2>/dev/null || true
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 launch uav_bringup px4_gazebo_depth.launch.py
```

This starts:

```text
Micro XRCE-DDS Agent
PX4 + Gazebo x500_depth
MAVProxy GCS
ros_gz_bridge /depth_camera
px4_offboard_adapter
```

If the depth model fails because of OpenGL/EGL, use software rendering:

```bash
ros2 launch uav_bringup px4_gazebo_depth.launch.py software_render:=true
```

If depth is not needed:

```bash
ros2 launch uav_bringup px4_gazebo_depth.launch.py model:=gz_x500 start_bridge:=false
```

## 3. Manual Backend Fallback

For debugging or for cases where the launch file is not ready, start each PX4
backend process manually from:

```text
src/uav_backend_gazebo_px4/README.md
```

## 4. Keyboard Control

Run keyboard control in an interactive terminal:

```bash
cd ~/uav
deactivate 2>/dev/null || true
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 run uav_control keyboard_cmd_vel
```

Keyboard mapping:

```text
1           Offboard + Arm
2           Land
3           Disarm
4           Print help
x           Exit

hold w/s    Forward / Backward
hold a/d    Left / Right
hold r/f    Up / Down
hold q/e    Yaw left / Yaw right

Up arrow    Increase speed
Down arrow  Decrease speed
```

Keyboard input modes:

```text
Local terminal on ROS machine:
  Uses /dev/input/event*. Supports true key press/release and multiple held
  movement keys at the same time, such as w+e for forward plus yaw right.

SSH terminal:
  Uses terminal fallback. Lifecycle keys work, and movement works through key
  repeat, but SSH cannot reliably report multiple simultaneous held keys or true
  key-release events. Release is approximated by a short timeout after key
  repeat stops. Use local mode for proper multi-key flying.
```

## 5. State Monitor

Run:

```bash
cd ~/uav
deactivate 2>/dev/null || true
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 run uav_state state_monitor
```

Expected output:

```text
===== UAV STATE MONITOR =====
rates: odom=100 Hz | imu=250 Hz | depth=... Hz
PX4 NED pos: ...
IMU gyro(rad/s): ...
Depth image: WIDTHxHEIGHT, encoding=...
```

If using `gz_x500` instead of `gz_x500_depth`, depth will stay:

```text
depth=0 Hz
```

This is expected because `gz_x500` has no depth camera.