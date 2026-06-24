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

## 3. Manual Fallback: Start Each PX4 Backend Process

Use this section if the launch file is not ready or if debugging is needed.

### Terminal 1: Micro XRCE-DDS Agent

```bash
micro-xrce-dds-agent udp4 -p 8888
```

### Terminal 2: PX4 + Gazebo

Depth model:

```bash
source ~/px4-venv/bin/activate
cd ~/uav/external/PX4-Autopilot

PX4_GZ_WORLD=default HEADLESS=1 make px4_sitl gz_x500_depth
```

No-depth model:

```bash
source ~/px4-venv/bin/activate
cd ~/uav/external/PX4-Autopilot

PX4_GZ_WORLD=default HEADLESS=1 make px4_sitl gz_x500
```

If depth rendering fails, try:

```bash
source ~/px4-venv/bin/activate
cd ~/uav/external/PX4-Autopilot

export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe

PX4_GZ_WORLD=default HEADLESS=1 make px4_sitl gz_x500_depth
```

### Terminal 3: MAVProxy Headless GCS

```bash
source ~/px4-venv/bin/activate

mavproxy.py \
  --master=udpin:0.0.0.0:14550 \
  --aircraft uav_headless
```

### Terminal 4: Depth Bridge

Check Gazebo depth topics:

```bash
gz topic -l | grep -Ei "depth|camera|image"
```

For the current `x500_depth` model, the depth image topic is usually:

```text
/depth_camera
```

Bridge it to ROS 2:

```bash
cd ~/uav
deactivate 2>/dev/null || true
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /depth_camera:=/uav/camera/depth/image
```

Optional point cloud bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  --ros-args -r /depth_camera/points:=/uav/camera/depth/points
```

Optional camera info bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  --ros-args -r /camera_info:=/uav/camera/depth/camera_info
```

### Terminal 5: PX4 Offboard Adapter

```bash
cd ~/uav
deactivate 2>/dev/null || true
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 run uav_backend_gazebo_px4 px4_offboard_adapter
```

## 4. Keyboard Control

Keyboard hold/release control reads Linux keyboard event devices so it can detect
when keys are released and can combine multiple held keys. Make sure your user
can read `/dev/input/event*`:

```bash
groups
sudo usermod -aG input $USER
sudo reboot
```

After reboot, `groups` should include `input`.

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

hold w/s    Forward / Backward
hold a/d    Left / Right
hold r/f    Up / Down
hold q/e    Yaw left / Yaw right

Up arrow    Increase speed
Down arrow  Decrease speed
4           Print help
x           Exit
```

Recommended basic test:

```text
1. Press 1 to enter Offboard and arm.
2. Hold r for 1-2 seconds to climb, then release r to stop climbing.
3. Hold w/a/s/d to test horizontal motion, then release to stop that axis.
4. Hold q/e to test yaw.
5. Press 2 to land.
6. Press 3 to disarm after landing.
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

## 6. Check PX4 Status

```bash
ros2 topic echo /fmu/out/vehicle_status_v4
```

Important fields:

```text
arming_state: 2
nav_state: 14
accepts_offboard_setpoints: true
pre_flight_checks_pass: true
```

Meaning:

```text
arming_state: 2               armed
nav_state: 14                 Offboard
accepts_offboard_setpoints    PX4 accepts Offboard setpoints
pre_flight_checks_pass        PX4 is healthy enough to arm
```

## 7. Check Command Topic

```bash
ros2 topic echo /uav/cmd_vel_body
```

Expected convention:

```text
linear.x   forward
linear.y   left
linear.z   up
angular.z  yaw-left rate
```

## 8. Check Depth Topic

```bash
ros2 topic echo /uav/camera/depth/image --once
```

If no message appears:

1. Check that `gz_x500_depth` is running.
2. Check Gazebo topic:

```bash
gz topic -l | grep -Ei "depth|camera|image"
```

3. Check bridge is running:

```bash
ros2 topic list | grep depth
```

4. Restart the bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  --ros-args -r /depth_camera:=/uav/camera/depth/image
```

## 9. Debugging Notes

### PX4 does not arm

Check PX4 shell:

```sh
commander check
```

Common causes:

```text
No GCS connection
Power check failed
Attitude failure
Offboard setpoints not being published
```

### GCS connection missing

Run MAVProxy:

```bash
source ~/px4-venv/bin/activate
mavproxy.py --master=udpin:0.0.0.0:14550 --aircraft uav_headless
```

### Offboard setpoints not accepted

Make sure `px4_offboard_adapter` is running and publishing at about 20 Hz:

```bash
ros2 topic hz /fmu/in/offboard_control_mode
ros2 topic hz /fmu/in/trajectory_setpoint
```

### Odometry QoS warning

If you see:

```text
offering incompatible QoS
Last incompatible policy: RELIABILITY
```

make sure subscribers to PX4 output topics use `qos_profile_sensor_data`.

### Depth model crashes

Check GPU permission:

```bash
groups
ls -l /dev/dri
```

Fix:

```bash
sudo usermod -aG render,video $USER
sudo reboot
```

If needed, use software rendering:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
```

## 10. Recommended Development Flow

For control development:

```bash
ros2 launch uav_bringup px4_gazebo_depth.launch.py model:=gz_x500 start_bridge:=false
ros2 run uav_control keyboard_cmd_vel
ros2 run uav_state state_monitor
```

For depth/perception development:

```bash
ros2 launch uav_bringup px4_gazebo_depth.launch.py
ros2 run uav_state state_monitor
```

For future RL:

```text
RL environment calls /uav/offboard_arm at episode start.
RL policy publishes /uav/cmd_vel_body during the episode.
Environment calls /uav/land or resets simulation at episode end.
```

For future XAI:

```text
XAI observes state topics, depth image, and /uav/cmd_vel_body.
XAI publishes explanations separately without directly controlling PX4.
```
