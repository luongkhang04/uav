# Installation Guide

This guide installs and builds the UAV ROS 2 workspace.

The expected setup is:

```text
Ubuntu
ROS 2 Jazzy
Gazebo Harmonic / Gazebo Sim 8
PX4-Autopilot
px4_msgs
Micro XRCE-DDS Agent
MAVProxy
ros_gz_bridge
```

## 1. Clone the Repository

```bash
cd ~
git clone --recursive https://github.com/luongkhang04/uav.git
cd ~/uav
```

If the repository was cloned without submodules:

```bash
git submodule update --init --recursive
```

Expected important paths:

```text
~/uav/external/PX4-Autopilot
~/uav/src/px4_msgs
~/uav/src/uav_control
~/uav/src/uav_backend_gazebo_px4
~/uav/src/uav_bringup
```

## 2. Prevent Colcon from Building PX4

PX4 must be built with `make`, not with `colcon`.

```bash
cd ~/uav
touch external/COLCON_IGNORE
```

## 3. Install ROS 2 Dependencies

```bash
sudo apt update

sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  tmux \
  mesa-utils \
  libgl1-mesa-dri \
  ros-$ROS_DISTRO-ros-gz \
  ros-$ROS_DISTRO-ros-gz-bridge \
  ros-$ROS_DISTRO-ros-gz-image
```

Initialize `rosdep` if needed:

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

Install package dependencies:

```bash
cd ~/uav
source /opt/ros/$ROS_DISTRO/setup.bash

rosdep install --from-paths src --ignore-src -r -y
```

## 4. Install Micro XRCE-DDS Agent

If using the snap package:

```bash
sudo snap install micro-xrce-dds-agent --edge
```

Check:

```bash
micro-xrce-dds-agent --help
```

The command used by this repository is:

```bash
micro-xrce-dds-agent udp4 -p 8888
```

## 5. Prepare PX4 Python Environment

This repository assumes a PX4 virtual environment exists at:

```text
~/px4-venv
```

Activate it:

```bash
source ~/px4-venv/bin/activate
```

Install MAVProxy and required Python packages:

```bash
python -m pip install --upgrade pip setuptools wheel
python -m pip install MAVProxy pymavlink future
```

Check:

```bash
which mavproxy.py
mavproxy.py --help | head
```

If `future` is missing:

```bash
python -m pip install future
```

## 6. GPU Permission for Gazebo Depth Camera

The `gz_x500_depth` model uses camera/depth rendering. The user must have permission to access GPU render devices.

Check:

```bash
ls -l /dev/dri
groups
```

If the user is not in `render` and `video`, run:

```bash
sudo usermod -aG render,video $USER
sudo reboot
```

After reboot:

```bash
groups
```

Expected groups include:

```text
render video
```

If Gazebo still fails with OpenGL/EGL errors, try software rendering:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
```

## 7. Keyboard Permission for Hold/Release Control

The keyboard command node reads Linux input events so it can detect key release
and multiple held keys. Add your user to the `input` group:

```bash
sudo usermod -aG input $USER
sudo reboot
```

After reboot, check:

```bash
groups
```

Expected groups include:

```text
input
```

This full hold/release mode is for a local terminal on the machine running ROS.
When using SSH, the keyboard node falls back to terminal input: lifecycle keys
still work, but true simultaneous held movement keys such as `w+e` are not
reliable because SSH terminals do not send physical key release state.

## 8. Build the ROS 2 Workspace

Build outside `px4-venv`:

```bash
cd ~/uav
deactivate 2>/dev/null || true

source /opt/ros/$ROS_DISTRO/setup.bash

colcon build --symlink-install
source install/setup.bash
```

## 9. Verify ROS Packages

```bash
ros2 pkg list | grep uav
```

Expected packages include:

```text
uav_control
uav_backend_gazebo_px4
uav_bringup
```

Check executables:

```bash
ros2 run uav_control keyboard_cmd_vel --help
ros2 run uav_backend_gazebo_px4 px4_offboard_adapter
ros2 run uav_backend_gazebo_px4 state_monitor
```

Use `Ctrl+C` to stop test nodes.

## 10. Common Build Issues

### PX4 is being built by colcon

Symptom:

```text
Starting >>> px4
ModuleNotFoundError: No module named 'menuconfig'
```

Fix:

```bash
cd ~/uav
touch external/COLCON_IGNORE
rm -rf build install log

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### `MicroXRCEAgent` command not found

Use:

```bash
micro-xrce-dds-agent udp4 -p 8888
```

instead of:

```bash
MicroXRCEAgent udp4 -p 8888
```

### `mavproxy.py` fails with `No module named future`

Fix:

```bash
source ~/px4-venv/bin/activate
python -m pip install future
```

### Gazebo depth model fails with `/dev/dri` permission denied

Fix:

```bash
sudo usermod -aG render,video $USER
sudo reboot
```

### Gazebo depth model fails with OpenGL error

Try:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
```

Then rerun PX4/Gazebo.
