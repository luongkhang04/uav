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

## 6. Device Permissions for Gazebo and Keyboard Control

Gazebo depth rendering needs access to GPU render devices, and the keyboard
command node needs access to Linux input events for key release detection and
multiple held keys.

Check current device permissions and groups:

```bash
ls -l /dev/dri
groups
```

If the user is not already in `render`, `video`, and `input`, add the missing
groups:

```bash
sudo usermod -aG render,video,input $USER
sudo reboot
```

After reboot, check again:

```bash
groups
```

Expected groups include:

```text
render video input
```

The full keyboard hold/release mode is for a local terminal on the machine
running ROS. When using SSH, the keyboard node falls back to terminal input:
lifecycle keys still work, but true simultaneous held movement keys such as
`w+e` are not reliable because SSH terminals do not send physical key release
state.

## 7. Build the ROS 2 Workspace

Build outside `px4-venv`:

```bash
cd ~/uav
deactivate 2>/dev/null || true

source /opt/ros/$ROS_DISTRO/setup.bash

colcon build --symlink-install
source install/setup.bash
```