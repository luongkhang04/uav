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

## 5. Prepare PX4 Conda Environment

The PX4 conda env name is configured in `config/uav_env.sh` as `PX4_CONDA_ENV` (default `px4`).

Create and activate it:

```bash
source config/uav_env.sh
source ~/miniconda3/etc/profile.d/conda.sh
conda create -n "$PX4_CONDA_ENV" python=3.10 -y
conda activate "$PX4_CONDA_ENV"
export PYTHONNOUSERSITE=1
unset PYTHONPATH
```

Install MAVProxy and required PX4 Python packages:

```bash
python -m pip install --upgrade pip wheel "setuptools<82"
python -m pip install MAVProxy pymavlink future
python -m pip install "empy>=3.3,<4" kconfiglib Jinja2 jsonschema lxml numpy packaging pyros-genmsg pyserial PyYAML toml catkin-pkg
```

Check:

```bash
which mavproxy.py
mavproxy.py --help | head
```


## 6. Prepare XAI SAC Conda Environment

The `uav_train` and `uav_evaluate` wrappers use the conda env name configured
in `config/uav_env.sh` as `UAV_CONDA_ENV` (default `uav`). They run Python with
`conda run -n "$UAV_CONDA_ENV"`, so clone-specific env names only need to be
changed in that config file.

Create and activate it:

```bash
source config/uav_env.sh
source ~/miniconda3/etc/profile.d/conda.sh
conda create -n "$UAV_CONDA_ENV" python=3.12 -y
conda activate "$UAV_CONDA_ENV"
export PYTHONNOUSERSITE=1
unset PYTHONPATH
```

Install the training/evaluation Python packages:

```bash
python -m pip install --upgrade pip wheel setuptools
python -m pip install stable-baselines3 gymnasium torch numpy PyYAML
```

For exact reproduction of the tested local environment, install the lock file
instead of the second command above:

```bash
python -m pip install -r requirements-uav-tested.txt
```

Check:

```bash
python -c "import stable_baselines3, gymnasium, torch, numpy, yaml; print(\"uav env ok\")"
```

ROS Python packages such as `rclpy` and message packages should still come from
ROS 2 and this workspace after `source /opt/ros/$ROS_DISTRO/setup.bash` and
`source install/setup.bash`; do not install ROS packages into this conda env.

## 7. Device Permissions for Gazebo and Keyboard Control

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

## 8. Build the ROS 2 Workspace

Build outside the PX4 and XAI SAC conda envs:

```bash
cd ~/uav
conda deactivate 2>/dev/null || true

source /opt/ros/$ROS_DISTRO/setup.bash

colcon build --symlink-install
source install/setup.bash
```