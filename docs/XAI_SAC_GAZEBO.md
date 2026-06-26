# XAI SAC Training and Evaluation

The `uav_train` and `uav_evaluate` packages use the same SAC setup as
`XAI_SAC_AirSim_UAV`: SB3 `SAC` with `MlpPolicy`, network
`[64, 32, 16]`, `tanh` activation, a 31-value observation, and a 3-value
continuous action.

Observation layout:

```text
25 depth proximity values from a 5 x 5 depth grid
6 state values: d_xy, d_z, relative_yaw, v_xy, v_z, yaw_rate
```

Action layout:

```text
forward velocity, vertical velocity, yaw-left rate
```

The ROS/Gazebo adapter consumes only backend-independent topics:

```text
/uav/odom
/uav/imu
/uav/camera/depth/image
```

and publishes:

```text
/uav/cmd_vel_body
```

## Build

```bash
cd ~/uav
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-select uav_train uav_evaluate
source install/setup.bash
```

The `train_xai_sac` and `evaluate_xai_sac` executables are wrapper
scripts that read `config/uav_env.sh` and run `conda run -n ""`
before starting Python, so you do not need to activate the conda env before
`ros2 run`. Change `UAV_CONDA_ENV` in that config file if your clone uses a
different env name. That Python env must provide `stable-baselines3`,
`gymnasium`, `torch`, `numpy`, and `PyYAML`.

## Evaluate Copied Model

Start PX4/Gazebo with depth first:

```bash
ros2 launch uav_bringup px4_gazebo_depth.launch.py
```

Then run:

```bash
ros2 run uav_evaluate evaluate_xai_sac \
  --model ~/uav/models/xai_sac/model_final.zip \
  --episodes 10
```

## Train

```bash
ros2 run uav_train train_xai_sac \
  --config ~/uav/src/uav_train/config/xai_sac_gazebo.yaml
```

Training writes runs to `logs/xai_sac_gazebo` by default.
