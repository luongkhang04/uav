# uav_backend_gazebo_px4

This package contains the PX4/Gazebo backend adapter for the UAV workspace. It
bridges high-level ROS 2 body-frame velocity commands into PX4 Offboard
setpoints and provides service hooks for arming, landing, and disarming.

## Manual Fallback: Start Each PX4 Backend Process

Use this section if the launch file is not ready or if debugging is needed.

### Terminal 1: Micro XRCE-DDS Agent

```bash
micro-xrce-dds-agent udp4 -p 8888
```

### Terminal 2: PX4 + Gazebo

Depth model:

```bash
cd ~/uav
source config/uav_env.sh
source ~/miniconda3/etc/profile.d/conda.sh
conda activate "$PX4_CONDA_ENV"
export PYTHONNOUSERSITE=1
unset PYTHONPATH
cd ~/uav/external/PX4-Autopilot

PX4_GZ_WORLD=default make px4_sitl gz_x500_depth
```

No-depth model:

```bash
cd ~/uav
source config/uav_env.sh
source ~/miniconda3/etc/profile.d/conda.sh
conda activate "$PX4_CONDA_ENV"
export PYTHONNOUSERSITE=1
unset PYTHONPATH
cd ~/uav/external/PX4-Autopilot

PX4_GZ_WORLD=default make px4_sitl gz_x500
```

If depth rendering fails, try:

```bash
cd ~/uav
source config/uav_env.sh
source ~/miniconda3/etc/profile.d/conda.sh
conda activate "$PX4_CONDA_ENV"
export PYTHONNOUSERSITE=1
unset PYTHONPATH
cd ~/uav/external/PX4-Autopilot

export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe

PX4_GZ_WORLD=default HEADLESS=1 make px4_sitl gz_x500_depth
```

### Terminal 3: MAVProxy Headless GCS

```bash
cd ~/uav
source config/uav_env.sh
source ~/miniconda3/etc/profile.d/conda.sh
conda activate "$PX4_CONDA_ENV"
export PYTHONNOUSERSITE=1
unset PYTHONPATH

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
conda deactivate 2>/dev/null || true
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
conda deactivate 2>/dev/null || true
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 run uav_backend_gazebo_px4 px4_backend_adapter
```
