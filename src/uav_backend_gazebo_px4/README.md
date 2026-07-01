# uav_backend_gazebo_px4

This package contains the PX4/Gazebo backend adapter for the UAV workspace. It
bridges high-level ROS 2 body-frame velocity commands into PX4 Offboard
setpoints, provides service hooks for arming, landing, and disarming, and
publishes backend-independent crash state on `/uav/crash`.

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

Optional Gazebo contact bridge, if your model has a contact sensor:

```bash
gz topic -l | grep -Ei "contact|collision"

ros2 run ros_gz_bridge parameter_bridge \
  /CONTACT_TOPIC@ros_gz_interfaces/msg/Contacts@gz.msgs.Contacts \
  --ros-args -r /CONTACT_TOPIC:=/uav/gazebo/contacts
```

The adapter also listens to PX4 `/fmu/out/failure_detector_status`,
`/fmu/out/vehicle_status`, and `/fmu/out/vehicle_land_detected`. Gazebo contacts
are classified first: names matching `allowed_contact_names` are allowed unless
`max_contact_force_n` or `max_contact_depth_m` is exceeded; unallowed contact
names are reported as crashes. The adapter publishes contact metrics on
`/uav/contact_force_n` and `/uav/contact_depth_m` for the state monitors. When
contact/PX4 state stays quiet, it falls back to `/uav/camera/depth/image` and
reports `depth_fallback` if robust near depth is below the crash distance for
`depth_crash_confirmations` consecutive frames. Depth values below
`depth_min_valid_m` are ignored as dropouts, and `depth_crash_percentile` uses a
small valid-depth percentile instead of one unstable minimum pixel. The depth
fallback is ignored below `depth_min_airborne_altitude` so normal ground landing
does not count as a crash; PX4 land-detected is only used as a backup when
odometry altitude is not available. In the monitor, `ok depth_near_m=...` means
depth is being received but has not crossed the confirmed crash threshold;
`ok contact_allowed:...` means a contact is present but within the allowed
name/force/depth limits.

### Terminal 5: PX4 Offboard Adapter

```bash
cd ~/uav
conda deactivate 2>/dev/null || true
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 run uav_backend_gazebo_px4 px4_backend_adapter
```
