# OpenVINS (ov_msckf) – Offline VIO on a ROS1 Bag (macOS/Linux, ROS Noetic)

This README walks you through running **OpenVINS (ov_msckf)** offline on a recorded **ROS1** bag that contains:
- a monocular camera stream (e.g. `/eo/color/image_color` or `/.../compressed`)
- an IMU stream (e.g. `/imu/data`)

We’ll build OpenVINS in a Catkin workspace, republish compressed images to raw (if needed), launch OpenVINS with a custom config, and play your bag. Works well inside Docker or a native Noetic install.

---

## 0) What you’ll need

- **ROS Noetic** environment (native or Docker).
- A rosbag with:
  - Camera topic (raw or compressed), e.g.  
    `/eo/color/image_color` or `/eo/color/image_color/compressed`
  - IMU topic, e.g. `/imu/data`
- (Recommended) Camera intrinsics and IMU parameters (Kalibr or vendor specs). You can start with guesses, but accurate calibration improves results.

---

## 1) Create and build the workspace

```bash
# ROS env
source /opt/ros/noetic/setup.bash

# Create workspace
mkdir -p ~/ws_openvins/src
cd ~/ws_openvins/src

# Clone OpenVINS
git clone https://github.com/rpng/open_vins.git

# Install dependencies
cd ~/ws_openvins
rosdep install --from-paths src --ignore-src -y

# Build
catkin_make -DCMAKE_BUILD_TYPE=Release

# Source the workspace (do this in every new shell before launching)
source ~/ws_openvins/devel/setup.bash

# Sanity check that ROS can find the package
rospack find ov_msckf
```

> If `rospack find ov_msckf` fails, re-check the build output and re-source `setup.bash`.

---

## 2) Prepare a minimal config (estimator_config.yaml)

Create your dataset config folder and a minimal config file:

```bash
mkdir -p ~/ws_openvins/src/open_vins/config/my_dataset
nano ~/ws_openvins/src/open_vins/config/my_dataset/estimator_config.yaml
```

Paste this template and **adjust values** (intrinsics, distortion, resolution):

```yaml
# === OpenVINS minimal mono-inertial config ===
# Topics (we'll republish /compressed -> /ov_cam0/image_raw below)
cam0_topic: /ov_cam0/image_raw
imu0_topic: /imu/data

# Camera model & intrinsics (replace with your calibration)
cam0_model: pinhole-equi
cam0_intrinsics: [460.0, 460.0, 320.0, 240.0]   # [fx, fy, cx, cy]
cam0_distortion: [0.0, 0.0, 0.0, 0.0]          # [k1, k2, r1, r2]
cam0_resolution: [640, 480]                    # width, height
cam0_timeshift: 0.0

# IMU noise params (start reasonable; refine later)
imu_noise_gyro: 0.001
imu_noise_accel: 0.01
imu_bias_gyro: 0.0001
imu_bias_accel: 0.001

# Mono vs stereo
use_stereo: false
max_cameras: 1

verbosity: INFO
```

---

## 3) If images are **compressed**, republish to raw

Install transports and run a republisher so OpenVINS gets a raw stream:

```bash
# In Terminal A
source /opt/ros/noetic/setup.bash
source ~/ws_openvins/devel/setup.bash
sudo apt-get update && sudo apt-get install -y   ros-noetic-image-transport ros-noetic-compressed-image-transport

# Republish /compressed -> /ov_cam0/image_raw
rosrun image_transport republish compressed   in:=/eo/color/image_color raw out:=/ov_cam0/image_raw
```

---

## 4) Launch OpenVINS with `subscribe.launch` (no internal bag playback)

```bash
# In Terminal B
source /opt/ros/noetic/setup.bash
source ~/ws_openvins/devel/setup.bash

# Use sim time for bag playback
rosparam set use_sim_time true

# Launch OpenVINS, point at your config file
roslaunch ov_msckf subscribe.launch   dobag:=false   config_path:=/root/ws_openvins/src/open_vins/config/my_dataset/estimator_config.yaml   use_stereo:=false max_cameras:=1 verbosity:=INFO
```

---

## 5) Play your rosbag

```bash
# In Terminal C
source /opt/ros/noetic/setup.bash
rosbag play /path/to/your_recording.bag --clock --pause
```

---

## 6) Save the trajectory

```bash
rostopic echo -p /ov_msckf/poseimu > /work/openvins_pose.csv
```

---

## 7) Troubleshooting

- **“Launch file not found”** → Use `subscribe.launch` (your repo doesn’t have `euroc_*.launch`).  
- **No tracking** → Confirm topics exist (`rosbag info your.bag`).  
- **Images are compressed** → Ensure republisher is running and YAML points to `/ov_cam0/image_raw`.  
- **Workspace not found** → Rebuild and re-source `~/ws_openvins/devel/setup.bash`.  

---

## 8) Notes

- Accurate calibration via **Kalibr** strongly recommended.  
- Stereo supported (`use_stereo:=true`).  
- Outputs: CSV, TUM, KITTI — you can add a small node to export in those formats.  
