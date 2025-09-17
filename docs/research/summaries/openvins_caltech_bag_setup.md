# OpenVINS + Caltech Bag Setup (Mono Camera)

This guide documents how to run **OpenVINS** against the Caltech dataset bag inside your existing Colima + Docker setup.  
It assumes you already confirmed the EuRoC demo runs successfully, so your environment and container are working.

---

## 1. Copy and Adapt Estimator Config

Start from the EuRoC example config:

```bash
cp /root/ws_openvins/src/open_vins/config/euroc_mav/estimator_config.yaml \
   /root/ws_openvins/src/open_vins/config/caltech_mono_estimator.yaml
```

Edit the copy:

- Set **mono mode**:
  ```yaml
  use_stereo: 0
  max_cameras: 1
  ```
- Keep references to your **Kalibr imu + imu-cam yaml files**:
  ```yaml
  relative_config_imu: "/root/ws_openvins/config/kalibr_imu_chain.yaml"
  relative_config_imucam: "/root/ws_openvins/config/kalibr_imucam_chain.yaml"
  ```
- Update camera intrinsics, distortion, resolution, and extrinsics to match your Caltech calibration.

---

## 2. Handle Topic Names

### IMU
Your Caltech bag publishes IMU as `/imu/imu`.  
OpenVINS expects `/imu0`.  
Remap during bag playback:
```bash
rosbag play /bags/caltech_dataset.bag --clock /imu/imu:=/imu0
```

### Camera
Your bag stores the camera as **compressed images**.  
Republish them as raw on `/cam0/image_raw`:

```bash
rosrun image_transport republish compressed in:=/cam0/image_raw/compressed raw out:=/cam0/image_raw
```

> Run this in a separate terminal inside the container while the bag is playing.

---

## 3. Launch Sequence (3 terminals)

### T1 — Launch OpenVINS
```bash
docker exec -it ovdev bash
source /opt/ros/noetic/setup.bash
source /root/ws_openvins/devel/setup.bash

rosparam set use_sim_time true
rosparam delete /ov_msckf 2>/dev/null || true

roslaunch ov_msckf subscribe.launch \
  dobag:=false use_stereo:=false max_cameras:=1 \
  config_path:=/root/ws_openvins/src/open_vins/config/caltech_mono_estimator.yaml
```

### T2 — Play the Caltech bag
```bash
docker exec -it ovdev bash
source /opt/ros/noetic/setup.bash

rosbag play /bags/caltech_dataset.bag --clock /imu/imu:=/imu0
```

### T3 — Republish the camera
```bash
docker exec -it ovdev bash
source /opt/ros/noetic/setup.bash

rosrun image_transport republish compressed in:=/cam0/image_raw/compressed raw out:=/cam0/image_raw
```

---

## 4. Inspect Outputs

Inside any container shell:

```bash
# Verify inputs
rostopic hz /cam0/image_raw
rostopic hz /imu0

# Check OV publications
rostopic list | grep ov_msckf
rostopic echo /ov_msckf/odomimu -n 1
```

RViz setup:
- Fixed Frame: `world`
- Displays:
  - **Odometry** on `/ov_msckf/odomimu`
  - **TF**
  - **Image** on `/ov_msckf/trackhist`

---

## 5. Common Issues

- **No messages** → Ensure `--clock` and `use_sim_time true`.
- **No images** → Make sure republish node is running (`rostopic hz /cam0/image_raw`).
- **Wrong topics** → Double-check remaps (`/imu/imu` → `/imu0`, `/cam0/image_raw/compressed` → `/cam0/image_raw`).
- **Initialization stuck** → Let bag play for at least 10–15 seconds with enough motion.

---

## 6. Summary of Differences from EuRoC Setup

- Switched config to **mono camera**.
- Kept **Kalibr imu + imu-cam chain** files referenced in config.
- Remapped **IMU topic** `/imu/imu` → `/imu0`.
- Added **republish node** to convert `/cam0/image_raw/compressed` → `/cam0/image_raw`.

This workflow allows running the Caltech dataset bag through OpenVINS without modifying the source code, just by adjusting configuration and topic handling.
