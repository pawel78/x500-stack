# Running Visual–Inertial Odometry (OpenVINS) on ROS Bags in Docker

This document describes how to run our VIO + mapping pipeline inside Docker, with host data mounted into the container.

---

## 1. Prerequisites
- **Docker Desktop** installed (macOS with Apple Silicon is fine; we’ll run x86 emulation).
- ROS Noetic–based container image with OpenVINS built (e.g. `my-ros:noetic-vio`).
- Bag files stored under:
  ```
  /Users/pawel/x500-stack/maps/bags
  ```

---

## 2. Start the container

Run from the host:

```bash
docker run --platform=linux/amd64 -it --rm   --name ros_vio   -v /Users/pawel/x500-stack/maps:/datasets   -v /Users/pawel/x500-stack/ws_openvins:/ws_openvins   --memory=12g --cpus=6   my-ros:noetic-vio bash
```

- `/datasets` → maps + bags mounted from host
- `/ws_openvins` → Catkin workspace on host
- `--platform=linux/amd64` → ensures Noetic image runs on Apple Silicon
- `--memory/--cpus` → optional resource limits

---

## 3. Source ROS and workspace

Inside the container:

```bash
source /opt/ros/noetic/setup.bash
cd /ws_openvins
catkin_make -DCMAKE_BUILD_TYPE=Release   # only needed first time
source devel/setup.bash
```

---

## 4. Verify bag access

Check your bags are visible:

```bash
ls -la /datasets/bags
```

Expected:
```
2022-12-20-12-48-59.bag
...
```

---

## 5. Run VIO pipeline (multi-terminal workflow)

We need four concurrent ROS processes:

1. **roscore**
   ```bash
   roscore
   ```

2. **Republish compressed → raw images**  
   (skip if your bag already has `/.../image_raw`)
   ```bash
   rosrun image_transport republish compressed      in:=/eo/color/image_color raw out:=/ov_cam0/image_raw
   ```

3. **Play the bag**
   ```bash
   rosparam set use_sim_time true
   rosbag play /datasets/bags/2022-12-20-12-48-59.bag --clock --pause
   ```

4. **Launch OpenVINS**
   ```bash
   roslaunch ov_msckf subscribe.launch      dobag:=false      use_stereo:=false      max_cameras:=1      config_path:=/ws_openvins/src/open_vins/config/my_dataset/estimator_config.yaml
   ```

---

## 6. Managing terminals

Options:
- Open 4 separate shells (`docker exec -it ros_vio bash` for each).
- Or, use **tmux** inside the container to split into panes:
  ```bash
  tmux new-session \; split-window -h \; split-window -v \; select-pane -t 0 \; split-window -v
  ```

---

## 7. Output trajectory

To save poses:
```bash
rostopic echo -p /ov_msckf/poseimu > /datasets/vio_trajectory.csv
```

---

## 8. Stopping

- Detach tmux: `Ctrl+b d`
- Kill tmux session: `tmux kill-server`
- Exit container: `exit` (on all shells)  
- If you didn’t use `--rm`, you can also `docker stop ros_vio`

---

✅ With this setup, you can restart any time by repeating step **2**, and your bags/workspace are always available under `/datasets` and `/ws_openvins`.
