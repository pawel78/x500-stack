# OpenVINS ETH Bag Demo Setup (Colima + Docker on macOS)

This guide explains how to run **OpenVINS** with the ETH Zurich EuRoC MAV dataset inside a Colima-managed Docker container on macOS.  
We assume you already have Colima installed and configured as your virtualization backend.

---

## 1. Start Colima VM
Make sure Colima is running:
```bash
colima start --cpu 4 --memory 8 --disk 60
```
Check status:
```bash
colima status
```
## 2. Build and Start the OpenVINS Container
From the x500-stack project repo:
```bash
docker build -t openvins:latest .
```
Start the container iwth ROS networking enabled:
```bash
docker run -it --rm --name openvins --network hose -v ~/dataset:/dataset -v $(pwd):/root/ws_openvins openvins:latest bash
```
Note:
1. --network host ensures ROS master is reachable across terminals.
2. ~/datasets/bags should contain EuRoC bag files (e.g. MH_01_easy.bag).
3. Code is mounted at /root/ws_openvins.s
## 3. Source workspaces
Inside the container, before running ROS commands:
```bash
source /opt/ros/noetic/setup.bash
source /root/ws_openvins/devel/setup.bash
```
To avoid repeating, add these lines to ~/.bashrc in the container

## 4. Run ROS Master
In terminal 1 (exec inot the container):
```bash
roscore
```

## 5. Play euroc_mav bag
In terminal 2:
```bash
docker -exec -it openvins bash
source /opt/ros/noetic/setup.bash
rosbag play /datasets/MH_01_easy.bag --clock
```

## 6. Launch OpenVINS with euroc config
In terminal 3:
```bash
docker exec -it openvins bash
source /opt/ros/noetic/setup.bash
source /root/ws_openvins/devel/setup.bash
roslaunch ov_msckf subscribe.launch config_path:=/root/ws_openvins/src/open_vins/config/euroc_mav/estimator.yaml use_stereo:=true dobag:=false
```

## 7. Check topics
Verify estimator is publishing:
```bash
rostopic list | grep ov_msckf
```
Examples:
    /ov_msckf/poseimu → pose estimate
    /ov_msckf/odomimu → odometry
    /ov_msckf/points_msckf → tracked features

If you don’t see data:
    Ensure /clock is being published (rostopic echo /clock)
    Make sure you did not remap topics incorrectly. Default EuRoC topics are:
        /cam0/image_raw
        /cam1/image_raw
        /imu0

## 8. Exec into Container Later
if container is still running
```bash 
docker exec -it openvins bash
```
If container stopped, restart using docker run command above

## 9. Cleanup
When finished:
```bash
docker stop openvins
colima stop
```

## References:
1. OpenVINS GitHub
2. EuRoC MAV Dataset