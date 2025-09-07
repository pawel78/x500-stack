# Foxglove for Log Visualization

We are using **Foxglove Studio** as our primary tool for visualizing flight data logs
(ROS bag and MCAP formats). This gives us an efficient way to explore raw sensor data
(RGB, thermal, IMU, pose) and collaborate internally.

---

## Why Foxglove?

- **Multi-format support** – reads both `.bag` (ROS1/ROS2) and `.mcap` logs directly.  
- **Rich visualization** – side-by-side image viewers (RGB vs Thermal), time-series plots,
  3D pose/TF view, and custom dashboards.  
- **Cross-platform** – available as a desktop app or via browser (web.foxglove.dev).  
- **Team-friendly** – free tier supports up to **3 teammates**, which fits our stealth robotics venture setup.  

---

## Our Workflow

1. **Collect Data**  
   - Raw flight data (ROS bag or MCAP) is stored in `/data/logs/...`.

2. **Open in Foxglove**  
   - Launch Foxglove Studio (desktop app preferred).  
   - Open a local file (`File → Open` → select `.bag` or `.mcap`).  
   - Alternatively, stream live using `rosbag play` and connect Foxglove to a local bridge.

3. **Visualize**  
   - Use the default viewing panels:  
     - **Image**: RGB and Thermal topics side by side.  
     - **Plot**: IMU, battery, and pose time series.  
     - **3D**: Vehicle pose and TF tree (if available).  
   - Or create a custom one for your needs.

4. **Save Layouts**  
   - Each teammate should save layouts (`.json`) and commit them under `/layouts/` for reuse.  

## References

- [Foxglove Studio – Website](https://foxglove.dev)  
- [Foxglove Studio – GitHub](https://github.com/foxglove/studio)  
- [MCAP Log Format](https://mcap.dev)  
- [ROS Bag Format](http://wiki.ros.org/Bags)  

---

## Notes

- We operate under a **stealth robotics venture** – data and layouts are for internal use only.  
- Free tier covers up to **3 teammates** – keep accounts limited until we decide on a paid plan.  

