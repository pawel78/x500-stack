#!/usr/bin/env python3
import argparse, os, math, csv
from pathlib import Path
from typing import Optional

from rosbags.highlevel import AnyReader

def parse_args():
    p = argparse.ArgumentParser("Sample a pose topic from a ROS1 .bag and write CSV.")
    p.add_argument("--bag", required=True)
    p.add_argument("--topic", default="/uav1/mavros/local_position/pose",
                   help="Pose topic (PoseStamped or Odometry).")
    p.add_argument("--outdir", required=True, help="Directory to place the CSV file.")
    p.add_argument("--start", type=float, default=0.0, help="seconds from bag start")
    p.add_argument("--end", type=float, default=float("inf"), help="seconds from bag start")
    p.add_argument("--sample", type=int, default=1,
                   help="Sampling stride: 1=every message, 2=every 2nd, etc.")
    p.add_argument("--prefix", default="pose_", help="(kept for compatibility; not used for files)")
    p.add_argument("--csv", default="poses.csv", help="output CSV filename")
    return p.parse_args()

def get_header_stamp_ns(msg) -> Optional[int]:
    try:
        s = getattr(msg.header.stamp, "sec", None)
        ns = getattr(msg.header.stamp, "nsec", None)
        if s is not None and ns is not None:
            return int(s) * 1_000_000_000 + int(ns)
        # some schemas use .nanosec
        nsec = getattr(msg.header.stamp, "nanosec", None)
        if nsec is not None:
            return int(nsec)
    except Exception:
        pass
    return None

def get_frame_id(msg) -> str:
    try:
        return str(getattr(msg.header, "frame_id", "")) or ""
    except Exception:
        return ""

def extract_pose_fields(msg):
    """
    Return (x,y,z,qx,qy,qz,qw) from either PoseStamped or Odometry message.
    Raise ValueError if unsupported.
    """
    # geometry_msgs/PoseStamped: msg.pose.{position,orientation}
    if hasattr(msg, "pose") and hasattr(msg.pose, "position") and hasattr(msg.pose, "orientation"):
        p = msg.pose.position
        o = msg.pose.orientation
        return float(p.x), float(p.y), float(p.z), float(o.x), float(o.y), float(o.z), float(o.w)

    # nav_msgs/Odometry: msg.pose.pose.{position,orientation}
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose") \
       and hasattr(msg.pose.pose, "position") and hasattr(msg.pose.pose, "orientation"):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        return float(p.x), float(p.y), float(p.z), float(o.x), float(o.y), float(o.z), float(o.w)

    raise ValueError("Unsupported message type: expected PoseStamped or Odometry-like layout.")

def main():
    a = parse_args()
    outdir = Path(a.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    csv_path = outdir / a.csv

    with AnyReader([Path(a.bag)]) as reader:
        bag_start_ns = reader.start_time
        bag_end_ns   = reader.end_time
        t0_ns = bag_start_ns + int(max(0.0, a.start) * 1e9)
        t1_ns = bag_start_ns + (int(a.end * 1e9) if math.isfinite(a.end) else (bag_end_ns - bag_start_ns))
        if t1_ns <= t0_ns:
            raise ValueError("Invalid window: --end must be > --start.")

        # Select the topic
        conns = [c for c in reader.connections if c.topic == a.topic]
        if not conns:
            raise RuntimeError(f"Topic not found: {a.topic}")

        seen = saved = 0

        with csv_path.open("w", newline="") as cf:
            w = csv.writer(cf)
            w.writerow(["t_ros_ns","header_stamp_ns","frame_id","x","y","z","qx","qy","qz","qw"])

            for conn, ts, raw in reader.messages(connections=conns):
                if ts < t0_ns:
                    continue
                if ts > t1_ns:
                    break

                seen += 1
                if seen % a.sample != 0:
                    continue  # skip based on stride

                msg = reader.deserialize(raw, conn.msgtype)
                try:
                    x,y,z,qx,qy,qz,qw = extract_pose_fields(msg)
                except ValueError:
                    continue

                header_ns = get_header_stamp_ns(msg)
                frame_id  = get_frame_id(msg)

                w.writerow([
                    int(ts),
                    "" if header_ns is None else int(header_ns),
                    frame_id,
                    f"{x:.6f}", f"{y:.6f}", f"{z:.6f}",
                    f"{qx:.9f}", f"{qy:.9f}", f"{qz:.9f}", f"{qw:.9f}"
                ])
                saved += 1

    print(f"[INFO] Done. Messages seen in window: {seen}, rows written: {saved}")
    print(f"[INFO] CSV: {csv_path.resolve()}")

if __name__ == "__main__":
    main()
