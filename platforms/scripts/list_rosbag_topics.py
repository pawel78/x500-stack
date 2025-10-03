#!/usr/bin/env python3
"""
List all connections (topic + type) in a ROS bag (ROS1 .bag or ROS2 folder/.db3)
Optionally count messages per topic.

Usage:
  python list_rosbag_topics.py /path/to/bag_or_ros2_dir
  python list_rosbag_topics.py /path/to/file.bag --counts
  python list_rosbag_topics.py /path/to/ros2_bag_dir --counts
"""
import argparse
from pathlib import Path
from collections import defaultdict

from rosbags.highlevel import AnyReader


def type_name(msgtype) -> str:
    # rosbags uses pythonic type names (e.g., sensor_msgs__msg__Image)
    # Convert to "pkg/msg/Type" for readability.
    raw = getattr(msgtype, "__name__", type(msgtype).__name__)
    if "__" in raw:
        parts = raw.split("__")
        if len(parts) >= 3 and parts[1] == "msg":
            return f"{parts[0]}/{parts[1]}/{parts[2]}"
    return raw


def list_connections(bag_path: Path, with_counts: bool) -> None:
    if not bag_path.exists():
        raise SystemExit(f"Path not found: {bag_path}")

    # Open with AnyReader (works for ROS1 .bag and ROS2 dirs/.db3 with metadata.yaml)
    with AnyReader([bag_path]) as reader:
        conns = sorted(reader.connections, key=lambda c: (c.topic, type_name(c.msgtype)))

        # Optional: count messages per connection (can take time on big bags)
        counts = defaultdict(int)
        if with_counts:
            for conn, _, _ in reader.messages():
                counts[conn] += 1

        # Group by topic for a clean print
        by_topic = defaultdict(list)
        for c in conns:
            by_topic[c.topic].append(c)

        # Pretty print
        print(f"\nFound {len(conns)} connections across {len(by_topic)} topics in: {bag_path}\n")
        for topic in sorted(by_topic.keys()):
            print(f"Topic: {topic}")
            for c in by_topic[topic]:
                tname = type_name(c.msgtype)
                if with_counts:
                    print(f"  - type: {tname:35s}  messages: {counts[c]}")
                else:
                    print(f"  - type: {tname}")
            print()

        # Quick summary table
        print("Summary (topic → unique types):")
        for topic in sorted(by_topic.keys()):
            uniq_types = sorted({type_name(c.msgtype) for c in by_topic[topic]})
            print(f"  {topic}: {', '.join(uniq_types)}")


def main():
    ap = argparse.ArgumentParser(description="List connections/topics in a ROS bag (ROS1/ROS2).")
    ap.add_argument("bag", type=Path, help="Path to .bag (ROS1) or ROS2 bag directory/.db3")
    ap.add_argument("--counts", action="store_true", help="Count messages per connection (slower)")
    args = ap.parse_args()
    list_connections(args.bag, args.counts)


if __name__ == "__main__":
    main()
