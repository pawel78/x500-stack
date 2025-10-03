#!/usr/bin/env python3
"""
truncate_rosbag.py — Truncate a ROS1 .bag file by relative time window from the bag start.

Examples:
  # Keep from 5s to 65s after the bag's first message
  python3 truncate_rosbag.py input.bag output.bag --start 5 --end 65

  # Keep a 30s slice starting 10s in
  python3 truncate_rosbag.py input.bag output.bag --start 10 --duration 30

  # Keep the first 90s of the bag
  python3 truncate_rosbag.py input.bag output.bag --duration 90

  # Filter topics while truncating
  python3 truncate_rosbag.py input.bag output.bag --duration 60 --topics /cam0/image_raw /imu0

Notes:
  - Works with ROS bag v1 (.bag, ROS1). Not for ROS2 .db3.
  - Preserves message order and headers; copies connection info automatically.
  - You do NOT need roscore running, but you need ROS1's Python installed (rosbag, rospy or roslib).
  - If --compress is given, output uses the same compression as input unless explicitly set.
"""

import argparse
import os
import sys

try:
    import rosbag
    try:
        import rospy  # preferred for Time
    except Exception:
        rospy = None
        from genpy import Time as GenTime
except Exception as e:
    print("ERROR: This script requires ROS1's 'rosbag' (and ideally 'rospy') Python packages.", file=sys.stderr)
    print("Install ROS1, source your ROS environment (e.g., 'source /opt/ros/noetic/setup.bash'), and try again.", file=sys.stderr)
    sys.exit(1)


def make_time(sec_float):
    """Return a rospy.Time (or genpy.Time) from seconds (float)."""
    if rospy is not None:
        return rospy.Time.from_sec(sec_float)
    else:
        # Fallback without rospy
        return GenTime.from_sec(sec_float)


def parse_args():
    p = argparse.ArgumentParser(description="Truncate a ROS1 .bag by relative time from the bag start.")
    p.add_argument("input_bag", help="Path to input .bag (ROS1)")
    p.add_argument("output_bag", help="Path to output .bag")
    p.add_argument("--start", type=float, default=0.0,
                   help="Start time (seconds) relative to the bag's first message time. Default: 0.0")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--end", type=float, default=None,
                   help="End time (seconds) relative to the bag's first message time (exclusive).")
    g.add_argument("--duration", type=float, default=None,
                   help="Duration (seconds) from --start. Mutually exclusive with --end.")
    p.add_argument("--topics", nargs="*", default=None,
                   help="Optional list of topics to include (others are dropped).")
    p.add_argument("--compression", choices=["none", "bz2", "lz4"], default=None,
                   help="Force output compression. Default: match input bag's compression.")
    p.add_argument("--chunk-threshold", type=int, default=None,
                   help="Override chunk_threshold (bytes) for writing. Advanced.")
    p.add_argument("--force", action="store_true",
                   help="Overwrite output if it exists.")
    return p.parse_args()


def main():
    args = parse_args()

    if os.path.exists(args.output_bag):
        if not args.force:
            print(f"ERROR: Output bag exists: {args.output_bag}. Use --force to overwrite.", file=sys.stderr)
            sys.exit(2)
        else:
            os.remove(args.output_bag)

    # Open input to determine time bounds and compression
    with rosbag.Bag(args.input_bag, "r") as inbag:
        if inbag.size == 0:
            print("ERROR: Input bag is empty.", file=sys.stderr)
            sys.exit(3)

        t0 = inbag.get_start_time()  # float seconds
        tN = inbag.get_end_time()    # float seconds

        if args.end is None and args.duration is None:
            # Default to full bag starting at --start to end
            rel_end = tN - t0
        elif args.duration is not None:
            if args.duration < 0:
                print("ERROR: --duration must be >= 0.", file=sys.stderr)
                sys.exit(4)
            rel_end = args.start + args.duration
        else:
            rel_end = args.end

        if args.start < 0:
            print("ERROR: --start must be >= 0.", file=sys.stderr)
            sys.exit(5)

        if rel_end is None:
            print("ERROR: internal: rel_end is None.", file=sys.stderr)
            sys.exit(6)

        if rel_end < args.start:
            print("ERROR: End (<start). Ensure --end >= --start or use --duration >= 0.", file=sys.stderr)
            sys.exit(7)

        # Convert relative window to absolute bag times
        abs_start = t0 + args.start
        abs_end = t0 + rel_end

        # Clamp to bag bounds
        abs_start = max(abs_start, t0)
        abs_end = min(abs_end, tN)

        if abs_start >= abs_end:
            print("ERROR: The requested window is empty after clamping to bag bounds.", file=sys.stderr)
            print(f"Bag span: [{t0:.3f}, {tN:.3f}] (len={tN - t0:.3f}s); requested [{abs_start:.3f}, {abs_end:.3f}].", file=sys.stderr)
            sys.exit(8)

        # Determine output compression
        in_compression = inbag._compression  # 'none' | 'bz2' | 'lz4'
        out_compression = args.compression if args.compression is not None else in_compression

        # Prepare writer
        write_kwargs = {}
        if out_compression and out_compression != "none":
            write_kwargs["compression"] = out_compression
        if args.chunk_threshold is not None:
            write_kwargs["chunk_threshold"] = args.chunk_threshold

        abs_start_t = make_time(abs_start)
        abs_end_t = make_time(abs_end)

        total_written = 0
        total_read = 0

        with rosbag.Bag(args.output_bag, "w", **write_kwargs) as outbag:
            # Efficiently iterate using start_time / end_time filters
            for topic, msg, t in inbag.read_messages(start_time=abs_start_t, end_time=abs_end_t, topics=args.topics):
                outbag.write(topic, msg, t)
                total_written += 1
            # For reporting, we can also count total read in window if needed
            # But read_messages already filtered; so total_read == total_written
            total_read = total_written

        kept = abs_end - abs_start
        print(f"✔ Wrote {total_written} messages to '{args.output_bag}'")
        print(f"   Window: [{args.start:.3f}s .. {rel_end:.3f}s] rel  =>  [{abs_start:.3f}s .. {abs_end:.3f}s] abs (len={kept:.3f}s)")
        print(f"   Compression: {out_compression or 'none'}; Topics: {'ALL' if not args.topics else ','.join(args.topics)}")

if __name__ == "__main__":
    main()
