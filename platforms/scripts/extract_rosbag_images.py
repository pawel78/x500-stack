#!/usr/bin/env python3
import argparse, os, math, csv
from pathlib import Path
from rosbags.highlevel import AnyReader

def parse_args():
    p = argparse.ArgumentParser("Extract JPEGs from /.../compressed in a ROS1 .bag using rosbags.")
    p.add_argument("--bag", required=True)
    p.add_argument("--topic", default="/eo/color/image_color/compressed")
    p.add_argument("--outdir", required=True)
    p.add_argument("--start", type=float, default=0.0, help="seconds from bag start")
    p.add_argument("--end", type=float, default=float("inf"), help="seconds from bag start")
    p.add_argument("--rate", type=float, default=1.0, help="fps")
    p.add_argument("--prefix", default="frame_")
    p.add_argument("--csv", default="frames.csv", help="output CSV {filename,rostime_ns}")
    return p.parse_args()

def main():
    a = parse_args()
    Path(a.outdir).mkdir(parents=True, exist_ok=True)

    with AnyReader([Path(a.bag)]) as reader:
        # rosbags uses nanoseconds
        bag_start_ns = reader.start_time
        bag_end_ns   = reader.end_time
        t0_ns = bag_start_ns + int(max(0.0, a.start) * 1e9)
        t1_ns = bag_start_ns + (int(a.end * 1e9) if math.isfinite(a.end) else (bag_end_ns - bag_start_ns))

        if t1_ns <= t0_ns:
            raise ValueError("Invalid window: --end must be > --start.")

        dt_ns = int(1e9 / max(a.rate, 1e-9))
        next_save_ns = t0_ns

        # Filter to the one topic
        conns = [c for c in reader.connections if c.topic == a.topic]
        if not conns:
            raise RuntimeError(f"Topic not found: {a.topic}")

        saved = 0
        seen  = 0
        csv_path = os.path.join(a.outdir, a.csv)
        with open(csv_path, "w", newline="") as cf:
            w = csv.writer(cf)
            w.writerow(["filename", "rostime_ns"])

            for conn, ts, raw in reader.messages(connections=conns):
                # ts is in ns
                if ts < t0_ns:
                    continue
                if ts > t1_ns:
                    break
                seen += 1

                if ts >= next_save_ns:
                    # Deserialize; CompressedImage has .format (e.g., "jpeg") and .data (bytes)
                    msg = reader.deserialize(raw, conn.msgtype)
                    # Default to .jpg; if you care, switch extension based on msg.format
                    fname = f"{a.prefix}{saved:06d}.jpg"
                    with open(os.path.join(a.outdir, fname), "wb") as f:
                        f.write(bytes(msg.data))
                    w.writerow([fname, ts])
                    saved += 1

                    # schedule next sample
                    next_save_ns += dt_ns
                    if ts > next_save_ns:
                        # catch up if we fell behind
                        k = (ts - next_save_ns) // dt_ns + 1
                        next_save_ns += k * dt_ns

        print(f"[INFO] Done. Messages seen in window: {seen}, frames saved: {saved}")
        print(f"[INFO] Output dir: {os.path.abspath(a.outdir)}")
        print(f"[INFO] CSV: {os.path.abspath(csv_path)}")

if __name__ == "__main__":
    main()
