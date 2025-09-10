#!/usr/bin/env python3
"""
Geotag frames from a ROS1 bag using nearest GPS fix (or Odometry+origin):
- Extract JPEGs from a compressed image topic
- Attach EXIF GPS (WGS-84) per frame using the closest GPS fix in time
- Optional fallback: Odometry (local ENU) + --origin "lat,lon,alt"
- Write a CSV: filename, t_ros_ns, lat, lon, alt

Requires (Jammy-friendly):
  pip install "rosbags>=0.9,<1" piexif numpy pandas

Examples:
  # GPS nearest-neighbor (no gap limit)
  python3 ../platforms/scripts/geotag_images_from_bag.py \
    --bag bags/2022-12-20-12-48-59.bag \
    --img-topic /eo/color/image_color/compressed \
    --gps-topic /gps/fix \
    --outdir images_geotagged --start 120 --end 480 --sample 5

  # GPS nearest-neighbor with 2s max allowed gap (skip if farther)
  python3 geotag_images_from_bag.py \
    --bag bags/2022-12-20-12-48-59.bag \
    --img-topic /eo/color/image_color/compressed \
    --gps-topic /gps/fix \
    --outdir images --max-gps-gap 2.0

  # Odometry (ENU) + known origin (if no GPS):
  python3 geotag_images_from_bag.py \
    --bag bags/2022-12-20-12-48-59.bag \
    --img-topic /eo/color/image_color/compressed \
    --odom-topic /uav1/mavros/local_position/odom \
    --origin "34.123456,-118.123456,250.0" \
    --outdir images --sample 3
"""

import argparse
import bisect
import csv
import math
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import piexif
from rosbags.highlevel import AnyReader

# ----------------- Small EXIF helpers -----------------

def _to_dms_rationals(deg: float):
    # Decimal degrees -> EXIF DMS rationals; seconds with denom 100 for precision.
    deg_abs = abs(deg)
    d = int(deg_abs)
    m_float = (deg_abs - d) * 60.0
    m = int(m_float)
    s = (m_float - m) * 60.0
    return ((d, 1), (m, 1), (int(round(s * 100)), 100))

def _gps_ref(lat: float, lon: float):
    lat_ref = b'N' if lat >= 0 else b'S'
    lon_ref = b'E' if lon >= 0 else b'W'
    return lat_ref, lon_ref

def _load_or_create_exif(jpeg_path: Path) -> dict:
    try:
        exif_dict = piexif.load(str(jpeg_path))
    except Exception:
        exif_dict = {"0th":{}, "Exif":{}, "GPS":{}, "Interop":{}, "1st":{}, "thumbnail":None}
    exif_dict.setdefault("GPS", {})
    return exif_dict

def _write_exif(jpeg_path: Path, exif_dict: dict):
    exif_bytes = piexif.dump(exif_dict)
    piexif.insert(exif_bytes, str(jpeg_path))

# ----------------- ENU <-> LLA (no external deps) -----------------

_WGS84_A = 6378137.0
_WGS84_E2 = 6.69437999014e-3

def _lla_to_ecef(lat_deg, lon_deg, alt_m):
    lat = math.radians(lat_deg); lon = math.radians(lon_deg)
    a = _WGS84_A; e2 = _WGS84_E2
    sinlat = math.sin(lat); coslat = math.cos(lat)
    sinlon = math.sin(lon); coslon = math.cos(lon)
    N = a / math.sqrt(1 - e2 * sinlat * sinlat)
    x = (N + alt_m) * coslat * coslon
    y = (N + alt_m) * coslat * sinlon
    z = (N * (1 - e2) + alt_m) * sinlat
    return np.array([x, y, z], dtype=float)

def _ecef_to_lla(x, y, z):
    a = _WGS84_A; e2 = _WGS84_E2
    b = a * math.sqrt(1 - e2)
    ep2 = (a*a - b*b) / (b*b)
    p = math.sqrt(x*x + y*y)
    th = math.atan2(a*z, b*p)
    lon = math.atan2(y, x)
    sinth = math.sin(th); costh = math.cos(th)
    lat = math.atan2(z + ep2 * b * sinth**3, p - e2 * a * costh**3)
    sinlat = math.sin(lat)
    N = a / math.sqrt(1 - e2 * sinlat * sinlat)
    alt = p / math.cos(lat) - N
    return math.degrees(lat), math.degrees(lon), alt

def _enu_to_ecef(dx, dy, dz, lat0_deg, lon0_deg, alt0_m):
    lat0 = math.radians(lat0_deg); lon0 = math.radians(lon0_deg)
    sinlat = math.sin(lat0); coslat = math.cos(lat0)
    sinlon = math.sin(lon0); coslon = math.cos(lon0)
    # ENU->ECEF rotation (matrix columns are E,N,U in ECEF basis)
    R = np.array([[-sinlon,               coslon,              0.0     ],
                  [-sinlat*coslon, -sinlat*sinlon,  coslat],
                  [ coslat*coslon,  coslat*sinlon,  sinlat]], dtype=float)
    ecef0 = _lla_to_ecef(lat0_deg, lon0_deg, alt0_m)
    de = np.array([dx, dy, dz], dtype=float)
    return ecef0 + R.T @ de

def enu_to_lla(x_e, y_n, z_u, lat0, lon0, alt0):
    xe, ye, ze = _enu_to_ecef(x_e, y_n, z_u, lat0, lon0, alt0)
    return _ecef_to_lla(xe, ye, ze)

# ----------------- Nearest-neighbor helper -----------------

def nearest_idx(sorted_ts_ns: List[int], t_ns: int) -> int:
    """Return index of closest timestamp in sorted_ts_ns to t_ns."""
    i = bisect.bisect_left(sorted_ts_ns, t_ns)
    if i == 0:
        return 0
    if i == len(sorted_ts_ns):
        return len(sorted_ts_ns) - 1
    before = sorted_ts_ns[i-1]
    after  = sorted_ts_ns[i]
    return i-1 if (t_ns - before) <= (after - t_ns) else i

# ----------------- CLI & main -----------------

def parse_args():
    p = argparse.ArgumentParser("Extract & geotag JPEGs from a ROS1 bag (GPS-nearest or Odom+origin).")
    p.add_argument("--bag", required=True)
    p.add_argument("--img-topic", required=True,
                   help="sensor_msgs/CompressedImage topic (e.g., /eo/color/image_color/compressed)")
    p.add_argument("--outdir", required=True, help="Folder for images + CSV.")
    p.add_argument("--start", type=float, default=0.0, help="seconds from bag start")
    p.add_argument("--end", type=float, default=float("inf"), help="seconds from bag start")
    p.add_argument("--sample", type=int, default=1, help="stride: 1=every frame, 2=every other, ...")

    # Geotag sources
    p.add_argument("--gps-topic", default=None, help="sensor_msgs/NavSatFix topic (preferred if provided).")
    p.add_argument("--odom-topic", default=None, help="nav_msgs/Odometry (local ENU).")
    p.add_argument("--origin", default=None, help='Required if using --odom-topic and no GPS. Format: "lat,lon,alt"')
    p.add_argument("--max-gps-gap", type=float, default=None,
                   help="Max allowed time gap (seconds) between image and nearest GPS; skip if exceeded.")

    # CSV / naming
    p.add_argument("--prefix", default="frame_", help="output filename prefix")
    p.add_argument("--csv", default="frames_geotagged.csv", help="CSV filename to write")

    return p.parse_args()

def main():
    a = parse_args()
    outdir = Path(a.outdir); outdir.mkdir(parents=True, exist_ok=True)
    csv_path = outdir / a.csv
    stride = max(1, int(a.sample))

    # Parse origin if provided
    origin_lla = None
    if a.origin:
        try:
            lat0, lon0, alt0 = [float(x) for x in a.origin.split(",")]
            origin_lla = (lat0, lon0, alt0)
        except Exception:
            raise SystemExit('[ERROR] Bad --origin format. Use: --origin "lat,lon,alt"')

    with AnyReader([Path(a.bag)]) as reader:
        bag_start_ns = reader.start_time
        bag_end_ns   = reader.end_time
        t0_ns = bag_start_ns + int(max(0.0, a.start) * 1e9)
        t1_ns = bag_start_ns + (int(a.end * 1e9) if math.isfinite(a.end) else (bag_end_ns - bag_start_ns))
        if t1_ns <= t0_ns:
            raise SystemExit("[ERROR] Invalid window: --end must be > --start")

        # Connections
        img_conns  = [c for c in reader.connections if c.topic == a.img_topic]
        if not img_conns:
            raise SystemExit(f"[ERROR] Image topic not found: {a.img_topic}")

        gps_conns  = [c for c in reader.connections if a.gps_topic and c.topic == a.gps_topic]
        odom_conns = [c for c in reader.connections if a.odom_topic and c.topic == a.odom_topic]

        # Preload GPS
        gps_ts, lats, lons, alts = [], [], [], []
        if gps_conns:
            for conn, ts, raw in reader.messages(connections=gps_conns):
                msg = reader.deserialize(raw, conn.msgtype)
                # Basic sanity: skip NaN fixes if present
                lat = float(getattr(msg, "latitude"))
                lon = float(getattr(msg, "longitude"))
                alt = float(getattr(msg, "altitude"))
                if not (math.isfinite(lat) and math.isfinite(lon) and math.isfinite(alt)):
                    continue
                gps_ts.append(ts); lats.append(lat); lons.append(lon); alts.append(alt)
            if gps_ts:
                order = np.argsort(gps_ts)
                gps_ts = list(np.array(gps_ts)[order])
                lats   = list(np.array(lats)[order])
                lons   = list(np.array(lons)[order])
                alts   = list(np.array(alts)[order])

        # Preload Odometry positions
        od_ts, pos_list = [], []
        if odom_conns:
            for conn, ts, raw in reader.messages(connections=odom_conns):
                msg = reader.deserialize(raw, conn.msgtype)
                # Handle PoseStamped-like or Odometry-like layout
                try:
                    p = msg.pose.pose.position  # Odometry
                except Exception:
                    p = msg.pose.position       # PoseStamped
                x, y, z = float(p.x), float(p.y), float(p.z)
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                od_ts.append(ts); pos_list.append(np.array([x, y, z], dtype=float))
            if od_ts:
                order = np.argsort(od_ts)
                od_ts    = list(np.array(od_ts)[order])
                pos_list = list(np.array(pos_list, dtype=object)[order])

        if not gps_conns and not odom_conns:
            raise SystemExit("[ERROR] Provide --gps-topic OR --odom-topic (with --origin).")
        if odom_conns and not gps_conns and origin_lla is None:
            raise SystemExit("[ERROR] Using odom without GPS requires --origin \"lat,lon,alt\".")

        seen = saved = 0

        with csv_path.open("w", newline="") as cf:
            w = csv.writer(cf)
            w.writerow(["filename", "t_ros_ns", "lat", "lon", "alt"])

            for conn, ts, raw in reader.messages(connections=img_conns):
                if ts < t0_ns:
                    continue
                if ts > t1_ns:
                    break

                seen += 1
                if (seen % stride) != 0:
                    continue

                # 1) Try GPS nearest-neighbor
                lat = lon = alt = None
                if gps_ts:
                    j = nearest_idx(gps_ts, ts)
                    # optional max gap
                    if a.max_gps_gap is not None:
                        gap_s = abs(gps_ts[j] - ts) / 1e9
                        if gap_s <= a.max_gps_gap:
                            lat, lon, alt = lats[j], lons[j], alts[j]
                    else:
                        lat, lon, alt = lats[j], lons[j], alts[j]

                # 2) Fallback: Odometry (ENU) + origin
                if (lat is None) and od_ts:
                    # Nearest odom sample (simpler than interpolation)
                    j = nearest_idx(od_ts, ts)
                    x_e, y_n, z_u = pos_list[j]
                    lat_i, lon_i, alt_i = enu_to_lla(x_e, y_n, z_u, origin_lla[0], origin_lla[1], origin_lla[2])
                    lat, lon, alt = float(lat_i), float(lon_i), float(alt_i)

                if lat is None:
                    # No geotag available; skip this frame
                    continue

                # Deserialize CompressedImage and write JPEG
                msg = reader.deserialize(raw, conn.msgtype)
                fmt = (getattr(msg, "format", "") or "").lower()
                ext = ".jpg" if ("jpeg" in fmt or "jpg" in fmt or fmt == "") else (".png" if "png" in fmt else ".jpg")
                fname = f"{a.prefix}{saved:06d}{ext}"
                jpg_path = outdir / fname
                with open(jpg_path, "wb") as f:
                    f.write(bytes(msg.data))

                # Write EXIF GPS
                exif = _load_or_create_exif(jpg_path)
                gps_ifd = exif["GPS"]
                lat_ref, lon_ref = _gps_ref(lat, lon)
                gps_ifd[piexif.GPSIFD.GPSLatitudeRef]  = lat_ref
                gps_ifd[piexif.GPSIFD.GPSLatitude]     = _to_dms_rationals(lat)
                gps_ifd[piexif.GPSIFD.GPSLongitudeRef] = lon_ref
                gps_ifd[piexif.GPSIFD.GPSLongitude]    = _to_dms_rationals(lon)
                gps_ifd[piexif.GPSIFD.GPSAltitudeRef]  = 0
                gps_ifd[piexif.GPSIFD.GPSAltitude]     = (int(round(alt * 100)), 100)  # meters w/ 2 decimals
                gps_ifd[piexif.GPSIFD.GPSMapDatum]     = b"WGS-84"
                _write_exif(jpg_path, exif)

                # CSV row (formatted for readability)
                w.writerow([fname, int(ts), f"{lat:.9f}", f"{lon:.9f}", f"{alt:.3f}"])
                saved += 1

    print(f"[INFO] Done. Frames seen: {seen}, saved: {saved}")
    print(f"[INFO] Output dir: {outdir.resolve()}")
    print(f"[INFO] CSV: {csv_path.resolve()}")

if __name__ == "__main__":
    main()
