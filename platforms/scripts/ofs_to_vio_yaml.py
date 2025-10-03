#!/usr/bin/env python3
import json, sys, math, pathlib
r = json.load(open(pathlib.Path(sys.argv[1]) / "opensfm" / "reconstruction.json"))
cam = next(iter(r[0]["cameras"].values()))
w, h = cam["width"], cam["height"]
m = max(w, h)
fx = cam.get("focal_x", cam.get("focal", 0.0)) * m
fy = cam.get("focal_y", cam.get("focal", 0.0)) * m
cx = cam["c_x"] * w
cy = cam["c_y"] * h
# Distortion (map as available)
k1 = cam.get("k1", 0.0); k2 = cam.get("k2", 0.0)
r1 = cam.get("p1", 0.0); r2 = cam.get("p2", 0.0)  # tangential
model = cam.get("projection_type", "brown")

print("# Derived from OpenSfM")
print(f"cam0_model: {'pinhole-equi' if 'fisheye' in model else 'pinhole-radtan'}")
print(f"cam0_intrinsics: [{fx:.6f}, {fy:.6f}, {cx:.6f}, {cy:.6f}]  # [fx, fy, cx, cy]")
print(f"cam0_distortion: [{k1:.8f}, {k2:.8f}, {r1:.8f}, {r2:.8f}]  # [k1, k2, r1, r2]")
print(f"cam0_resolution: [{w}, {h}]  # [width, height]")
