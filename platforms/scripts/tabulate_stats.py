#!/usr/bin/env python3
"""
Tabulate ODM/OpenSfM-style stats.json files into a single table.

Usage:
  python tabulate_stats.py stats1.json stats2.json ...
  python tabulate_stats.py --csv out.csv stats/*.json
"""

import argparse
import json
import os
from datetime import datetime
from typing import Any, Dict, List, Tuple

import pandas as pd


def _safe_get(d: Dict[str, Any], path: List[str], default=None):
    cur = d
    for key in path:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def _first_existing(d: Dict[str, Any], paths: List[List[str]], default=None):
    for p in paths:
        v = _safe_get(d, p, None)
        if v is not None:
            return v
    return default


def _parse_date(s: Any) -> Any:
    """Return a normalized ISO8601 date string if input looks like 'MM/DD/YYYY at HH:MM:SS' else return as-is."""
    if not isinstance(s, str):
        return s
    try:
        # Example: "10/09/2025 at 03:31:48"
        s2 = s.replace(" at ", " ")
        dt = datetime.strptime(s2, "%m/%d/%Y %H:%M:%S")
        return dt.isoformat(sep=" ")
    except Exception:
        return s


def extract_metrics(stats: Dict[str, Any]) -> Dict[str, Any]:
    # date
    date = _first_existing(
        stats,
        [
            ["processing_statistics", "date"],
            ["point_cloud_statistics", "now"],
        ],
        default=None,
    )
    date = _parse_date(date)

    # processing time (seconds)
    proc_time_sec = _first_existing(
        stats,
        [
            ["processing_statistics", "steps_times", "Total Time"],
            ["odm_processing_statistics", "total_time"],  # alt location
        ],
        default=None,
    )

    # area covered
    area = _first_existing(
        stats,
        [
            ["processing_statistics", "area"],
            ["odm_processing_statistics", "average_gsd"],  # fallback, though not area
        ],
        default=None,
    )

    # number of images used for final reconstruction
    images_final = _first_existing(
        stats,
        [
            ["reconstruction_statistics", "reconstructed_shots_count"],
            ["reconstruction_statistics", "initial_shots_count"],  # fallback
        ],
        default=None,
    )

    # detected features (use mean)
    detected_features_mean = _first_existing(
        stats, [["features_statistics", "detected_features", "mean"]], default=None
    )

    # reconstructed features (use mean)
    reconstructed_features_mean = _first_existing(
        stats, [["features_statistics", "reconstructed_features", "mean"]], default=None
    )

    # whether GPS data available
    has_gps = _first_existing(
        stats, [["reconstruction_statistics", "has_gps"]], default=None
    )

    # Bundle Adjustment reprojection errors
    ba_reproj_err_nrm = _first_existing(
        stats, [["reconstruction_statistics", "reprojection_error_normalized"]], default=None
    )
    ba_reproj_err_pxl = _first_existing(
        stats, [["reconstruction_statistics", "reprojection_error_pixels"]], default=None
    )
    ba_reproj_err_ang = _first_existing(
        stats, [["reconstruction_statistics", "reprojection_error_angular"]], default=None
    )

    # 3D errors: mean, std per axis
    # In many stats.json variants, "camera_errors" -> "3d_errors" holds these triplets.
    mean_x = _first_existing(stats, [["3d_errors", "mean", "x"]])
    mean_y = _first_existing(stats, [["3d_errors", "mean", "y"]])
    mean_z = _first_existing(stats, [["camera_errors", "3d_errors", "mean", "z"]])

    std_x = _first_existing(stats, [["3d_errors", "std", "x"]])
    std_y = _first_existing(stats, [["3d_errors", "std", "y"]])
    std_z = _first_existing(stats, [["3d_errors", "std", "z"]])

    return {
        "Date": date,
        "Processing Time (s)": proc_time_sec,
        "Area Covered": area,
        "Images (final)": images_final,
        "Detected Features (mean)": detected_features_mean,
        "Reconstructed Features (mean)": reconstructed_features_mean,
        "GPS Available": has_gps,
        "Reprojection Errors Normalized": ba_reproj_err_nrm,
        "Reprojection Errors Pixels": ba_reproj_err_pxl,
        "Reprojection Errors Angular": ba_reproj_err_ang,
        "3D Mean X": mean_x,
        "3D Mean Y": mean_y,
        "3D Mean Z": mean_z,
        "3D Std X": std_x,
        "3D Std Y": std_y,
        "3D Std Z": std_z,
    }


def build_table(paths: List[str]) -> pd.DataFrame:
    cols: List[str] = []
    rows: Dict[str, List[Any]] = {}
    for p in paths:
        with open(p, "r") as f:
            stats = json.load(f)
        metrics = extract_metrics(stats)
        col_name = os.path.splitext(os.path.basename(p))[0]
        cols.append(col_name)
        for k, v in metrics.items():
            rows.setdefault(k, []).append(v)

    # Normalize rows (fill missing with None)
    max_len = len(cols)
    for k in rows:
        if len(rows[k]) < max_len:
            rows[k] += [None] * (max_len - len(rows[k]))

    df = pd.DataFrame(rows, index=cols).T
    return df


def main():
    parser = argparse.ArgumentParser(description="Tabulate stats.json files.")
    parser.add_argument("paths", nargs="+", help="One or more stats.json files.")
    parser.add_argument("--csv", help="Optional path to write CSV.", default=None)
    args = parser.parse_args()

    df = build_table(args.paths)
    # Print as a clean text table
    with pd.option_context("display.max_rows", None, "display.max_columns", None):
        print(df.to_string())

    if args.csv:
        df.to_csv(args.csv, index=True)
        print(f"\nWrote CSV to: {args.csv}")


if __name__ == "__main__":
    main()
