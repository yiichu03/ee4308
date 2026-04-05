#!/usr/bin/env python3

import argparse
import subprocess
import sys
from datetime import datetime
from pathlib import Path


DEFAULT_CASES = [
    "current_xy:",
    "best_so_far:var_gps_x=0.4,var_gps_y=0.4,var_imu_x=3.0,var_imu_y=3.0",

    "refine_gps_0p35__imu_2p5:var_gps_x=0.35,var_gps_y=0.35,var_imu_x=2.5,var_imu_y=2.5",
    "refine_gps_0p35__imu_3p0:var_gps_x=0.35,var_gps_y=0.35,var_imu_x=3.0,var_imu_y=3.0",
    "refine_gps_0p35__imu_3p5:var_gps_x=0.35,var_gps_y=0.35,var_imu_x=3.5,var_imu_y=3.5",

    "refine_gps_0p40__imu_2p5:var_gps_x=0.40,var_gps_y=0.40,var_imu_x=2.5,var_imu_y=2.5",
    "refine_gps_0p40__imu_3p0:var_gps_x=0.40,var_gps_y=0.40,var_imu_x=3.0,var_imu_y=3.0",
    "refine_gps_0p40__imu_3p5:var_gps_x=0.40,var_gps_y=0.40,var_imu_x=3.5,var_imu_y=3.5",

    "refine_gps_0p45__imu_2p5:var_gps_x=0.45,var_gps_y=0.45,var_imu_x=2.5,var_imu_y=2.5",
    "refine_gps_0p45__imu_3p0:var_gps_x=0.45,var_gps_y=0.45,var_imu_x=3.0,var_imu_y=3.0",
    "refine_gps_0p45__imu_3p5:var_gps_x=0.45,var_gps_y=0.45,var_imu_x=3.5,var_imu_y=3.5",
]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run a recommended proj2 x/y sweep to diagnose and reduce planar lag."
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=40,
        help="Requested aligned-data duration per case in seconds.",
    )
    parser.add_argument(
        "--output-root",
        default="",
        help="Output directory. Defaults to tmp/proj2_param_sweeps/xy_<timestamp>.",
    )
    parser.add_argument(
        "--build",
        action="store_true",
        help="Run colcon build once before the sweep.",
    )
    parser.add_argument(
        "--case",
        action="append",
        default=[],
        help="Optional extra case in the same format as run_proj2_param_sweep.py: name:key=value,key=value",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = (
        Path(args.output_root).expanduser().resolve()
        if args.output_root
        else (repo_root / "tmp" / "proj2_param_sweeps" / f"xy_{timestamp}").resolve()
    )

    command = [
        sys.executable,
        str((repo_root / "tools" / "run_proj2_param_sweep.py").resolve()),
        "--no-default-cases",
        "--duration",
        str(args.duration),
        "--output-root",
        str(output_root),
    ]
    if args.build:
        command.append("--build")

    for case in DEFAULT_CASES:
        command.extend(["--case", case])
    for case in args.case:
        command.extend(["--case", case])

    subprocess.run(command, cwd=repo_root, check=True)


if __name__ == "__main__":
    main()
