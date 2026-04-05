#!/usr/bin/env python3

import argparse
import csv
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional


def start_process(
    cmd: list[str],
    cwd: Path,
    stdout=None,
    stderr=None,
) -> subprocess.Popen:
    return subprocess.Popen(
        cmd,
        cwd=str(cwd),
        stdout=stdout,
        stderr=stderr,
        preexec_fn=os.setsid,
    )


def stop_process(proc: Optional[subprocess.Popen], name: str, sigint_timeout: float = 8.0) -> None:
    if proc is None or proc.poll() is not None:
        return

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=sigint_timeout)
        return
    except subprocess.TimeoutExpired:
        pass
    except ProcessLookupError:
        return

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=5.0)
        return
    except subprocess.TimeoutExpired:
        pass
    except ProcessLookupError:
        return

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait(timeout=2.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        print(f"[run_proj2_full_check] Warning: failed to fully stop {name}.", file=sys.stderr)


def csv_has_samples(csv_path: Path) -> bool:
    if not csv_path.exists() or csv_path.stat().st_size == 0:
        return False

    with csv_path.open(newline="") as f:
        reader = csv.reader(f)
        next(reader, None)
        return next(reader, None) is not None


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run proj2 once while recording aligned estimate/ground-truth CSV, then generate plots."
    )
    parser.add_argument("--run-name", default="full_check_01")
    parser.add_argument("--output-dir", default=None, help="Defaults to tmp/proj2_runs/<run-name>.")
    parser.add_argument(
        "--duration",
        type=float,
        default=150.0,
        help=(
            "Recording duration in seconds. "
            "When positive, the script auto-stops the simulation after the recorder finishes. "
            "Use 0 or a negative value to keep recording until Ctrl+C."
        ),
    )
    parser.add_argument("--param-file", default="proj2")
    parser.add_argument("--record-plan", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--libgl", action="store_true")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    output_dir = Path(args.output_dir) if args.output_dir else repo_root / "tmp" / "proj2_runs" / args.run_name
    output_dir.mkdir(parents=True, exist_ok=True)

    aligned_csv = output_dir / "aligned_pose_error.csv"
    plots_dir = output_dir / "plots"
    drone_plan_csv = output_dir / "drone_plan.csv"

    recorder_cmd = [
        sys.executable,
        str((repo_root / "tools" / "record_drone_alignment.py").resolve()),
        "--output",
        str(aligned_csv),
        "--duration",
        str(args.duration),
    ]

    launch_cmd = [
        "ros2",
        "launch",
        "ee4308_bringup",
        "proj2_sim.launch.py",
        f"param_file:={args.param_file}",
    ]
    if args.headless:
        launch_cmd.append("headless:=True")
    if args.libgl:
        launch_cmd.append("libgl:=True")

    print(f"[run_proj2_full_check] Output directory: {output_dir}")
    print(f"[run_proj2_full_check] Starting recorder: {' '.join(recorder_cmd)}")
    recorder_proc = start_process(recorder_cmd, repo_root)

    plan_proc = None
    plan_log_handle = None
    if args.record_plan:
        plan_cmd = [
            sys.executable,
            str((repo_root / "tools" / "record_drone_plan.py").resolve()),
            "--output",
            str(drone_plan_csv),
            "--duration",
            str(args.duration),
        ]
        print(f"[run_proj2_full_check] Recording /drone/plan to {drone_plan_csv}")
        plan_proc = start_process(plan_cmd, repo_root)

    launch_proc = None
    exit_code = 0
    try:
        print(f"[run_proj2_full_check] Launching simulation: {' '.join(launch_cmd)}")
        if args.duration > 0.0:
            print(
                "[run_proj2_full_check] Observe Gazebo/RViz. "
                f"The run will auto-stop after about {args.duration:.0f}s, "
                "or you can press Ctrl+C earlier after the drone fully lands."
            )
        else:
            print("[run_proj2_full_check] Observe Gazebo/RViz. Press Ctrl+C after the drone fully lands.")
        launch_proc = start_process(launch_cmd, repo_root)
        while True:
            launch_return = launch_proc.poll()
            if launch_return is not None:
                exit_code = launch_return
                break

            if args.duration > 0.0:
                recorder_return = recorder_proc.poll()
                if recorder_return is not None:
                    print(
                        "[run_proj2_full_check] Recorder finished requested duration. "
                        "Stopping simulation and finalizing outputs..."
                    )
                    break

            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[run_proj2_full_check] Ctrl+C received. Cleaning up processes and generating plots...")
    finally:
        stop_process(launch_proc, "proj2 launch")
        stop_process(plan_proc, "/drone/plan recorder")
        stop_process(recorder_proc, "alignment recorder")

    if csv_has_samples(aligned_csv):
        plot_cmd = [
            sys.executable,
            str((repo_root / "tools" / "plot_drone_bag.py").resolve()),
            str(aligned_csv),
            "--output-dir",
            str(plots_dir),
        ]
        print(f"[run_proj2_full_check] Generating plots: {' '.join(plot_cmd)}")
        plot_result = subprocess.run(plot_cmd, cwd=str(repo_root))
        if plot_result.returncode != 0:
            print("[run_proj2_full_check] Plot generation failed.", file=sys.stderr)
            exit_code = plot_result.returncode if exit_code == 0 else exit_code
        else:
            print(f"[run_proj2_full_check] Plots written to {plots_dir}")
    else:
        print(
            f"[run_proj2_full_check] No aligned samples were recorded at {aligned_csv}. "
            "Skipping plot generation.",
            file=sys.stderr,
        )

    if args.record_plan:
        print(f"[run_proj2_full_check] /drone/plan CSV written to {drone_plan_csv}")

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
