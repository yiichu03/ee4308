#!/usr/bin/env python3

import argparse
import re
import shlex
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List

from summarize_proj2_logs import format_summary, ranking_score, summarize_log_file, write_csv_summary


@dataclass
class SweepCase:
    name: str
    overrides: Dict[str, float]


DEFAULT_CASES = [
    SweepCase("current", {}),
    SweepCase("baro_0p5", {"var_baro": 0.5}),
    SweepCase("baro_1p5", {"var_baro": 1.5}),
    SweepCase("gps_z_0p3", {"var_gps_z": 0.3}),
    SweepCase("gps_z_0p7", {"var_gps_z": 0.7}),
    SweepCase("imu_z_10", {"var_imu_z": 10.0}),
    SweepCase("imu_z_30", {"var_imu_z": 30.0}),
]


def parse_case(text: str) -> SweepCase:
    if ":" not in text:
        raise SystemExit(f"Invalid --case format: {text}")
    name, config = text.split(":", 1)
    overrides: Dict[str, float] = {}
    for pair in config.split(","):
        if not pair:
            continue
        if "=" not in pair:
            raise SystemExit(f"Invalid key=value pair in --case: {pair}")
        key, value = pair.split("=", 1)
        overrides[key.strip()] = float(value.strip())
    return SweepCase(name=name.strip(), overrides=overrides)


def format_value(value: float) -> str:
    if float(value).is_integer():
        return f"{int(value)}.0"
    return f"{value}"


def patch_proj2_yaml(text: str, overrides: Dict[str, float]) -> str:
    patched = text
    for key, value in overrides.items():
        pattern = re.compile(rf"^(\s*{re.escape(key)}:\s*)([^#\n]*?)(\s*(#.*)?)$", re.MULTILINE)
        replacement = rf"\g<1>{format_value(value)}\g<3>"
        patched, count = pattern.subn(replacement, patched, count=1)
        if count != 1:
            raise SystemExit(f"Parameter not found in proj2.yaml: {key}")
    return patched


def run_shell(command: str, cwd: Path) -> None:
    result = subprocess.run(["bash", "-lc", command], cwd=cwd)
    if result.returncode != 0:
        raise SystemExit(f"Command failed with code {result.returncode}: {command}")


def start_background_process(command: str, cwd: Path) -> subprocess.Popen:
    return subprocess.Popen(["bash", "-lc", command], cwd=cwd)


def aligned_csv_has_samples(csv_path: Path) -> bool:
    if not csv_path.exists() or csv_path.stat().st_size <= 0:
        return False
    with csv_path.open() as file:
        line_count = sum(1 for _ in file)
    return line_count >= 2


def build_once(repo_root: Path) -> None:
    run_shell(
        "source /opt/ros/jazzy/setup.bash && "
        "cd /ws/ee4308 && "
        "colcon build --symlink-install --packages-select ee4308_drone ee4308_bringup",
        repo_root,
    )


def kill_sim_processes(repo_root: Path) -> None:
    run_shell(
        "pkill -9 -f \"[p]roj2_sim.launch.py\" || true && "
        "pkill -9 -f \"[r]viz2\" || true && "
        "pkill -9 -f \"[c]omponent_container\" || true && "
        "pkill -9 -f \"[g]z sim\" || true",
        repo_root,
    )


def run_case(repo_root: Path, duration_sec: int, case: SweepCase, case_dir: Path) -> Dict[str, float]:
    log_path = case_dir / "run.log"
    recorder_log_path = case_dir / "alignment_recorder.log"
    plot_log_path = case_dir / "plot.log"
    aligned_csv_path = case_dir / "aligned_pose_error.csv"
    plots_dir = case_dir / "plots"
    overrides_path = case_dir / "overrides.txt"
    overrides_path.write_text(
        "\n".join(f"{key}={value}" for key, value in sorted(case.overrides.items())) + "\n"
    )

    startup_margin_sec = 8
    recorder = start_background_process(
        "source /opt/ros/jazzy/setup.bash && "
        "cd /ws/ee4308 && "
        "source install/setup.bash && "
        "python3 tools/record_drone_alignment.py "
        f"--output {shlex.quote(str(aligned_csv_path))} "
        f"--duration {duration_sec} "
        f">{shlex.quote(str(recorder_log_path))} 2>&1",
        repo_root,
    )
    time.sleep(1.0)

    try:
        run_shell(
            "source /opt/ros/jazzy/setup.bash && "
            "cd /ws/ee4308 && "
            "source install/setup.bash && "
            f"timeout {duration_sec + startup_margin_sec}s stdbuf -oL -eL "
            "ros2 launch ee4308_bringup proj2_sim.launch.py headless:=True libgl:=True param_file:=proj2 "
            f">{shlex.quote(str(log_path))} 2>&1 || true",
            repo_root,
        )
    finally:
        try:
            recorder.wait(timeout=20)
        except subprocess.TimeoutExpired:
            recorder.terminate()
            try:
                recorder.wait(timeout=5)
            except subprocess.TimeoutExpired:
                recorder.kill()
                recorder.wait(timeout=5)

    summary = summarize_log_file(log_path)
    if aligned_csv_has_samples(aligned_csv_path):
        run_shell(
            "source /opt/ros/jazzy/setup.bash && "
            "cd /ws/ee4308 && "
            f"python3 tools/plot_drone_bag.py {shlex.quote(str(aligned_csv_path))} "
            f"--output-dir {shlex.quote(str(plots_dir))} "
            f">{shlex.quote(str(plot_log_path))} 2>&1",
            repo_root,
        )
    (case_dir / "summary.txt").write_text(format_summary([summary]))
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run a batch of proj2 parameter cases and summarize their logs plus aligned estimator CSVs."
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=40,
        help="Requested aligned-data duration per case in seconds. The script adds startup margin internally.",
    )
    parser.add_argument(
        "--output-root",
        default="",
        help="Output directory. Defaults to tmp/proj2_param_sweeps/<timestamp>.",
    )
    parser.add_argument(
        "--case",
        action="append",
        default=[],
        help="Additional case in the form name:key=value,key=value",
    )
    parser.add_argument(
        "--no-default-cases",
        action="store_true",
        help="Do not run the built-in default cases.",
    )
    parser.add_argument(
        "--build",
        action="store_true",
        help="Run colcon build once before the sweep.",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    proj2_yaml = repo_root / "src/ee4308_bringup/params/proj2.yaml"
    original_yaml = proj2_yaml.read_text()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = (
        Path(args.output_root).expanduser().resolve()
        if args.output_root
        else (repo_root / "tmp" / "proj2_param_sweeps" / timestamp).resolve()
    )
    output_root.mkdir(parents=True, exist_ok=True)

    cases = [] if args.no_default_cases else list(DEFAULT_CASES)
    cases.extend(parse_case(case_text) for case_text in args.case)
    if not cases:
        raise SystemExit("No sweep cases specified.")

    if args.build:
        build_once(repo_root)

    all_summaries: List[Dict[str, float]] = []
    try:
        for case in cases:
            case_dir = output_root / case.name
            case_dir.mkdir(parents=True, exist_ok=True)

            patched_yaml = patch_proj2_yaml(original_yaml, case.overrides)
            proj2_yaml.write_text(patched_yaml)
            (case_dir / "proj2_used.yaml").write_text(patched_yaml)

            kill_sim_processes(repo_root)
            summary = run_case(repo_root, args.duration, case, case_dir)
            summary["case_name"] = case.name
            all_summaries.append(summary)
            kill_sim_processes(repo_root)
    finally:
        proj2_yaml.write_text(original_yaml)

    all_summaries.sort(key=ranking_score)
    for row in all_summaries:
        row["log_path"] = str(Path(row["log_path"]).resolve())

    summary_text = format_summary(all_summaries)
    print(summary_text, end="")

    write_csv_summary(output_root / "summary.csv", all_summaries)
    (output_root / "summary.txt").write_text(summary_text)
    print(f"Wrote sweep results to {output_root}")


if __name__ == "__main__":
    main()
