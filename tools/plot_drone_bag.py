#!/usr/bin/env python3

import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


@dataclass
class OdomSample:
    stamp_sec: float
    x: float
    y: float
    z: float
    yaw: float
    vx: float
    vy: float
    vz: float
    wz: float


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def unwrap_angles(angles: Sequence[float]) -> List[float]:
    if not angles:
        return []

    unwrapped = [angles[0]]
    for angle in angles[1:]:
        prev = unwrapped[-1]
        delta = wrap_to_pi(angle - wrap_to_pi(prev))
        unwrapped.append(prev + delta)
    return unwrapped


def detect_storage_id(bag_path: Path) -> str:
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.exists():
        return "sqlite3"

    for line in metadata_path.read_text().splitlines():
        stripped = line.strip()
        if stripped.startswith("storage_identifier:"):
            _, value = stripped.split(":", 1)
            value = value.strip()
            if value:
                return value
    return "sqlite3"


def load_odom_series(bag_path: Path, topics: Iterable[str]) -> Dict[str, List[OdomSample]]:
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    storage_id = detect_storage_id(bag_path)
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id=storage_id),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    topic_types = {
        topic_metadata.name: topic_metadata.type
        for topic_metadata in reader.get_all_topics_and_types()
    }
    selected = set(topics)
    missing = [topic for topic in selected if topic not in topic_types]
    if missing:
        raise SystemExit(f"Missing topic(s) in bag: {', '.join(sorted(missing))}")

    messages: Dict[str, List[OdomSample]] = {topic: [] for topic in selected}
    type_cache = {topic: get_message(topic_types[topic]) for topic in selected}

    while reader.has_next():
        topic, data, bag_stamp_ns = reader.read_next()
        if topic not in selected:
            continue

        msg = deserialize_message(data, type_cache[topic])
        header_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        stamp_ns = header_stamp_ns if header_stamp_ns > 0 else bag_stamp_ns
        orientation = msg.pose.pose.orientation
        twist = msg.twist.twist
        messages[topic].append(
            OdomSample(
                stamp_sec=stamp_ns * 1e-9,
                x=float(msg.pose.pose.position.x),
                y=float(msg.pose.pose.position.y),
                z=float(msg.pose.pose.position.z),
                yaw=quaternion_to_yaw(
                    float(orientation.x),
                    float(orientation.y),
                    float(orientation.z),
                    float(orientation.w),
                ),
                vx=float(twist.linear.x),
                vy=float(twist.linear.y),
                vz=float(twist.linear.z),
                wz=float(twist.angular.z),
            )
        )

    for topic, series in messages.items():
        if not series:
            raise SystemExit(f"No messages found for topic: {topic}")
        series.sort(key=lambda sample: sample.stamp_sec)

    return messages


def load_aligned_rows_from_csv(csv_path: Path) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with csv_path.open() as file:
        reader = csv.DictReader(file)
        required_columns = {
            "time_sec",
            "est_x",
            "est_y",
            "est_z",
            "est_yaw",
            "true_x",
            "true_y",
            "true_z",
            "true_yaw",
            "err_x",
            "err_y",
            "err_z",
            "err_yaw",
        }
        missing = required_columns.difference(reader.fieldnames or [])
        if missing:
            raise SystemExit(f"Aligned CSV is missing columns: {', '.join(sorted(missing))}")

        for row in reader:
            rows.append({key: float(value) for key, value in row.items()})

    if not rows:
        raise SystemExit(f"No rows found in CSV: {csv_path}")
    return rows


def interpolate_scalar(times: Sequence[float], values: Sequence[float], t: float) -> float:
    if t <= times[0]:
        return values[0]
    if t >= times[-1]:
        return values[-1]

    upper = bisect.bisect_left(times, t)
    lower = upper - 1
    t0 = times[lower]
    t1 = times[upper]
    if abs(t1 - t0) < 1e-12:
        return values[lower]

    ratio = (t - t0) / (t1 - t0)
    return values[lower] + ratio * (values[upper] - values[lower])


def align_truth_to_estimate(
    est_series: Sequence[OdomSample],
    true_series: Sequence[OdomSample],
) -> List[Dict[str, float]]:
    true_times = [sample.stamp_sec for sample in true_series]
    true_x = [sample.x for sample in true_series]
    true_y = [sample.y for sample in true_series]
    true_z = [sample.z for sample in true_series]
    true_yaw_unwrapped = unwrap_angles([sample.yaw for sample in true_series])

    aligned_rows = []
    t0 = est_series[0].stamp_sec
    for est in est_series:
        aligned_true_x = interpolate_scalar(true_times, true_x, est.stamp_sec)
        aligned_true_y = interpolate_scalar(true_times, true_y, est.stamp_sec)
        aligned_true_z = interpolate_scalar(true_times, true_z, est.stamp_sec)
        aligned_true_yaw = interpolate_scalar(true_times, true_yaw_unwrapped, est.stamp_sec)
        est_yaw = wrap_to_pi(est.yaw)
        true_yaw = wrap_to_pi(aligned_true_yaw)

        aligned_rows.append(
            {
                "time_sec": est.stamp_sec - t0,
                "est_x": est.x,
                "est_y": est.y,
                "est_z": est.z,
                "est_yaw": est_yaw,
                "true_x": aligned_true_x,
                "true_y": aligned_true_y,
                "true_z": aligned_true_z,
                "true_yaw": true_yaw,
                "err_x": aligned_true_x - est.x,
                "err_y": aligned_true_y - est.y,
                "err_z": aligned_true_z - est.z,
                "err_yaw": wrap_to_pi(true_yaw - est_yaw),
            }
        )

    return aligned_rows


def mean_abs(values: Sequence[float]) -> float:
    return sum(abs(value) for value in values) / len(values) if values else 0.0


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else 0.0


def write_csv(path: Path, rows: Sequence[Dict[str, float]], columns: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(columns))
        writer.writeheader()
        for row in rows:
            writer.writerow({column: row[column] for column in columns})


def plot_3d_trajectory(path: Path, rows: Sequence[Dict[str, float]]) -> None:
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(
        [row["true_x"] for row in rows],
        [row["true_y"] for row in rows],
        [row["true_z"] for row in rows],
        label="Ground Truth",
        linewidth=2.0,
    )
    ax.plot(
        [row["est_x"] for row in rows],
        [row["est_y"] for row in rows],
        [row["est_z"] for row in rows],
        label="Estimate",
        linewidth=1.5,
    )
    ax.set_title("Drone 3D Trajectory")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_position_vs_time(path: Path, rows: Sequence[Dict[str, float]]) -> None:
    times = [row["time_sec"] for row in rows]
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for axis, name in zip(axes, ("x", "y", "z")):
        axis.plot(times, [row[f"true_{name}"] for row in rows], label="Ground Truth")
        axis.plot(times, [row[f"est_{name}"] for row in rows], label="Estimate")
        axis.set_ylabel(f"{name} (m)")
        axis.grid(True, linestyle="--", alpha=0.4)
    axes[0].set_title("Position vs Time")
    axes[0].legend()
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_error_vs_time(path: Path, rows: Sequence[Dict[str, float]]) -> None:
    times = [row["time_sec"] for row in rows]
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    for axis, name in zip(axes[:3], ("x", "y", "z")):
        axis.plot(times, [row[f"err_{name}"] for row in rows], color="tab:red")
        axis.set_ylabel(f"e_{name} (m)")
        axis.grid(True, linestyle="--", alpha=0.4)
    axes[3].plot(times, [row["err_yaw"] for row in rows], color="tab:purple")
    axes[3].set_ylabel("e_yaw (rad)")
    axes[3].set_xlabel("time (s)")
    axes[3].grid(True, linestyle="--", alpha=0.4)
    axes[0].set_title("Estimation Error vs Time")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def build_summary(rows: Sequence[Dict[str, float]], input_path: Path) -> str:
    err_x = [row["err_x"] for row in rows]
    err_y = [row["err_y"] for row in rows]
    err_z = [row["err_z"] for row in rows]
    err_yaw = [row["err_yaw"] for row in rows]

    lines = [
        f"input_path: {input_path}",
        f"samples: {len(rows)}",
        "",
        "Mean absolute error:",
        f"- x: {mean_abs(err_x):.6f} m",
        f"- y: {mean_abs(err_y):.6f} m",
        f"- z: {mean_abs(err_z):.6f} m",
        f"- yaw: {mean_abs(err_yaw):.6f} rad",
        "",
        "RMSE:",
        f"- x: {rmse(err_x):.6f} m",
        f"- y: {rmse(err_y):.6f} m",
        f"- z: {rmse(err_z):.6f} m",
        f"- yaw: {rmse(err_yaw):.6f} rad",
    ]
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot estimator vs ground truth from either a ROS 2 bag directory or an aligned CSV."
    )
    parser.add_argument("input_path", help="Path to a ROS 2 bag directory or an aligned CSV file.")
    parser.add_argument(
        "--output-dir",
        default="tmp/bag_plots",
        help="Directory to write plots and CSV summaries into.",
    )
    parser.add_argument("--odom-topic", default="/drone/odom")
    parser.add_argument("--true-odom-topic", default="/drone/true_odom")
    args = parser.parse_args()

    input_path = Path(args.input_path).expanduser().resolve()
    if not input_path.exists():
        raise SystemExit(f"Input path does not exist: {input_path}")

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if input_path.is_file():
        aligned = load_aligned_rows_from_csv(input_path)
    else:
        series = load_odom_series(input_path, (args.odom_topic, args.true_odom_topic))
        aligned = align_truth_to_estimate(
            est_series=series[args.odom_topic],
            true_series=series[args.true_odom_topic],
        )

    summary = build_summary(aligned, input_path)
    summary_path = output_dir / "summary.txt"
    summary_path.write_text(summary)
    print(summary, end="")

    write_csv(
        output_dir / "aligned_pose_error.csv",
        aligned,
        (
            "time_sec",
            "est_x",
            "est_y",
            "est_z",
            "est_yaw",
            "true_x",
            "true_y",
            "true_z",
            "true_yaw",
            "err_x",
            "err_y",
            "err_z",
            "err_yaw",
        ),
    )

    plot_3d_trajectory(output_dir / "trajectory_3d.png", aligned)
    plot_position_vs_time(output_dir / "position_vs_time.png", aligned)
    plot_error_vs_time(output_dir / "error_vs_time.png", aligned)

    print(f"Wrote plots and summaries to {output_dir}")


if __name__ == "__main__":
    main()
