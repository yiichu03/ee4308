#!/usr/bin/env python3

import argparse
import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence


ERRPOSE_RE = re.compile(
    r"ErrPose\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)"
)
TIME_RE = re.compile(r"\]\s+([-\d.]+)\s+ErrPose")
TMP_ERR_RE = re.compile(
    r"TMP LOG err_xyz=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\) \| "
    r"z_est=([-\d.]+) vz_est=([-\d.]+) bias=([-\d.]+) \| "
    r"gps_z=([-\d.inf]+) sonar=([-\d.inf]+) baro=([-\d.inf]+)"
)


@dataclass
class ErrPoseSample:
    time_sec: float
    err_x: float
    err_y: float
    err_z: float
    err_yaw: float


def parse_float(text: str) -> float:
    lowered = text.lower()
    if lowered == "inf":
        return math.inf
    if lowered == "-inf":
        return -math.inf
    return float(text)


def parse_log(log_path: Path) -> Dict[str, List[Dict[str, float]]]:
    errpose_samples: List[ErrPoseSample] = []
    tmp_samples: List[Dict[str, float]] = []

    for line in log_path.read_text(errors="replace").splitlines():
        time_match = TIME_RE.search(line)
        err_match = ERRPOSE_RE.search(line)
        if time_match and err_match:
            errpose_samples.append(
                ErrPoseSample(
                    time_sec=float(time_match.group(1)),
                    err_x=float(err_match.group(1)),
                    err_y=float(err_match.group(2)),
                    err_z=float(err_match.group(3)),
                    err_yaw=float(err_match.group(4)),
                )
            )
            continue

        tmp_match = TMP_ERR_RE.search(line)
        if tmp_match:
            tmp_samples.append(
                {
                    "err_x": parse_float(tmp_match.group(1)),
                    "err_y": parse_float(tmp_match.group(2)),
                    "err_z": parse_float(tmp_match.group(3)),
                    "z_est": parse_float(tmp_match.group(4)),
                    "vz_est": parse_float(tmp_match.group(5)),
                    "bias": parse_float(tmp_match.group(6)),
                    "gps_z": parse_float(tmp_match.group(7)),
                    "sonar": parse_float(tmp_match.group(8)),
                    "baro": parse_float(tmp_match.group(9)),
                }
            )

    return {
        "errpose": [sample.__dict__ for sample in errpose_samples],
        "tmp": tmp_samples,
    }


def mean_abs(values: Sequence[float]) -> float:
    return sum(abs(value) for value in values) / len(values) if values else math.nan


def rmse(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else math.nan


def select_window(samples: Sequence[Dict[str, float]], start: float, end: float) -> List[Dict[str, float]]:
    return [sample for sample in samples if start <= sample["time_sec"] <= end]


def summarize_window(samples: Sequence[Dict[str, float]], prefix: str) -> Dict[str, float]:
    result: Dict[str, float] = {f"{prefix}_samples": float(len(samples))}
    if not samples:
        for axis in ("x", "y", "z", "yaw"):
            result[f"{prefix}_mae_{axis}"] = math.nan
            result[f"{prefix}_rmse_{axis}"] = math.nan
        return result

    for axis in ("x", "y", "z", "yaw"):
        values = [sample[f"err_{axis}"] for sample in samples]
        result[f"{prefix}_mae_{axis}"] = mean_abs(values)
        result[f"{prefix}_rmse_{axis}"] = rmse(values)
    return result


def summarize_tmp(samples: Sequence[Dict[str, float]]) -> Dict[str, float]:
    result: Dict[str, float] = {
        "tmp_samples": float(len(samples)),
        "tmp_mean_abs_err_z": math.nan,
        "tmp_mean_bias": math.nan,
        "tmp_sonar_inf_ratio": math.nan,
    }
    if not samples:
        return result

    err_z = [sample["err_z"] for sample in samples]
    biases = [sample["bias"] for sample in samples]
    sonar_inf = [sample for sample in samples if not math.isfinite(sample["sonar"])]

    result["tmp_mean_abs_err_z"] = mean_abs(err_z)
    result["tmp_mean_bias"] = sum(biases) / len(biases)
    result["tmp_sonar_inf_ratio"] = len(sonar_inf) / len(samples)
    return result


def load_aligned_csv(csv_path: Path) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with csv_path.open() as file:
        reader = csv.DictReader(file)
        required_columns = {
            "time_sec",
            "err_x",
            "err_y",
            "err_z",
            "err_yaw",
        }
        missing = required_columns.difference(reader.fieldnames or [])
        if missing:
            raise SystemExit(f"Aligned CSV is missing columns: {', '.join(sorted(missing))}")

        for row in reader:
            rows.append(
                {
                    "time_sec": float(row["time_sec"]),
                    "err_x": float(row["err_x"]),
                    "err_y": float(row["err_y"]),
                    "err_z": float(row["err_z"]),
                    "err_yaw": float(row["err_yaw"]),
                }
            )
    return rows


def summarize_aligned_csv(csv_path: Path) -> Dict[str, float]:
    rows = load_aligned_csv(csv_path)
    summary: Dict[str, float] = {
        "aligned_csv_path": str(csv_path),
        "aligned_samples": float(len(rows)),
    }
    summary.update(summarize_window(rows, "aligned_all"))
    summary.update(summarize_window(select_window(rows, 7.0, 25.0), "aligned_w7_25"))
    summary.update(summarize_window(select_window(rows, 10.0, 18.0), "aligned_w10_18"))

    score_terms = [
        summary.get("aligned_w10_18_mae_x", math.nan),
        summary.get("aligned_w10_18_mae_y", math.nan),
        summary.get("aligned_w10_18_mae_z", math.nan),
        summary.get("aligned_w7_25_mae_z", math.nan),
    ]
    finite_terms = [term for term in score_terms if math.isfinite(term)]
    summary["aligned_score"] = sum(finite_terms) if finite_terms else math.nan
    return summary


def ranking_score(row: Dict[str, float]) -> float:
    aligned_score = row.get("aligned_score", math.nan)
    if math.isfinite(aligned_score):
        return aligned_score
    log_score = row.get("score", math.nan)
    if math.isfinite(log_score):
        return log_score
    return math.inf


def summarize_log_file(log_path: Path) -> Dict[str, float]:
    parsed = parse_log(log_path)
    errpose = parsed["errpose"]
    summary: Dict[str, float] = {
        "log_path": str(log_path),
    }
    summary.update(summarize_window(errpose, "all"))
    summary.update(summarize_window(select_window(errpose, 7.0, 25.0), "w7_25"))
    summary.update(summarize_window(select_window(errpose, 10.0, 18.0), "w10_18"))
    summary.update(summarize_tmp(parsed["tmp"]))

    # Lower score is better. Weight the main demo windows more heavily than global metrics.
    score_terms = [
        summary.get("w10_18_mae_x", math.nan),
        summary.get("w10_18_mae_y", math.nan),
        summary.get("w10_18_mae_z", math.nan),
        summary.get("w7_25_mae_z", math.nan),
    ]
    finite_terms = [term for term in score_terms if math.isfinite(term)]
    summary["score"] = sum(finite_terms) if finite_terms else math.nan

    aligned_csv_path = log_path.parent / "aligned_pose_error.csv"
    if aligned_csv_path.exists():
        summary.update(summarize_aligned_csv(aligned_csv_path))
    return summary


def collect_logs(inputs: Iterable[str]) -> List[Path]:
    logs: List[Path] = []
    for item in inputs:
        path = Path(item).expanduser().resolve()
        if path.is_file():
            logs.append(path)
        elif path.is_dir():
            logs.extend(sorted(path.rglob("*.log")))
        else:
            raise SystemExit(f"Input path not found: {path}")
    unique_logs = []
    seen = set()
    for log_path in logs:
        if log_path not in seen:
            unique_logs.append(log_path)
            seen.add(log_path)
    if not unique_logs:
        raise SystemExit("No .log files found.")
    return unique_logs


def write_csv_summary(path: Path, rows: Sequence[Dict[str, float]]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def format_summary(rows: Sequence[Dict[str, float]]) -> str:
    lines = []
    for index, row in enumerate(rows, start=1):
        display_name = row.get("case_name", "") or Path(row["log_path"]).name
        if math.isfinite(row.get("aligned_score", math.nan)):
            lines.append(
                f"{index}. {display_name} | "
                f"aligned_score={row['aligned_score']:.3f} | "
                f"aligned w10_18 mae(x,y,z)=({row['aligned_w10_18_mae_x']:.3f}, "
                f"{row['aligned_w10_18_mae_y']:.3f}, {row['aligned_w10_18_mae_z']:.3f}) | "
                f"aligned w7_25 mae_z={row['aligned_w7_25_mae_z']:.3f}"
            )
            continue

        lines.append(
            f"{index}. {display_name} | "
            f"score={row['score']:.3f} | "
            f"w10_18 mae(x,y,z)=({row['w10_18_mae_x']:.3f}, {row['w10_18_mae_y']:.3f}, {row['w10_18_mae_z']:.3f}) | "
            f"w7_25 mae_z={row['w7_25_mae_z']:.3f}"
        )
    return "\n".join(lines) + ("\n" if lines else "")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Summarize proj2 estimator logs and rank cases."
    )
    parser.add_argument("paths", nargs="+", help="Log files or directories containing .log files.")
    parser.add_argument("--csv", default="", help="Optional CSV output path.")
    parser.add_argument("--summary", default="", help="Optional text summary output path.")
    args = parser.parse_args()

    rows = [summarize_log_file(log_path) for log_path in collect_logs(args.paths)]
    rows.sort(key=ranking_score)

    text = format_summary(rows)
    print(text, end="")

    if args.csv:
        write_csv_summary(Path(args.csv), rows)
    if args.summary:
        summary_path = Path(args.summary)
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(text)


if __name__ == "__main__":
    main()
