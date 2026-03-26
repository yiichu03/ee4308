#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path


def fit_line(xs, ys):
    n = len(xs)
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    sxx = sum((x - mean_x) ** 2 for x in xs)
    if sxx == 0.0:
        slope = 0.0
    else:
        sxy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
        slope = sxy / sxx
    intercept = mean_y - slope * mean_x
    return slope, intercept


def sample_variance(values):
    n = len(values)
    if n < 2:
        return 0.0
    mean_v = sum(values) / n
    return sum((v - mean_v) ** 2 for v in values) / (n - 1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--summary", default="")
    parser.add_argument("--residual-csv", default="")
    args = parser.parse_args()

    csv_path = Path(args.csv_path)
    xs = []
    ys = []
    with csv_path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            xs.append(float(row["time_sec"]))
            ys.append(float(row["sonar_range_m"]))

    if len(xs) < 2:
        raise SystemExit("Need at least 2 valid samples.")

    slope, intercept = fit_line(xs, ys)
    fitted = [slope * x + intercept for x in xs]
    residuals = [y - yhat for y, yhat in zip(ys, fitted)]
    variance = sample_variance(residuals)
    stddev = variance ** 0.5

    lines = [
        f"input_csv: {csv_path}",
        f"samples: {len(xs)}",
        f"best_fit_slope_m_per_s: {slope:.9f}",
        f"best_fit_intercept_m: {intercept:.9f}",
        f"residual_mean_m: {sum(residuals) / len(residuals):.9f}",
        f"residual_stddev_m: {stddev:.9f}",
        f"residual_variance_m2: {variance:.9f}",
        "",
        "Interpretation:",
        "- Use residual_variance_m2 as an initial estimate for var_sonar.",
        "- Then run the simulator and tune around this value.",
    ]

    print("\n".join(lines))

    if args.summary:
        summary_path = Path(args.summary)
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text("\n".join(lines) + "\n")

    if args.residual_csv:
        residual_path = Path(args.residual_csv)
        residual_path.parent.mkdir(parents=True, exist_ok=True)
        with residual_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                ["index", "time_sec", "sonar_range_m", "fit_range_m", "residual_m"]
            )
            for i, (x, y, yhat, r) in enumerate(zip(xs, ys, fitted, residuals)):
                writer.writerow([i, x, y, yhat, r])


if __name__ == "__main__":
    main()
