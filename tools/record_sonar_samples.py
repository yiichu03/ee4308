#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SonarRecorder(Node):
    def __init__(self, output_path: Path, sample_count: int, topic_name: str) -> None:
        super().__init__("lab2_sonar_recorder")
        self.output_path = output_path
        self.sample_count = sample_count
        self.rows = []
        self.start_time = None
        self.subscription = self.create_subscription(
            LaserScan, topic_name, self.callback, 10
        )

    def callback(self, msg: LaserScan) -> None:
        if not msg.ranges:
            return

        value = float(msg.ranges[0])
        if not math.isfinite(value):
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = stamp

        rel_time = stamp - self.start_time
        self.rows.append((len(self.rows), rel_time, value))

        if len(self.rows) >= self.sample_count:
            self.write_csv()
            self.get_logger().info(
                f"Wrote {len(self.rows)} samples to {self.output_path}"
            )
            rclpy.shutdown()

    def write_csv(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["index", "time_sec", "sonar_range_m"])
            writer.writerows(self.rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--samples", type=int, default=120)
    parser.add_argument("--topic", default="/drone/sonar")
    args = parser.parse_args()

    rclpy.init()
    node = SonarRecorder(Path(args.output), args.samples, args.topic)
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
