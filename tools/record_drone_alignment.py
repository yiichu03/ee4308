#!/usr/bin/env python3

import argparse
import bisect
import csv
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


@dataclass
class OdomSample:
    stamp_sec: float
    x: float
    y: float
    z: float
    yaw: float


def quaternion_to_yaw(msg) -> float:
    siny_cosp = 2.0 * (msg.w * msg.z + msg.x * msg.y)
    cosy_cosp = 1.0 - 2.0 * (msg.y * msg.y + msg.z * msg.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class DroneAlignmentRecorder(Node):
    def __init__(self, output_path: Path, duration_sec: Optional[float], odom_topic: str, true_odom_topic: str) -> None:
        super().__init__("proj2_drone_alignment_recorder")
        self.output_path = output_path
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.output_path.open("w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(
            [
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
            ]
        )

        self.duration_sec = duration_sec
        self.finished = False
        self.start_time_est = None
        self.true_samples: Deque[OdomSample] = deque()
        self.pending_est_samples: Deque[OdomSample] = deque()

        self.create_subscription(Odometry, odom_topic, self.callback_est_odom, 10)
        self.create_subscription(Odometry, true_odom_topic, self.callback_true_odom, 10)
        # Safety timeout in case topics never start publishing.
        self.safety_timer = None
        if self.duration_sec is not None:
            self.safety_timer = self.create_timer(max(self.duration_sec + 20.0, 20.0), self.finish_due_to_timeout)

    def extract_sample(self, msg: Odometry) -> OdomSample:
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return OdomSample(
            stamp_sec=stamp_sec,
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            z=float(msg.pose.pose.position.z),
            yaw=quaternion_to_yaw(msg.pose.pose.orientation),
        )

    def callback_true_odom(self, msg: Odometry) -> None:
        self.true_samples.append(self.extract_sample(msg))
        self.flush_pending_estimates()
        self.trim_true_samples()

    def callback_est_odom(self, msg: Odometry) -> None:
        self.pending_est_samples.append(self.extract_sample(msg))
        self.flush_pending_estimates()

    def flush_pending_estimates(self) -> None:
        if not self.true_samples:
            return

        while self.pending_est_samples:
            est = self.pending_est_samples[0]
            if len(self.true_samples) >= 2 and est.stamp_sec > self.true_samples[-1].stamp_sec:
                break

            self.pending_est_samples.popleft()
            aligned_true = self.interpolate_true_sample(est.stamp_sec)
            if aligned_true is None:
                continue

            if self.start_time_est is None:
                self.start_time_est = est.stamp_sec
                if self.duration_sec is None:
                    self.get_logger().info(
                        f"Started aligned CSV recording at est stamp {self.start_time_est:.3f}s "
                        "and will continue until interrupted."
                    )
                else:
                    self.get_logger().info(
                        f"Started aligned CSV recording at est stamp {self.start_time_est:.3f}s "
                        f"for {self.duration_sec:.1f}s of estimator data."
                    )

            rel_time = est.stamp_sec - self.start_time_est
            err_yaw = wrap_to_pi(aligned_true.yaw - est.yaw)
            self.writer.writerow(
                [
                    f"{rel_time:.9f}",
                    f"{est.x:.9f}",
                    f"{est.y:.9f}",
                    f"{est.z:.9f}",
                    f"{est.yaw:.9f}",
                    f"{aligned_true.x:.9f}",
                    f"{aligned_true.y:.9f}",
                    f"{aligned_true.z:.9f}",
                    f"{aligned_true.yaw:.9f}",
                    f"{aligned_true.x - est.x:.9f}",
                    f"{aligned_true.y - est.y:.9f}",
                    f"{aligned_true.z - est.z:.9f}",
                    f"{err_yaw:.9f}",
                ]
            )
            self.file.flush()
            if self.duration_sec is not None and rel_time >= self.duration_sec:
                self.get_logger().info(
                    f"Reached requested duration ({self.duration_sec:.1f}s). Finishing recorder."
                )
                self.finish()
                return

    def interpolate_true_sample(self, target_time: float):
        samples = list(self.true_samples)
        if not samples:
            return None
        if len(samples) == 1:
            return samples[0]

        times = [sample.stamp_sec for sample in samples]
        if target_time <= times[0]:
            return samples[0]
        if target_time >= times[-1]:
            return samples[-1]

        upper_index = bisect.bisect_left(times, target_time)
        lower_index = max(0, upper_index - 1)
        low = samples[lower_index]
        high = samples[upper_index]

        dt = high.stamp_sec - low.stamp_sec
        if abs(dt) < 1e-12:
            return low

        ratio = (target_time - low.stamp_sec) / dt
        yaw_delta = wrap_to_pi(high.yaw - low.yaw)
        return OdomSample(
            stamp_sec=target_time,
            x=low.x + ratio * (high.x - low.x),
            y=low.y + ratio * (high.y - low.y),
            z=low.z + ratio * (high.z - low.z),
            yaw=wrap_to_pi(low.yaw + ratio * yaw_delta),
        )

    def trim_true_samples(self) -> None:
        if len(self.true_samples) <= 200:
            return

        lower_bound = self.pending_est_samples[0].stamp_sec if self.pending_est_samples else self.true_samples[-2].stamp_sec
        while len(self.true_samples) > 200 and len(self.true_samples) >= 2 and self.true_samples[1].stamp_sec < lower_bound:
            self.true_samples.popleft()

    def finish(self) -> None:
        if self.finished:
            return
        self.finished = True
        self.flush_pending_estimates()
        if rclpy.ok():
            self.get_logger().info(f"Wrote aligned estimate/ground-truth CSV to {self.output_path}")
        else:
            print(f"Wrote aligned estimate/ground-truth CSV to {self.output_path}")
        self.file.close()
        if self.safety_timer is not None:
            self.destroy_timer(self.safety_timer)
        if rclpy.ok():
            rclpy.shutdown()

    def finish_due_to_timeout(self) -> None:
        self.get_logger().warning(
            "Safety timeout reached before collecting the full requested duration. "
            "Finishing recorder with whatever samples are available."
        )
        self.finish()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Record /drone/odom and /drone/true_odom into an aligned CSV during runtime."
    )
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Requested aligned data duration in seconds. Use 0 or a negative value to record until interrupted.",
    )
    parser.add_argument("--odom-topic", default="/drone/odom")
    parser.add_argument("--true-odom-topic", default="/drone/true_odom")
    args = parser.parse_args()

    rclpy.init()
    duration_sec = args.duration if args.duration > 0.0 else None
    node = DroneAlignmentRecorder(
        output_path=Path(args.output),
        duration_sec=duration_sec,
        odom_topic=args.odom_topic,
        true_odom_topic=args.true_odom_topic,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish()
    finally:
        if not node.finished:
            node.finish()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
