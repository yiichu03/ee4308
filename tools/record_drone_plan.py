#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path
from typing import Optional

import rclpy
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node


class DronePlanRecorder(Node):
    def __init__(self, output_path: Path, duration_sec: Optional[float], plan_topic: str) -> None:
        super().__init__("proj2_drone_plan_recorder")
        self.output_path = output_path
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.output_path.open("w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(
            [
                "recv_time_sec",
                "plan_stamp_sec",
                "num_points",
                "goal_x",
                "goal_y",
                "goal_z",
            ]
        )

        self.duration_sec = duration_sec
        self.finished = False
        self.start_time_sec = None
        self.safety_timer = None

        self.create_subscription(PathMsg, plan_topic, self.callback_plan, 10)
        if self.duration_sec is not None:
            self.safety_timer = self.create_timer(max(self.duration_sec + 20.0, 20.0), self.finish_due_to_timeout)

    def callback_plan(self, msg: PathMsg) -> None:
        recv_time_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time_sec is None:
            self.start_time_sec = recv_time_sec
            if self.duration_sec is None:
                self.get_logger().info("Started /drone/plan recording and will continue until interrupted.")
            else:
                self.get_logger().info(
                    f"Started /drone/plan recording for up to {self.duration_sec:.1f}s."
                )

        if not msg.poses:
            self.writer.writerow(
                [
                    f"{recv_time_sec:.9f}",
                    f"{msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9:.9f}",
                    "0",
                    "",
                    "",
                    "",
                ]
            )
        else:
            goal = msg.poses[-1].pose.position
            self.writer.writerow(
                [
                    f"{recv_time_sec:.9f}",
                    f"{msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9:.9f}",
                    str(len(msg.poses)),
                    f"{goal.x:.9f}",
                    f"{goal.y:.9f}",
                    f"{goal.z:.9f}",
                ]
            )
        self.file.flush()

        if self.duration_sec is not None and recv_time_sec - self.start_time_sec >= self.duration_sec:
            self.get_logger().info(
                f"Reached requested duration ({self.duration_sec:.1f}s). Finishing /drone/plan recorder."
            )
            self.finish()

    def finish(self) -> None:
        if self.finished:
            return
        self.finished = True
        if rclpy.ok():
            self.get_logger().info(f"Wrote /drone/plan CSV to {self.output_path}")
        else:
            print(f"Wrote /drone/plan CSV to {self.output_path}")
        self.file.close()
        if self.safety_timer is not None:
            self.destroy_timer(self.safety_timer)
        if rclpy.ok():
            rclpy.shutdown()

    def finish_due_to_timeout(self) -> None:
        self.get_logger().warning(
            "Safety timeout reached before finishing /drone/plan recording. Finishing with available samples."
        )
        self.finish()


def main() -> None:
    parser = argparse.ArgumentParser(description="Record /drone/plan into a CSV during runtime.")
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Requested recording duration in seconds. Use 0 or a negative value to record until interrupted.",
    )
    parser.add_argument("--plan-topic", default="/drone/plan")
    args = parser.parse_args()

    rclpy.init()
    duration_sec = args.duration if args.duration > 0.0 else None
    node = DronePlanRecorder(
        output_path=Path(args.output),
        duration_sec=duration_sec,
        plan_topic=args.plan_topic,
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
