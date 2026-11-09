#!/usr/bin/env python3
"""Measure green-target recentering from ROS topics without commanding motors."""

import argparse
import math
import statistics
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


@dataclass
class Trial:
    number: int
    started_at: float
    start_px: float
    start_distance_m: float | None
    first_inside_at: float | None = None
    inside_frames: int = 0
    max_command_w: float | None = None


class GreenCenterTiming(Node):
    def __init__(
        self,
        half_width: float,
        deadzone: float,
        yaml_kp: float,
        stable_frames: int,
        frame_timeout: float,
        target_lost_frames: int,
    ) -> None:
        super().__init__("green_center_timing")
        self.half_width = half_width
        self.deadzone = deadzone
        self.stable_frames = stable_frames
        self.frame_timeout = frame_timeout
        self.target_lost_frames = target_lost_frames

        self.target_found = False
        self.missing_frames = 0
        self.last_error_at: float | None = None
        self.target_distance_m: float | None = None
        self.trial: Trial | None = None
        self.trials_started = 0
        self.trials_aborted = 0
        self.completed_ms: list[float] = []

        self.create_subscription(Float32, "/target_error", self.on_error, 1)
        self.create_subscription(Bool, "/target_found", self.on_found, 1)
        self.create_subscription(Float32, "/target_distance", self.on_distance, 1)
        self.create_subscription(Twist, "/cmd_vel", self.on_command, 1)
        self.create_timer(0.05, self.check_frame_timeout)

        print(
            f"READY yaml_kp={yaml_kp:.2f} deadzone={deadzone:.3f} "
            f"half_width={half_width:.0f}px "
            f"stable_frames={stable_frames}",
            flush=True,
        )
        print(
            "Timing begins on the first out-of-zone /target_error message. "
            "No motor commands are published.",
            flush=True,
        )

    def on_found(self, msg: Bool) -> None:
        if msg.data:
            self.target_found = True
            self.missing_frames = 0
            return
        self.missing_frames += 1
        if self.missing_frames >= self.target_lost_frames:
            self.target_found = False
            self.last_error_at = None
            self.abort_trial("TARGET_LOST")

    def on_distance(self, msg: Float32) -> None:
        value = float(msg.data)
        self.target_distance_m = value if math.isfinite(value) and 0.0 < value < 9.9 else None

    def on_command(self, msg: Twist) -> None:
        if self.trial is not None and math.isfinite(msg.angular.z):
            value = abs(float(msg.angular.z))
            if self.trial.max_command_w is None or value > self.trial.max_command_w:
                self.trial.max_command_w = value

    def on_error(self, msg: Float32) -> None:
        if not math.isfinite(msg.data):
            return

        # Vision publishes target_error only when green is detected. This can
        # arrive before the matching target_found=True on a different topic.
        self.target_found = True
        self.missing_frames = 0
        now = time.monotonic()
        if self.last_error_at is not None and now - self.last_error_at > self.frame_timeout:
            self.abort_trial("FRAME_GAP")
        self.last_error_at = now

        offset_px = abs(float(msg.data)) * self.half_width
        inside = abs(float(msg.data)) < self.deadzone
        if self.trial is None:
            if not inside:
                self.trials_started += 1
                self.trial = Trial(
                    self.trials_started, now, offset_px, self.target_distance_m
                )
                distance = self.format_distance(self.target_distance_m)
                print(
                    f"START #{self.trial.number} offset={offset_px:.1f}px "
                    f"distance={distance}",
                    flush=True,
                )
            return

        if not inside:
            self.trial.first_inside_at = None
            self.trial.inside_frames = 0
            return

        if self.trial.first_inside_at is None:
            self.trial.first_inside_at = now
        self.trial.inside_frames += 1
        if self.trial.inside_frames < self.stable_frames:
            return

        first_ms = (self.trial.first_inside_at - self.trial.started_at) * 1000.0
        stable_ms = (now - self.trial.started_at) * 1000.0
        self.completed_ms.append(stable_ms)
        median_ms = statistics.median(self.completed_ms[-10:])
        max_w = (
            "n/a" if self.trial.max_command_w is None
            else f"{self.trial.max_command_w:.3f}rad/s"
        )
        print(
            f"CENTERED #{self.trial.number} first={first_ms:.0f}ms "
            f"stable={stable_ms:.0f}ms start={self.trial.start_px:.1f}px "
            f"max_cmd_w={max_w} median_last_10={median_ms:.0f}ms "
            f"completed={len(self.completed_ms)} aborted={self.trials_aborted}",
            flush=True,
        )
        self.trial = None

    def check_frame_timeout(self) -> None:
        if self.trial is not None and self.last_error_at is not None:
            if time.monotonic() - self.last_error_at > self.frame_timeout:
                self.abort_trial("FRAME_GAP")
                self.target_found = False
                self.last_error_at = None

    def abort_trial(self, reason: str) -> None:
        if self.trial is None:
            return
        self.trials_aborted += 1
        elapsed_ms = (time.monotonic() - self.trial.started_at) * 1000.0
        print(
            f"ABORT #{self.trial.number} reason={reason} "
            f"elapsed={elapsed_ms:.0f}ms start={self.trial.start_px:.1f}px",
            flush=True,
        )
        self.trial = None

    @staticmethod
    def format_distance(distance_m: float | None) -> str:
        return "n/a" if distance_m is None else f"{distance_m:.2f}m"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stable-frames", type=int, default=3,
        help="Consecutive in-zone samples needed for success (default: 3).",
    )
    parser.add_argument(
        "--frame-timeout", type=float, default=0.5,
        help="Abort a trial after this many seconds without target-error data.",
    )
    args = parser.parse_args()

    settings_path = Path(__file__).resolve().parent / "hfsm_brain" / "robot_settings.yaml"
    with settings_path.open("r", encoding="utf-8") as stream:
        settings = yaml.safe_load(stream)
    brain = settings["brain"]
    half_width = float(settings["vision"]["WIDTH"]) / 2.0
    deadzone = float(brain["deadzone"])
    if (half_width <= 0.0 or not math.isfinite(half_width)
            or not 0.0 < deadzone < 1.0
            or args.stable_frames < 1
            or not math.isfinite(args.frame_timeout) or args.frame_timeout <= 0.0):
        parser.error("Invalid width, deadzone, stable frames, or timeout")

    rclpy.init()
    node = GreenCenterTiming(
        half_width=half_width,
        deadzone=deadzone,
        yaml_kp=float(brain["kp"]),
        stable_frames=args.stable_frames,
        frame_timeout=args.frame_timeout,
        target_lost_frames=int(brain["target_lost_frames"]),
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
