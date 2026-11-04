#!/usr/bin/env python3
"""Publish a standalone 45-degree in-place turn on ROS 2 /cmd_vel."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


MAX_ANGULAR_SPEED_RAD_S = 10.0
DEFAULT_ANGULAR_SPEED_RAD_S = MAX_ANGULAR_SPEED_RAD_S
DEFAULT_PUBLISH_RATE_HZ = 20.0
DEFAULT_MOTOR_WAIT_SEC = 10.0
STOP_REPEAT_SEC = 0.25
TURN_ANGLE_DEGREES = 45.0
CALIBRATION_REFERENCE_ANGLE_DEGREES = 90.0
CALIBRATION_REFERENCE_SPEED_RAD_S = 5.0
# Calibration duration from the attached working code; target angle scales it.
CALIBRATION_REFERENCE_DURATION_SEC = 0.05


def calculate_turn_duration(angle_degrees: float, angular_speed: float) -> float:
    """Scale the measured reference turn duration to the requested angle."""
    return (
        CALIBRATION_REFERENCE_DURATION_SEC
        * (angle_degrees / CALIBRATION_REFERENCE_ANGLE_DEGREES)
        * (CALIBRATION_REFERENCE_SPEED_RAD_S / angular_speed)
    )


class Turn45Node(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("turn_45_once")
        self.command_publisher = self.create_publisher(Twist, topic, 1)

    def publish_turn(self, angular_speed: float) -> None:
        command = Twist()
        command.linear.x = 0.0
        command.angular.z = angular_speed
        self.command_publisher.publish(command)

    def publish_stop(self) -> None:
        self.command_publisher.publish(Twist())


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            f"Rotate the robot in place by approximately "
            f"{TURN_ANGLE_DEGREES:g} degrees."
        )
    )
    parser.add_argument(
        "--direction",
        choices=("left", "right"),
        default="left",
        help="turn direction (default: left)",
    )
    parser.add_argument(
        "--angular-speed",
        type=float,
        default=DEFAULT_ANGULAR_SPEED_RAD_S,
        help="absolute angular speed in rad/s (default: 10.0)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help=(
            "turn duration in seconds; by default calculated for "
            f"{TURN_ANGLE_DEGREES:g} degrees"
        ),
    )
    parser.add_argument(
        "--topic", default="/cmd_vel", help="Twist command topic (default: /cmd_vel)"
    )
    parser.add_argument(
        "--publish-rate",
        type=float,
        default=DEFAULT_PUBLISH_RATE_HZ,
        help="command publish rate in Hz (default: 20)",
    )
    parser.add_argument(
        "--motor-wait",
        type=float,
        default=DEFAULT_MOTOR_WAIT_SEC,
        help="seconds to wait for a /cmd_vel subscriber (default: 10)",
    )
    args, ros_args = parser.parse_known_args()

    if not math.isfinite(args.angular_speed) or args.angular_speed <= 0.0:
        parser.error("--angular-speed must be a positive finite number")
    if args.angular_speed > MAX_ANGULAR_SPEED_RAD_S:
        parser.error(
            f"--angular-speed cannot exceed {MAX_ANGULAR_SPEED_RAD_S:g} rad/s "
            "(the maximum configured by the robot code)"
        )
    if not math.isfinite(args.publish_rate) or args.publish_rate <= 0.0:
        parser.error("--publish-rate must be a positive finite number")
    if not math.isfinite(args.motor_wait) or args.motor_wait <= 0.0:
        parser.error("--motor-wait must be a positive finite number")
    if args.duration is not None and (
        not math.isfinite(args.duration) or args.duration <= 0.0
    ):
        parser.error("--duration must be a positive finite number")
    if not args.topic:
        parser.error("--topic must not be empty")

    return args, ros_args


def main() -> int:
    args, ros_args = parse_args()
    duration = (
        args.duration
        if args.duration is not None
        else calculate_turn_duration(TURN_ANGLE_DEGREES, args.angular_speed)
    )
    direction_sign = 1.0 if args.direction == "left" else -1.0
    angular_speed = direction_sign * args.angular_speed
    period = 1.0 / args.publish_rate
    zero_count = max(3, math.ceil(STOP_REPEAT_SEC * args.publish_rate))

    rclpy.init(args=ros_args)
    node = Turn45Node(args.topic)
    try:
        node.get_logger().info(
            f"Waiting for a subscriber on {args.topic} (up to {args.motor_wait:g}s)"
        )
        wait_deadline = time.monotonic() + args.motor_wait
        while rclpy.ok() and node.command_publisher.get_subscription_count() == 0:
            if time.monotonic() >= wait_deadline:
                node.get_logger().error(
                    f"No motor node subscribed to {args.topic}; no turn was sent"
                )
                return 2
            rclpy.spin_once(node, timeout_sec=0.1)

        node.get_logger().info(
            f"Turning {TURN_ANGLE_DEGREES:g} degrees {args.direction} "
            f"at {args.angular_speed:g} rad/s "
            f"for {duration:.3f}s"
        )
        end_time = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end_time:
            node.publish_turn(angular_speed)
            time.sleep(min(period, max(0.0, end_time - time.monotonic())))

        for _ in range(zero_count):
            node.publish_stop()
            time.sleep(period)
        node.get_logger().info("Turn complete; zero velocity published")
        return 0
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted; publishing zero velocity")
        for _ in range(zero_count):
            node.publish_stop()
            time.sleep(period)
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
