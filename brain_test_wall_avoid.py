# -*- coding: utf-8 -*-
"""Test only wall avoidance using the wall topics from vision_node.py."""

import math

from std_msgs.msg import Float32

from brain_test_common import BrainTestBase, run_node


class WallAvoidTestNode(BrainTestBase):
    def __init__(self):
        super().__init__('brain_test_wall_avoid')
        self.wall_close_distance = 0.20
        self.opponent_close_distance = 0.50
        self.wall_left_distance = 9.9
        self.wall_right_distance = 9.9
        self.wall_turning = False
        self.wall_turn_direction = 0.0
        self.wall_turn_speed = 0.8
        self.wall_turn_duration = (math.pi / 2.0) / self.wall_turn_speed
        self.wall_turn_started_at = self.get_clock().now()

        self.create_subscription(Float32, '/wall_left_distance', self.wall_left_callback, 10)
        self.create_subscription(Float32, '/wall_right_distance', self.wall_right_callback, 10)
        self.start_control_timer()
        self.get_logger().info('TEST MODE: wall_avoid')

    def wall_left_callback(self, msg):
        self.wall_left_distance = msg.data

    def wall_right_callback(self, msg):
        self.wall_right_distance = msg.data

    def get_wall_avoid_direction(self):
        if self.target_found and 0.0 < self.target_distance <= self.opponent_close_distance:
            return None

        left_close = 0.0 < self.wall_left_distance <= self.wall_close_distance
        right_close = 0.0 < self.wall_right_distance <= self.wall_close_distance
        if not (left_close or right_close):
            return None
        if self.wall_left_distance > self.wall_right_distance:
            return 1.0
        if self.wall_right_distance > self.wall_left_distance:
            return -1.0
        return 1.0

    def start_wall_turn(self, direction):
        self.wall_turn_direction = direction
        self.wall_turn_started_at = self.get_clock().now()
        self.wall_turning = True
        side = 'left' if direction > 0.0 else 'right'
        self.get_logger().warn(
            f'WALL AVOID START: {side} 90 deg | '
            f'L={self.wall_left_distance:.2f} R={self.wall_right_distance:.2f}'
        )

    def extra_log_text(self):
        turn = 'left' if self.wall_turn_direction > 0.0 else 'right'
        return (
            f' | wall_turning={self.wall_turning} turn={turn} '
            f'wall_L={self.wall_left_distance:.2f} wall_R={self.wall_right_distance:.2f}'
        )

    def control_loop(self):
        if self.green_data_is_stale():
            self.reset_green_target()

        if self.wall_turning:
            elapsed = (self.get_clock().now() - self.wall_turn_started_at).nanoseconds / 1e9
            if elapsed < self.wall_turn_duration:
                self.publish_smoothed_command(0.0, self.wall_turn_direction * self.wall_turn_speed)
            else:
                self.wall_turning = False
                self.publish_smoothed_command(0.0, 0.0)
            return

        direction = self.get_wall_avoid_direction()
        if direction is not None:
            self.start_wall_turn(direction)
            self.publish_smoothed_command(0.0, direction * self.wall_turn_speed)
            return

        self.publish_smoothed_command(*self.normal_forward_command())


if __name__ == '__main__':
    run_node(WallAvoidTestNode)
