# -*- coding: utf-8 -*-
"""Test only the opening: left 45 degrees, then straight until a trigger."""

import math

from std_msgs.msg import Float32

from brain_test_common import BrainTestBase, run_node


class OpeningTestNode(BrainTestBase):
    def __init__(self):
        super().__init__('brain_test_opening')
        self.wall_close_distance = 0.20
        self.wall_left_distance = 9.9
        self.wall_right_distance = 9.9
        self.turn_45_speed = 0.8
        self.turn_45_duration = (math.pi / 4.0) / self.turn_45_speed
        self.state = 'TURN_LEFT_45'
        self.state_started_at = self.get_clock().now()

        self.create_subscription(Float32, '/wall_left_distance', self.wall_left_callback, 10)
        self.create_subscription(Float32, '/wall_right_distance', self.wall_right_callback, 10)
        self.start_control_timer()
        self.get_logger().info('TEST MODE: opening')

    def wall_left_callback(self, msg):
        self.wall_left_distance = msg.data

    def wall_right_callback(self, msg):
        self.wall_right_distance = msg.data

    def wall_is_close(self):
        return (
            0.0 < self.wall_left_distance <= self.wall_close_distance
            or 0.0 < self.wall_right_distance <= self.wall_close_distance
        )

    def extra_log_text(self):
        return (
            f' | state={self.state} '
            f'wall_L={self.wall_left_distance:.2f} wall_R={self.wall_right_distance:.2f}'
        )

    def control_loop(self):
        if self.green_data_is_stale():
            self.reset_green_target()

        now = self.get_clock().now()
        if self.state == 'TURN_LEFT_45':
            elapsed = (now - self.state_started_at).nanoseconds / 1e9
            if elapsed < self.turn_45_duration:
                self.publish_smoothed_command(0.0, self.turn_45_speed)
            else:
                self.state = 'GO_STRAIGHT'
                self.state_started_at = now
                self.publish_smoothed_command(*self.normal_forward_command())
            return

        if self.state == 'GO_STRAIGHT':
            if self.target_found:
                self.state = 'GREEN_SEEN_STOP'
            elif self.wall_is_close():
                self.state = 'WALL_SEEN_STOP'
            else:
                self.publish_smoothed_command(*self.normal_forward_command())
                return

        self.publish_smoothed_command(0.0, 0.0)


if __name__ == '__main__':
    run_node(OpeningTestNode)
