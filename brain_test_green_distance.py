# -*- coding: utf-8 -*-
"""Test the 1.2 m green-distance decision by itself."""

import math

from brain_test_common import BrainTestBase, run_node


class GreenDistanceTestNode(BrainTestBase):
    def __init__(self):
        super().__init__('brain_test_green_distance')
        self.turn_45_speed = 0.8
        self.turn_45_duration = (math.pi / 4.0) / self.turn_45_speed
        self.state = 'SEARCH'
        self.state_started_at = self.get_clock().now()
        self.start_control_timer()
        self.get_logger().info('TEST MODE: green_distance')

    def start_state(self, state):
        self.state = state
        self.state_started_at = self.get_clock().now()

    def extra_log_text(self):
        return f' | state={self.state}'

    def control_loop(self):
        if self.green_data_is_stale():
            self.reset_green_target()

        now = self.get_clock().now()
        if self.state == 'TURN_RIGHT_45':
            elapsed = (now - self.state_started_at).nanoseconds / 1e9
            if elapsed < self.turn_45_duration:
                self.publish_smoothed_command(0.0, self.turn_45_speed)
            else:
                self.start_state('GO_STRAIGHT')
                self.publish_smoothed_command(*self.normal_forward_command())
            return

        if not self.target_found:
            self.start_state('SEARCH')
            self.publish_smoothed_command(0.0, 0.8)
            return

        if self.target_distance < 1.2:
            self.start_state('GREEN_FOLLOW')
            self.publish_smoothed_command(*self.compute_green_follow_command())
            return

        if self.state != 'GO_STRAIGHT':
            self.start_state('TURN_RIGHT_45')
            self.publish_smoothed_command(0.0, self.turn_45_speed)
            return

        self.publish_smoothed_command(*self.normal_forward_command())


if __name__ == '__main__':
    run_node(GreenDistanceTestNode)
