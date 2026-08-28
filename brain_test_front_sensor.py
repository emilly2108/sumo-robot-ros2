# -*- coding: utf-8 -*-
"""Test the front color-sensor red/blue behavior with console sensor topics."""

import math

from std_msgs.msg import Bool

from brain_test_common import BrainTestBase, run_node


class FrontSensorTestNode(BrainTestBase):
    def __init__(self):
        super().__init__('brain_test_front_sensor')
        self.create_console_sensor_inputs()
        self.create_subscription(Bool, '/red_between_green', self.red_between_green_callback, 10)

        self.turn_45_speed = 0.8
        self.turn_45_duration = (math.pi / 4.0) / self.turn_45_speed
        self.front_green_distance_threshold = 0.40
        self.front_reverse_distance = 0.10
        self.front_reverse_linear_vel = -0.25
        self.front_reverse_duration = self.front_reverse_distance / abs(self.front_reverse_linear_vel)
        self.front_escape_state = None
        self.front_escape_started_at = self.get_clock().now()
        self.front_turn_direction = 1.0

        self.red_between_green = False
        self.red_front_state = None
        self.red_front_started_at = self.get_clock().now()
        self.red_front_turn_direction = 1.0
        self.red_front_repeat_count = 0
        self.red_front_max_repeats = 3
        self.red_front_forward_linear_vel = 0.25
        self.red_front_forward_duration = self.front_reverse_distance / self.red_front_forward_linear_vel
        self.red_charge_mode = False
        self.green_angle_error_limit = 0.85

        self.start_control_timer()
        self.get_logger().info('TEST MODE: front_sensor')

    def red_between_green_callback(self, msg):
        self.red_between_green = msg.data

    def start_red_front_escape(self, turn_direction, red_between_green):
        self.red_front_turn_direction = turn_direction
        self.red_front_repeat_count = 0
        self.red_charge_mode = False
        self.red_front_started_at = self.get_clock().now()
        if red_between_green:
            self.red_front_state = 'RED_BACK_10CM'
            self.target_found = True
        else:
            self.red_front_state = 'SIMPLE_BACK_10CM'
        return self.front_reverse_linear_vel, 0.0

    def green_ready_to_charge(self):
        return self.camera_target_found and 0.0 < self.target_distance <= self.front_green_distance_threshold

    def green_in_approach_angle(self):
        return self.camera_target_found and abs(self.target_error) <= self.green_angle_error_limit

    def run_red_front_escape(self):
        now = self.get_clock().now()
        if self.red_charge_mode:
            if self.camera_target_found:
                return self.boost_forward_command()
            self.red_charge_mode = False
            self.reset_green_target()
            return self.normal_forward_command()

        if self.red_front_state in ('RED_BACK_10CM', 'SIMPLE_BACK_10CM'):
            elapsed = (now - self.red_front_started_at).nanoseconds / 1e9
            if elapsed < self.front_reverse_duration:
                if self.red_front_state == 'RED_BACK_10CM':
                    self.target_found = True
                return self.front_reverse_linear_vel, 0.0
            self.red_front_state = (
                'RED_TURN_45' if self.red_front_state == 'RED_BACK_10CM' else 'SIMPLE_TURN_45'
            )
            self.red_front_started_at = now
            return 0.0, self.red_front_turn_direction * self.turn_45_speed

        if self.red_front_state in ('RED_TURN_45', 'SIMPLE_TURN_45', 'GREEN_APPROACH_TURN'):
            elapsed = (now - self.red_front_started_at).nanoseconds / 1e9
            if elapsed < self.turn_45_duration:
                if self.red_front_state == 'RED_TURN_45':
                    self.target_found = True
                return 0.0, self.red_front_turn_direction * self.turn_45_speed
            if self.red_front_state == 'SIMPLE_TURN_45':
                self.red_front_state = None
                return self.normal_forward_command()
            self.red_front_state = (
                'GREEN_APPROACH_FORWARD_10CM'
                if self.red_front_state == 'GREEN_APPROACH_TURN'
                else 'RED_FORWARD_10CM'
            )
            self.red_front_started_at = now
            return self.red_front_forward_linear_vel, 0.0

        if self.red_front_state in ('RED_FORWARD_10CM', 'GREEN_APPROACH_FORWARD_10CM'):
            elapsed = (now - self.red_front_started_at).nanoseconds / 1e9
            if elapsed < self.red_front_forward_duration:
                if self.red_front_state == 'RED_FORWARD_10CM':
                    self.target_found = True
                return self.red_front_forward_linear_vel, 0.0
            if self.red_front_state == 'RED_FORWARD_10CM':
                self.red_front_repeat_count += 1
                if self.red_front_repeat_count < self.red_front_max_repeats:
                    self.red_front_state = 'RED_TURN_45'
                    self.red_front_started_at = now
                    return 0.0, self.red_front_turn_direction * self.turn_45_speed
            self.red_front_state = 'RED_CHECK_GREEN'

        if self.red_front_state == 'RED_CHECK_GREEN':
            if not self.camera_target_found:
                self.reset_green_target()
                self.red_front_state = None
                return self.normal_forward_command()
            self.target_found = True
            if self.green_ready_to_charge():
                self.red_charge_mode = True
                self.red_front_state = None
                return self.boost_forward_command()
            if self.green_in_approach_angle():
                self.red_front_state = 'GREEN_APPROACH_TURN'
                self.red_front_started_at = now
                return 0.0, self.red_front_turn_direction * self.turn_45_speed
            self.red_front_state = None
            return self.compute_green_follow_command()

        return self.normal_forward_command()

    def green_color_sensor_forward(self, sensor1, sensor2):
        if self.red_front_state is not None or self.red_charge_mode:
            return self.run_red_front_escape()

        if self.front_escape_state == 'BACK_10CM':
            elapsed = (self.get_clock().now() - self.front_escape_started_at).nanoseconds / 1e9
            if elapsed < self.front_reverse_duration:
                return self.front_reverse_linear_vel, 0.0
            self.front_escape_state = 'TURN_45'
            self.front_escape_started_at = self.get_clock().now()
            return 0.0, self.front_turn_direction * self.turn_45_speed

        if self.front_escape_state == 'TURN_45':
            elapsed = (self.get_clock().now() - self.front_escape_started_at).nanoseconds / 1e9
            if elapsed < self.turn_45_duration:
                return 0.0, self.front_turn_direction * self.turn_45_speed
            self.front_escape_state = None
            return self.compute_green_follow_command()

        if sensor1 == 1 and sensor2 == 0:
            return self.start_red_front_escape(1.0, self.red_between_green)
        if sensor1 == 0 and sensor2 == 1:
            return self.start_red_front_escape(-1.0, self.red_between_green)
        if sensor1 == 1 and sensor2 == 1:
            return self.normal_forward_command()
        if sensor1 == 2 and sensor2 == 0:
            self.front_turn_direction = 1.0
            self.front_escape_state = 'BACK_10CM'
            self.front_escape_started_at = self.get_clock().now()
            return self.front_reverse_linear_vel, 0.0
        if sensor1 == 0 and sensor2 == 2:
            self.front_turn_direction = -1.0
            self.front_escape_state = 'BACK_10CM'
            self.front_escape_started_at = self.get_clock().now()
            return self.front_reverse_linear_vel, 0.0
        if sensor1 == 2 and sensor2 == 2:
            self.front_escape_state = None
            if self.target_found and self.target_distance >= self.front_green_distance_threshold:
                return self.compute_green_follow_command()
            return 0.0, 0.0

        self.front_escape_state = None
        return self.normal_forward_command()

    def extra_log_text(self):
        return (
            f' | s1={self.sensor1} s2={self.sensor2} '
            f'red_between={self.red_between_green} red_state={self.red_front_state} '
            f'blue_state={self.front_escape_state}'
        )

    def control_loop(self):
        if self.green_data_is_stale():
            self.reset_green_target()
        target_v, target_w = self.green_color_sensor_forward(self.sensor1, self.sensor2)
        self.publish_smoothed_command(target_v, target_w)


if __name__ == '__main__':
    run_node(FrontSensorTestNode)
