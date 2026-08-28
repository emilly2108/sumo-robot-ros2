# -*- coding: utf-8 -*-
"""Test only the rear color-sensor boost, reverse, and hold behavior."""

from brain_test_common import BrainTestBase, run_node


class RearSensorTestNode(BrainTestBase):
    def __init__(self):
        super().__init__('brain_test_rear_sensor')
        self.create_console_sensor_inputs()
        self.reverse_linear_vel = -0.3
        self.all_blue_started_at = None
        self.backing_until_forward_bule = False
        self.holding_torque_after_forward_bule = False
        self.all_sensor_blue_started_at = None
        self.backing_until_no_color = False
        self.start_control_timer()
        self.get_logger().info('TEST MODE: rear_sensor')

    def all_blue(self, sensor1, sensor2, sensor3):
        return sensor1 == 1 and sensor2 == 1 and sensor3 == 1

    def forward_bule(self, sensor1, sensor2, sensor3):
        return sensor1 == 1 and sensor2 == 1 and sensor3 == 0

    def green_color_sensor_back(self, sensor1, sensor2, sensor3):
        if self.holding_torque_after_forward_bule:
            if self.forward_bule(sensor1, sensor2, sensor3):
                return 0.0, 0.0
            self.holding_torque_after_forward_bule = False

        if self.backing_until_no_color:
            if sensor1 == 0 and sensor2 == 0 and sensor3 == 0:
                self.backing_until_no_color = False
                self.all_sensor_blue_started_at = None
                return self.normal_forward_command()
            return self.reverse_linear_vel, 0.0

        if self.backing_until_forward_bule:
            if self.forward_bule(sensor1, sensor2, sensor3):
                self.backing_until_forward_bule = False
                self.holding_torque_after_forward_bule = True
                self.all_blue_started_at = None
                return 0.0, 0.0
            return self.reverse_linear_vel, 0.0

        if sensor1 == 0 and sensor2 == 0 and sensor3 == 1:
            self.all_blue_started_at = None
            self.all_sensor_blue_started_at = None
            return self.boost_forward_command()

        if sensor1 == 0 and sensor2 == 0 and sensor3 == 2:
            self.all_blue_started_at = None
            self.all_sensor_blue_started_at = None
            return self.boost_forward_command()

        if self.all_blue(sensor1, sensor2, sensor3):
            self.all_sensor_blue_started_at = None
            now = self.get_clock().now()
            if self.all_blue_started_at is None:
                self.all_blue_started_at = now
            elapsed = (now - self.all_blue_started_at).nanoseconds / 1e9
            if elapsed >= 5.0:
                self.backing_until_forward_bule = True
                return self.reverse_linear_vel, 0.0
            return self.boost_forward_command()

        if sensor1 == 2 and sensor2 == 2 and sensor3 == 2:
            self.all_blue_started_at = None
            now = self.get_clock().now()
            if self.all_sensor_blue_started_at is None:
                self.all_sensor_blue_started_at = now
            elapsed = (now - self.all_sensor_blue_started_at).nanoseconds / 1e9
            if elapsed >= 5.0:
                self.backing_until_no_color = True
                return self.reverse_linear_vel, 0.0
            return self.boost_forward_command()

        self.all_blue_started_at = None
        self.all_sensor_blue_started_at = None
        return self.normal_forward_command()

    def extra_log_text(self):
        return (
            f' | s1={self.sensor1} s2={self.sensor2} s3={self.sensor3} '
            f'back_111={self.backing_until_forward_bule} '
            f'hold_110={self.holding_torque_after_forward_bule} '
            f'back_222={self.backing_until_no_color}'
        )

    def control_loop(self):
        target_v, target_w = self.green_color_sensor_back(
            self.sensor1, self.sensor2, self.sensor3
        )
        self.publish_smoothed_command(target_v, target_w)


if __name__ == '__main__':
    run_node(RearSensorTestNode)
