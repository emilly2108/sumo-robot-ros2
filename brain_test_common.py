# -*- coding: utf-8 -*-
"""Common ROS2 code used by the one-function brain tests."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32


class BrainTestBase(Node):
    """Keeps the vision topics and /cmd_vel interface identical in every test."""

    def __init__(self, node_name):
        super().__init__(node_name)

        self.normal_linear_vel = 0.6
        self.boost_linear_vel = 0.8
        self.max_linear_vel = self.boost_linear_vel
        self.limit_angular_vel = 2.0
        self.accel_linear = 0.6
        self.accel_angular = 1.5
        self.kp = 1.8
        self.deadzone = 0.03
        self.dt = 0.02

        self.target_error = 0.0
        self.target_distance = 9.9
        self.target_found = False
        self.camera_target_found = False
        self.last_seen = self.get_clock().now()

        self.current_v = 0.0
        self.current_w = 0.0
        self.last_log_text = None
        self.last_log_time = self.get_clock().now()

        self.create_subscription(Float32, '/target_error', self.error_callback, 10)
        self.create_subscription(Float32, '/target_distance', self.distance_callback, 10)
        self.create_subscription(Bool, '/target_found', self.found_callback, 10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)

    def start_control_timer(self):
        self.create_timer(self.dt, self.control_loop)

    def error_callback(self, msg):
        self.target_error = msg.data
        self.last_seen = self.get_clock().now()

    def distance_callback(self, msg):
        self.target_distance = msg.data
        self.last_seen = self.get_clock().now()

    def found_callback(self, msg):
        self.camera_target_found = msg.data
        self.target_found = msg.data
        if not msg.data:
            self.reset_green_target()
        self.last_seen = self.get_clock().now()

    def reset_green_target(self):
        self.camera_target_found = False
        self.target_found = False
        self.target_error = 0.0
        self.target_distance = 9.9

    def green_data_is_stale(self):
        elapsed = (self.get_clock().now() - self.last_seen).nanoseconds / 1e9
        return elapsed > 0.5

    def compute_green_follow_command(self):
        target_v = self.normal_linear_vel
        target_w = -self.kp * self.target_error
        if abs(self.target_error) < self.deadzone:
            target_w = 0.0
        return target_v, target_w

    def normal_forward_command(self):
        return self.normal_linear_vel, 0.0

    def boost_forward_command(self):
        return self.boost_linear_vel, 0.0

    def create_console_sensor_inputs(self):
        """Three temporary topics replace unconnected color sensors for testing."""
        self.sensor1 = 0
        self.sensor2 = 0
        self.sensor3 = 0
        self.create_subscription(Int32, '/test_sensor1', self.sensor1_callback, 10)
        self.create_subscription(Int32, '/test_sensor2', self.sensor2_callback, 10)
        self.create_subscription(Int32, '/test_sensor3', self.sensor3_callback, 10)

    def sensor1_callback(self, msg):
        self.sensor1 = msg.data
        self.get_logger().info(f'TEST SENSOR: s1={self.sensor1} s2={self.sensor2} s3={self.sensor3}')

    def sensor2_callback(self, msg):
        self.sensor2 = msg.data
        self.get_logger().info(f'TEST SENSOR: s1={self.sensor1} s2={self.sensor2} s3={self.sensor3}')

    def sensor3_callback(self, msg):
        self.sensor3 = msg.data
        self.get_logger().info(f'TEST SENSOR: s1={self.sensor1} s2={self.sensor2} s3={self.sensor3}')

    def motion_text(self, target_v, target_w):
        if abs(target_v) < 0.05 and target_w > 0.05:
            return 'TURN LEFT'
        if abs(target_v) < 0.05 and target_w < -0.05:
            return 'TURN RIGHT'
        if target_v > 0.05 and abs(target_w) < 0.05:
            return 'GO STRAIGHT'
        if target_v < -0.05 and abs(target_w) < 0.05:
            return 'GO BACKWARD'
        if target_v > 0.05 and target_w > 0.05:
            return 'GO + TURN LEFT'
        if target_v > 0.05 and target_w < -0.05:
            return 'GO + TURN RIGHT'
        return 'STOP'

    def extra_log_text(self):
        return ''

    def log_command(self, target_v, target_w):
        text = f'{self.motion_text(target_v, target_w)}{self.extra_log_text()}'
        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds / 1e9
        if text == self.last_log_text and elapsed < 1.0:
            return

        self.last_log_text = text
        self.last_log_time = now
        self.get_logger().info(
            f'{text} | v={target_v:.2f} w={target_w:.2f} '
            f'green={self.target_found} distance={self.target_distance:.2f}m'
        )

    def publish_smoothed_command(self, target_v, target_w):
        v_diff = target_v - self.current_v
        max_v_step = self.accel_linear * self.dt
        if abs(v_diff) > max_v_step:
            self.current_v += math.copysign(max_v_step, v_diff)
        else:
            self.current_v = target_v

        w_diff = target_w - self.current_w
        max_w_step = self.accel_angular * self.dt
        if abs(w_diff) > max_w_step:
            self.current_w += math.copysign(max_w_step, w_diff)
        else:
            self.current_w = target_w

        self.current_v = max(min(self.current_v, self.max_linear_vel), -self.max_linear_vel)
        self.current_w = max(min(self.current_w, self.limit_angular_vel), -self.limit_angular_vel)

        twist = Twist()
        twist.linear.x = self.current_v
        twist.angular.z = self.current_w
        self.pub_vel.publish(twist)
        self.log_command(self.current_v, self.current_w)

    def control_loop(self):
        raise NotImplementedError


def run_node(node_class, args=None):
    rclpy.init(args=args)
    node = node_class()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
