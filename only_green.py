# -*- coding: utf-8 -*-

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class OnlyGreenNode(Node):
    def __init__(self):
        super().__init__('only_green_node')

        self.limit_linear_vel = 0.6
        self.limit_angular_vel = 2.0
        self.accel_linear = 0.12
        self.accel_angular = 0.25

        self.Kp = 1.8
        self.deadzone = 0.03
        self.dt = 0.02

        self.target_error = 0.0
        self.target_distance = 9.9
        self.target_found = False
        self.last_seen = self.get_clock().now()

        self.no_wall_distance = 9.9
        self.wall_close_distance = 0.20
        self.opponent_close_distance = 0.50
        self.wall_distance = self.no_wall_distance
        self.wall_left_distance = self.no_wall_distance
        self.wall_right_distance = self.no_wall_distance

        self.wall_turning = False
        self.wall_turn_direction = 0.0
        self.wall_turn_speed = 0.8
        self.wall_turn_duration = (math.pi / 2.0) / self.wall_turn_speed
        self.wall_turn_started_at = self.get_clock().now()

        self.current_v = 0.0
        self.current_w = 0.0

        self.state = "IDLE"
        self.state_started_at = self.get_clock().now()
        self.turn_45_direction = 1.0
        self.turn_45_speed = 0.8
        self.turn_45_duration = (math.pi / 4.0) / self.turn_45_speed
        self.opening_state = "TURN_LEFT_45"
        self.opening_started_at = self.get_clock().now()
        self.opening_turn_direction = 1.0
        self.last_motion_log_time = self.get_clock().now()
        self.last_motion_text = None

        self.create_subscription(Float32, '/target_error', self.error_callback, 10)
        self.create_subscription(Float32, '/target_distance', self.distance_callback, 10)
        self.create_subscription(Bool, '/target_found', self.found_callback, 10)
        self.create_subscription(Float32, '/wall_distance', self.wall_callback, 10)
        self.create_subscription(Float32, '/wall_left_distance', self.wall_left_callback, 10)
        self.create_subscription(Float32, '/wall_right_distance', self.wall_right_callback, 10)

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('Only green node started.')

    def error_callback(self, msg):
        self.target_error = msg.data
        self.last_seen = self.get_clock().now()

    def distance_callback(self, msg):
        self.target_distance = msg.data
        self.last_seen = self.get_clock().now()

    def found_callback(self, msg):
        self.target_found = msg.data
        if not self.target_found:
            self.reset_green_target()
        self.last_seen = self.get_clock().now()

    def reset_green_target(self):
        self.target_found = False
        self.target_error = 0.0
        self.target_distance = 9.9

    def wall_callback(self, msg):
        self.wall_distance = msg.data

    def wall_left_callback(self, msg):
        self.wall_left_distance = msg.data

    def wall_right_callback(self, msg):
        self.wall_right_distance = msg.data

    def start_state(self, state):
        self.state = state
        self.state_started_at = self.get_clock().now()

    def wall_is_close(self):
        center_close = 0.0 < self.wall_distance <= self.wall_close_distance
        left_close = 0.0 < self.wall_left_distance <= self.wall_close_distance
        right_close = 0.0 < self.wall_right_distance <= self.wall_close_distance
        return center_close or left_close or right_close

    def opponent_is_close(self):
        return self.target_found and 0.0 < self.target_distance <= self.opponent_close_distance

    def should_avoid_wall(self):
        if self.opponent_is_close():
            return False

        return self.wall_is_close()

    def choose_wall_turn_direction(self):
        if self.wall_left_distance > self.wall_right_distance:
            return 1.0
        if self.wall_right_distance > self.wall_left_distance:
            return -1.0
        return 1.0

    def wall_turn_direction_text(self):
        return "left" if self.wall_turn_direction > 0.0 else "right"

    def start_wall_turn(self):
        self.wall_turn_direction = self.choose_wall_turn_direction()
        self.wall_turn_started_at = self.get_clock().now()
        self.wall_turning = True
        self.get_logger().warn(
            f"WALL AVOID START: turn {self.wall_turn_direction_text()} 90 deg | "
            f"L={self.wall_left_distance:.2f}m R={self.wall_right_distance:.2f}m"
        )

    def wall_turn_finished(self):
        elapsed = (self.get_clock().now() - self.wall_turn_started_at).nanoseconds / 1e9
        return elapsed >= self.wall_turn_duration

    def compute_green_follow_command(self):
        target_v = self.limit_linear_vel
        target_w = -self.Kp * self.target_error

        if abs(self.target_error) < self.deadzone:
            target_w = 0.0

        return target_v, target_w

    def simple_green_distance_command(self, sensor1, sensor2, sensor3):
        if sensor1 == 0 and sensor2 == 0 and sensor3 == 0:
            if self.target_distance > 1.2:
                self.start_state("START_TURN_RIGHT_45")
                return 0.0, self.turn_45_direction * self.turn_45_speed

            elif self.target_distance < 1.2:
                self.start_state("GREEN_FOLLOW")
                return self.compute_green_follow_command()

        return 0.0, 0.0

    def run_start_turn_right_45(self):
        elapsed = (self.get_clock().now() - self.state_started_at).nanoseconds / 1e9

        if elapsed < self.turn_45_duration:
            self.publish_smoothed_command(0.0, self.turn_45_direction * self.turn_45_speed)
        else:
            self.start_state("GO_STRAIGHT")
            self.publish_smoothed_command(self.limit_linear_vel, 0.0)

    def run_opening_sequence(self):
        now = self.get_clock().now()

        if self.opening_state == "TURN_LEFT_45":
            elapsed = (now - self.opening_started_at).nanoseconds / 1e9
            if elapsed < self.turn_45_duration:
                self.publish_smoothed_command(0.0, self.opening_turn_direction * self.turn_45_speed)
                return True

            self.opening_state = "GO_STRAIGHT"
            self.opening_started_at = now
            self.publish_smoothed_command(self.limit_linear_vel, 0.0)
            return True

        if self.opening_state == "GO_STRAIGHT":
            if self.target_found:
                self.opening_state = "DONE"
                return False

            if self.should_avoid_wall():
                self.opening_state = "DONE"
                self.start_wall_turn()
                self.publish_smoothed_command(0.0, self.wall_turn_direction * self.wall_turn_speed)
                return True

            self.publish_smoothed_command(self.limit_linear_vel, 0.0)
            return True

        return False

    def control_loop(self):
        stale_time = (self.get_clock().now() - self.last_seen).nanoseconds / 1e9
        if stale_time > 0.5:
            self.reset_green_target()

        if self.run_opening_sequence():
            return

        if self.wall_turning:
            if self.wall_turn_finished():
                self.wall_turning = False
                self.start_state("GO_STRAIGHT")
                self.publish_smoothed_command(0.0, 0.0)
            else:
                self.publish_smoothed_command(0.0, self.wall_turn_direction * self.wall_turn_speed)
            return

        if self.should_avoid_wall():
            self.start_wall_turn()
            self.publish_smoothed_command(0.0, self.wall_turn_direction * self.wall_turn_speed)
            return

        if self.state == "START_TURN_RIGHT_45":
            self.run_start_turn_right_45()
            return

        if self.state == "GO_STRAIGHT":
            if self.target_found and self.target_distance < 1.2:
                self.start_state("GREEN_FOLLOW")
                target_v, target_w = self.compute_green_follow_command()
            else:
                target_v, target_w = self.limit_linear_vel, 0.0
            self.publish_smoothed_command(target_v, target_w)
            return

        if self.target_found:
            sensor1 = 0
            sensor2 = 0
            sensor3 = 0
            target_v, target_w = self.simple_green_distance_command(sensor1, sensor2, sensor3)
        else:
            target_v, target_w = 0.0, 0.8

        self.publish_smoothed_command(target_v, target_w)

    def motion_text(self, target_v, target_w):
        if self.wall_turning:
            return f"WALL AVOID: turn {self.wall_turn_direction_text()} 90 deg"

        if self.opening_state == "TURN_LEFT_45":
            return "OPENING: turn left 45 deg"

        if self.opening_state == "GO_STRAIGHT":
            if self.wall_is_close():
                return "WALL: stop"
            return "OPENING: go straight until wall or green"

        if self.state == "START_TURN_RIGHT_45":
            direction = "left" if target_w > 0.0 else "right"
            return f"GREEN FAR: turn {direction} 45 deg"

        if self.state == "GO_STRAIGHT" and self.wall_is_close() and not self.target_found:
            return "WALL: stop"

        if self.target_found and abs(target_v) > 0.05 and abs(target_w) < 0.05:
            return "GREEN FOLLOW: straight"

        if self.target_found and abs(target_v) > 0.05 and target_w > 0.05:
            return "GREEN FOLLOW: straight + left correction"

        if self.target_found and abs(target_v) > 0.05 and target_w < -0.05:
            return "GREEN FOLLOW: straight + right correction"

        if abs(target_v) > 0.05 and abs(target_w) < 0.05:
            return "STRAIGHT"

        if abs(target_v) < 0.05 and target_w > 0.05:
            return "TURN LEFT"

        if abs(target_v) < 0.05 and target_w < -0.05:
            return "TURN RIGHT"

        if abs(target_v) > 0.05 and target_w > 0.05:
            return "STRAIGHT + LEFT"

        if abs(target_v) > 0.05 and target_w < -0.05:
            return "STRAIGHT + RIGHT"

        return "STOP"

    def log_motion_text(self, target_v, target_w):
        motion = self.motion_text(target_v, target_w)
        now = self.get_clock().now()
        elapsed = (now - self.last_motion_log_time).nanoseconds / 1e9

        if motion == self.last_motion_text and elapsed < 1.0:
            return

        self.last_motion_text = motion
        self.last_motion_log_time = now
        self.get_logger().info(
            f"{motion} | v={target_v:.2f}, w={target_w:.2f}, "
            f"green={self.target_found}, distance={self.target_distance:.2f}m, "
            f"wall L={self.wall_left_distance:.2f}m R={self.wall_right_distance:.2f}m"
        )

    def publish_smoothed_command(self, target_v, target_w):
        twist = Twist()

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

        self.current_v = max(min(self.current_v, self.limit_linear_vel), -self.limit_linear_vel)
        self.current_w = max(min(self.current_w, self.limit_angular_vel), -self.limit_angular_vel)

        twist.linear.x = self.current_v
        twist.angular.z = self.current_w
        self.pub_vel.publish(twist)
        self.log_motion_text(self.current_v, self.current_w)

def main(args=None):
    rclpy.init(args=args)
    node = OnlyGreenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
