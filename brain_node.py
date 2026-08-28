import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class GreenFollowerNode(Node):
    def __init__(self):
        super().__init__('brain_node')

        # Motion limits
        self.limit_linear_vel = 0.6
        self.limit_angular_vel = 2.0
        self.accel_linear = 0.12
        self.accel_angular = 0.25

        # Control gains
        self.Kp = 1.8
        self.deadzone = 0.03
        self.dt = 0.02

        # Vision state
        self.target_error = 0.0
        self.target_distance = 9.9
        self.target_found = False
        self.last_seen = self.get_clock().now()

        # Smoothed command state
        self.current_v = 0.0
        self.current_w = 0.0

        # Subscriptions from vision node
        self.create_subscription(Float32, '/target_error', self.error_callback, 10)
        self.create_subscription(Float32, '/target_distance', self.distance_callback, 10)
        self.create_subscription(Bool, '/target_found', self.found_callback, 10)

        # Publish velocity commands to motor node
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('Green follower brain node started.')

    def error_callback(self, msg):
        self.target_error = msg.data
        self.last_seen = self.get_clock().now()

    def distance_callback(self, msg):
        self.target_distance = msg.data
        self.last_seen = self.get_clock().now()

    def found_callback(self, msg):
        self.target_found = msg.data
        self.last_seen = self.get_clock().now()

    def compute_command(self):
        if self.target_found:
            target_v = self.limit_linear_vel
            target_w = -self.Kp * self.target_error

            if abs(self.target_error) < self.deadzone:
                target_w = 0.0

            if self.target_distance > 1.5:
                target_v = self.limit_linear_vel
            elif self.target_distance > 0.6:
                target_v = self.limit_linear_vel * 0.7
            else:
                target_v = self.limit_linear_vel * 0.35

            return target_v, target_w

        return 0.0, 0.8

    def control_loop(self):
        stale_time = (self.get_clock().now() - self.last_seen).nanoseconds / 1e9
        if stale_time > 0.5:
            self.target_found = False

        target_v, target_w = self.compute_command()
        self.publish_smoothed_command(target_v, target_w)

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


def main(args=None):
    rclpy.init(args=args)
    node = GreenFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
