# -*- coding: utf-8 -*-

import json
import board
import busio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import adafruit_tca9548a
import adafruit_tcs34725


BLACK = 0
RED = 1
BLUE = 2


class TCS3472ColorNode(Node):
    def __init__(self):
        super().__init__("color_node")
        self.integration_time_ms = 10
        self.gain = 16

        self.loop_delay_sec = 0.05

        self.channel_config = {
            0: {
                "position": "front_left",
                "enabled": True,
            },
            1: {
                "position": "front_right",
                "enabled": True,
            },
            2: {
                "position": "back",
                "enabled": True,
            },
        }

        self.red_r_min = 0.50
        self.red_g_max = 0.30
        self.red_b_max = 0.30

        self.blue_b_min = 0.47
        self.blue_r_max = 0.20

        self.publisher = self.create_publisher(String, "/color_sensor", 10)

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.tca = adafruit_tca9548a.TCA9548A(self.i2c)
        self.sensors = {}

        for channel, config in self.channel_config.items():
            if not config["enabled"]:
                self.get_logger().info(f"CH{channel}: disabled, reserved for later")
                continue

            try:
                sensor = adafruit_tcs34725.TCS34725(self.tca[channel])
                sensor.integration_time = self.integration_time_ms
                sensor.gain = self.gain

                self.sensors[channel] = sensor
                self.get_logger().info(
                    f"CH{channel}: sensor connected at {config['position']}"
                )

            except Exception as e:
                self.get_logger().error(f"CH{channel}: sensor init failed: {e}")


        self.timer = self.create_timer(self.loop_delay_sec, self.timer_callback)
        self.get_logger().info("TCS3472/TCS34725 floor color node started")

    def classify_color(self, r, g, b, c):
        if c <= 0:
            return BLACK

        r_ratio = r / c
        g_ratio = g / c
        b_ratio = b / c

        if (
            r_ratio > self.red_r_min
            and g_ratio < self.red_g_max
            and b_ratio < self.red_b_max
        ):
            return RED

        if b_ratio > self.blue_b_min and r_ratio < self.blue_r_max:
            return BLUE

        return BLACK

    def timer_callback(self):
        sensor_colors = {}
        failed_positions = []

        for channel, sensor in self.sensors.items():
            position = self.channel_config[channel]["position"]
            try:
                r, g, b, c = sensor.color_raw

                # 측정 비율을 BLACK=0, RED=1, BLUE=2로 변환한다.
                color = self.classify_color(r, g, b, c)
                sensor_colors[position] = color

            except Exception as e:
                self.get_logger().error(f"CH{channel}: read error: {e}")
                failed_positions.append(position)

        # 세 값이 같은 시점의 묶음으로 유지되도록 하나라도 읽지 못하면
        # 이번 주기는 발행하지 않고 Brain이 직전의 완전한 묶음을 유지하게 한다.
        if failed_positions:
            self.get_logger().warning(
                f"Color batch not published; failed positions: {failed_positions}"
            )
            return

        if not sensor_colors:
            self.get_logger().warning("Color batch not published; no initialized sensors")
            return

        msg = String()
        msg.data = json.dumps({"sensors": sensor_colors})
        self.publisher.publish(msg)
        self.get_logger().info(f"sensors: {sensor_colors}")


def main(args=None):
    rclpy.init(args=args)
    node = TCS3472ColorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Color sensor node stopped by user")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()