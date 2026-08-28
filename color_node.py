# -*- coding: utf-8 -*-

import json
import board
import busio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import adafruit_tca9548a
import adafruit_tcs34725


class TCS3472ColorNode(Node):
    def __init__(self):
        super().__init__("color_node")

        # =========================
        # 파라미터 설정
        # =========================

        # 센서 측정 설정값
        self.integration_time_ms = 10
        self.gain = 16

        # 센서를 읽는 주기
        self.loop_delay_sec = 0.05

        # TCA 채널과 로봇에서의 센서 위치 매핑
        self.channel_config = {
            0: {
                "position": "unused_ch0",
                "enabled": False,
            },
            1: {
                "position": "front_right",
                "enabled": True,
            },
            2: {
                "position": "back",
                "enabled": True,
            },
            3: {
                "position": "front_left",
                "enabled": True,
            },
        }

        # 빨간색 판정 기준
        self.red_r_min = 0.50
        self.red_g_max = 0.30
        self.red_b_max = 0.30

        # 파란색 판정 기준
        self.blue_b_min = 0.47
        self.blue_r_max = 0.20

        # =========================
        # ROS 설정
        # =========================

        # 바닥 색 인식 결과를 JSON 문자열로 publish
        self.publisher = self.create_publisher(String, "/color_sensor", 10)

        # =========================
        # I2C / 센서 설정
        # =========================

        # 라즈베리파이 I2C 버스 생성
        self.i2c = busio.I2C(board.SCL, board.SDA)

        # TCA9548A I2C 멀티플렉서 생성
        self.tca = adafruit_tca9548a.TCA9548A(self.i2c)

        # 연결된 센서를 저장할 딕셔너리
        self.sensors = {}

        # 활성화된 채널의 TCS3472/TCS34725 센서 초기화
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

        # 주기적으로 센서값을 읽는 타이머 생성
        self.timer = self.create_timer(self.loop_delay_sec, self.timer_callback)

        self.get_logger().info("TCS3472/TCS34725 floor color node started")

    def classify_color(self, r, g, b, c):
        # C 값이 0이면 비율 계산이 불가능하므로 UNKNOWN 처리
        if c <= 0:
            return "UNKNOWN"

        # 전체 밝기 C를 기준으로 RGB 비율 계산
        r_ratio = r / c
        g_ratio = g / c
        b_ratio = b / c

        # 빨간색 바닥 영역 판정
        if (
            r_ratio > self.red_r_min
            and g_ratio < self.red_g_max
            and b_ratio < self.red_b_max
        ):
            return "RED"

        # 파란색 바닥 영역 판정
        if b_ratio > self.blue_b_min and r_ratio < self.blue_r_max:
            return "BLUE"

        # 기본 바닥 또는 미분류 색상
        return "BLACK_OR_UNKNOWN"

    def timer_callback(self):
        # 각 센서에서 raw 값을 한 번만 읽고 색을 판정한 뒤 publish
        for channel, sensor in self.sensors.items():
            try:
                r, g, b, c = sensor.color_raw

                color = self.classify_color(r, g, b, c)
                position = self.channel_config[channel]["position"]

                # 토픽으로 보낼 최소 데이터 구성
                data = {
                    "position": position,
                    "color": color,
                }

                msg = String()
                msg.data = json.dumps(data)
                self.publisher.publish(msg)

                # 터미널에도 위치와 색만 출력
                self.get_logger().info(f"{position}: {color}")

            except Exception as e:
                self.get_logger().error(f"CH{channel}: read error: {e}")


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
