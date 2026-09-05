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

        # 센서를 읽고 /color_sensor를 발행하는 주기: 초당 10회
        self.loop_delay_sec = 0.10

        # TCA 채널과 로봇에서의 센서 위치 매핑
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
        self.publisher = self.create_publisher(String, "/color_sensor", 1)

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

    def color_to_value(self, color_name):
        """Brain이 사용하는 0=검정, 1=빨강, 2=파랑 값으로 바꾼다."""
        if color_name == "RED":
            return 1
        if color_name == "BLUE":
            return 2
        return 0

    def timer_callback(self):
        # 세 센서 결과를 한 메시지에 묶어 발행한다. 큐가 1이어도 앞 센서 값이
        # 뒤 센서 메시지에 밀려 사라지지 않도록 하기 위한 구조다.
        sensor_values = {}
        for channel, config in self.channel_config.items():
            if not config["enabled"]:
                continue

            position = config["position"]
            sensor = self.sensors.get(channel)
            if sensor is None:
                # 초기화에 실패한 센서는 검정으로 보내서 이전 색이 남지 않게 한다.
                sensor_values[position] = 0
                continue

            try:
                r, g, b, c = sensor.color_raw

                color_name = self.classify_color(r, g, b, c)
                color_value = self.color_to_value(color_name)
                sensor_values[position] = color_value

            except Exception as e:
                self.get_logger().error(f"CH{channel}: read error: {e}")
                # 읽기에 실패한 주기는 검정으로 보내 이전 색상 값이 남지 않게 한다.
                sensor_values[position] = 0

        if not sensor_values:
            return

        msg = String()
        msg.data = json.dumps({"sensors": sensor_values})
        self.publisher.publish(msg)
        self.get_logger().info(
            "colors: "
            f"front_left={sensor_values.get('front_left', 0)} "
            f"front_right={sensor_values.get('front_right', 0)} "
            f"back={sensor_values.get('back', 0)}"
        )


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
