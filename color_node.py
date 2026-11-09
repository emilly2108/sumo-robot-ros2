# -*- coding: utf-8 -*-

import json
import signal
import time
import board
import busio
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

import adafruit_tca9548a
import adafruit_tcs34725

from .hfsm_brain.robot_settings import load_section

COLOR_KEYS = (
    'integration_time_ms', 'gain', 'loop_delay_sec',
    'red_r_min', 'red_g_max', 'red_b_max', 'blue_b_min', 'blue_r_max',
    'max_consecutive_failures',
)


BLACK = 0
RED = 1
BLUE = 2


I2C_LOCK_TIMEOUT_SEC = 0.01


def bounded_i2c_transfer(sensor, output, input_buffer=None):
    """Select the mux channel with a bounded bus-lock wait, then transfer."""
    device = sensor._device
    channel = device.i2c
    bus = channel.tca.i2c
    deadline = time.monotonic() + I2C_LOCK_TIMEOUT_SEC
    while not bus.try_lock():
        if time.monotonic() >= deadline:
            raise TimeoutError("I2C bus lock unavailable for 10 ms")
        time.sleep(0.0001)
    try:
        bus.writeto(channel.tca.address, channel.channel_switch)
        if input_buffer is None:
            bus.writeto(device.device_address, output)
        else:
            bus.writeto_then_readfrom(device.device_address, output, input_buffer)
    finally:
        try:
            bus.writeto(channel.tca.address, b"\x00")
        finally:
            bus.unlock()


def read_rgbc_burst(sensor):
    """Read the TCS34725 C/R/G/B registers (0x14..0x1B) in one I2C transfer.

    The Pi's installed Adafruit driver reads four separate 16-bit registers in
    color_raw. Keep its data values and return order while reducing transfers.
    """
    data = bytearray(8)
    bounded_i2c_transfer(sensor, b"\x94", data)
    c = data[0] | (data[1] << 8)
    r = data[2] | (data[3] << 8)
    g = data[4] | (data[5] << 8)
    b = data[6] | (data[7] << 8)
    return r, g, b, c


class TCS3472ColorNode(Node):
    def __init__(self):
        super().__init__("color_node")
        # 모든 튜닝값은 robot_settings.yaml 의 color 섹션에서 읽는다.
        cfg = load_section("color", COLOR_KEYS)
        self.integration_time_ms = cfg["integration_time_ms"]
        self.gain = cfg["gain"]

        self.loop_delay_sec = float(cfg["loop_delay_sec"])
        self.max_consecutive_failures = cfg["max_consecutive_failures"]
        if self.loop_delay_sec <= 0:
            raise ValueError("color.loop_delay_sec must be positive")
        if (isinstance(self.max_consecutive_failures, bool)
                or not isinstance(self.max_consecutive_failures, int)
                or not 3 <= self.max_consecutive_failures <= 10):
            raise ValueError("color.max_consecutive_failures must be an integer from 3 to 10")

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

        self.red_r_min = float(cfg["red_r_min"])
        self.red_g_max = float(cfg["red_g_max"])
        self.red_b_max = float(cfg["red_b_max"])

        self.blue_b_min = float(cfg["blue_b_min"])
        self.blue_r_max = float(cfg["blue_r_max"])

        self.publisher = self.create_publisher(String, "/color_sensor", 10)
        self.last_logged_colors = None
        self.last_complete_colors = None
        self.last_complete_sample_ns = 0
        self.consecutive_failures = 0

        self.i2c = None
        self.sensors = {}
        enabled_channels = {
            channel for channel, config in self.channel_config.items() if config["enabled"]
        }
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.tca = adafruit_tca9548a.TCA9548A(self.i2c)
            for attempt in range(1, self.max_consecutive_failures + 1):
                for channel, config in self.channel_config.items():
                    if not config["enabled"] or channel in self.sensors:
                        continue
                    sensor = None
                    try:
                        sensor = adafruit_tcs34725.TCS34725(self.tca[channel])
                        sensor.integration_time = self.integration_time_ms
                        sensor.gain = self.gain
                        sensor.active = True
                        self.sensors[channel] = sensor
                        self.get_logger().info(
                            f"CH{channel}: sensor active at {config['position']}"
                        )
                    except Exception as exc:
                        if sensor is not None:
                            try:
                                sensor.active = False
                            except Exception:
                                pass
                        self.get_logger().error(
                            f"CH{channel}: init attempt {attempt}/"
                            f"{self.max_consecutive_failures} failed: {exc}"
                        )
                if len(self.sensors) == len(enabled_channels):
                    break
                if attempt < self.max_consecutive_failures:
                    time.sleep(self.loop_delay_sec)
            if len(self.sensors) != len(enabled_channels):
                missing = sorted(enabled_channels - set(self.sensors))
                raise RuntimeError(f"Color sensor initialization failed: channels {missing}")

            # The ADCs integrate independently while the TCA channel is closed.
            # Wait for fresh conversions before the first burst read.
            actual_ms = next(iter(self.sensors.values())).integration_time
            time.sleep(max(0.025, 2 * actual_ms / 1000.0))
            for attempt in range(1, self.max_consecutive_failures + 1):
                not_ready = []
                for channel, sensor in self.sensors.items():
                    try:
                        if not sensor._valid():
                            not_ready.append(channel)
                    except Exception as exc:
                        not_ready.append(channel)
                        self.get_logger().error(
                            f"CH{channel}: validity check {attempt}/"
                            f"{self.max_consecutive_failures} failed: {exc}"
                        )
                if not not_ready:
                    break
                if attempt < self.max_consecutive_failures:
                    time.sleep(self.loop_delay_sec)
            else:
                raise RuntimeError(f"First color conversion not valid: channels {not_ready}")
        except Exception:
            self._close_i2c()
            raise

        self.timer = self.create_timer(self.loop_delay_sec, self.timer_callback)
        self.get_logger().info(
            f"TCS34725 color node started: period={self.loop_delay_sec:.3f}s, "
            f"integration={actual_ms:.1f}ms, failure_limit={self.max_consecutive_failures}"
        )

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
        # Use the start of the three-sensor read as a conservative batch age.
        batch_read_started_ns = time.monotonic_ns()
        sensor_colors = {}

        for channel, sensor in self.sensors.items():
            position = self.channel_config[channel]["position"]
            try:
                r, g, b, c = read_rgbc_burst(sensor)

                # 측정 비율을 BLACK=0, RED=1, BLUE=2로 변환한다.
                color = self.classify_color(r, g, b, c)
                sensor_colors[position] = color

            except Exception as e:
                self.consecutive_failures += 1
                self.get_logger().error(
                    f"CH{channel} ({position}) read error "
                    f"{self.consecutive_failures}/{self.max_consecutive_failures}: {e}"
                )
                if self.last_complete_colors is not None:
                    # Preserve the last values, but identify them as stale.
                    self._publish_colors(self.last_complete_colors, False,
                                         self.last_complete_sample_ns)
                if self.consecutive_failures >= self.max_consecutive_failures:
                    raise RuntimeError(
                        f"Color sensor failed for {self.consecutive_failures} "
                        "consecutive frames"
                    ) from e
                return

        self.consecutive_failures = 0
        self.last_complete_colors = dict(sensor_colors)
        self.last_complete_sample_ns = batch_read_started_ns
        self._publish_colors(sensor_colors, True, self.last_complete_sample_ns)
        if sensor_colors != self.last_logged_colors:
            self.get_logger().info(f"sensors: {sensor_colors}")
            self.last_logged_colors = dict(sensor_colors)

    def _publish_colors(self, sensor_colors, valid, sample_ns):
        msg = String()
        msg.data = json.dumps({"sensors": sensor_colors, "valid": valid,
                               "sample_monotonic_ns": sample_ns})
        self.publisher.publish(msg)

    def _close_i2c(self):
        for channel, sensor in self.sensors.items():
            try:
                # ENABLE=0 turns off both power and ADC. Avoid the driver's
                # unbounded nested mux-lock loop during shutdown.
                bounded_i2c_transfer(sensor, b"\x80\x00")
            except Exception as exc:
                self.get_logger().warning(f"CH{channel}: deactivate failed: {exc}")
        if self.i2c is not None:
            try:
                self.i2c.deinit()
            except Exception as exc:
                self.get_logger().warning(f"I2C deinit failed: {exc}")
            self.i2c = None

    def destroy_node(self):
        # 센서 읽기 주기를 먼저 멈추고 I2C 장치를 해제한 뒤 ROS 자원을 정리한다.
        # The first Ctrl+C starts cleanup; further Ctrl+C must not interrupt it.
        previous_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if hasattr(self, "timer"):
                self.timer.cancel()
            self._close_i2c()
            return super().destroy_node()
        finally:
            signal.signal(signal.SIGINT, previous_sigint)


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = TCS3472ColorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None and rclpy.ok():
            node.get_logger().info("Color sensor node stopped by user")
    except ExternalShutdownException:
        # launch가 전체 ROS context를 먼저 종료한 정상 종료 경로다.
        pass

    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
