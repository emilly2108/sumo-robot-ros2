# -*- coding: utf-8 -*-

import gc
import json
import os
from pathlib import Path
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

from .i2c_guard import (
    BoundedI2C, BoundedTCAChannel, I2C_INITIALIZATION_TIMEOUT_SEC,
    check_initialization_deadline,
)

from .hfsm_brain.robot_settings import load_section
from .color_i2c import LinuxRGBCReader

COLOR_KEYS = (
    'integration_time_ms', 'gain', 'loop_delay_sec',
    'red_r_min', 'red_g_max', 'red_b_max', 'blue_b_min', 'blue_r_max',
    'max_consecutive_failures',
)


BLACK = 0
RED = 1
BLUE = 2


I2C_LOCK_TIMEOUT_SEC = 0.01
I2C_RECOVERY_RETRY_SEC = float(os.environ.get("SUMO_COLOR_RECOVERY_RETRY_SEC", "0.25"))
if I2C_RECOVERY_RETRY_SEC <= 0:
    raise ValueError("Color recovery retry interval must be positive")
I2C_RECOVERY_LOG_INTERVAL_SEC = 10.0
I2C_ERROR_GRACE_SEC = 2.0
I2C_ERROR_PROBE_SEC = 0.10
I2C_FRESH_CONFIRMATIONS = 3


class ColorInitializationError(RuntimeError):
    """A sensor could not be initialized; keep publishing unavailable data."""


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

        self.publisher = self.create_publisher(String, "/color_sensor", 1)
        self.last_logged_colors = None
        self.last_complete_colors = None
        self.last_complete_sample_ns = 0
        self.consecutive_failures = 0

        self.i2c = None
        self.tca = None
        self.sensors = {}
        self.rgbc_readers = {}
        self.sensors_ready = False
        self.recovery_attempts = 0
        self.next_recovery_at = 0.0
        self.last_recovery_log_at = float("-inf")
        self.last_recovery_error = None
        self.initialization_stage = "open I2C bus"
        self.node_created_ns = time.monotonic_ns()
        self.initialization_started_ns = 0
        self.initialized_ns = 0
        self.first_valid_published_ns = 0
        self.state = "STARTING"
        self.error_started_ns = 0
        self.next_probe_ns = 0
        self.fresh_confirmations = 0
        self.confirming_fresh = True
        self.reinitialization_count = 0
        self.last_status_write_ns = 0
        self.status_path = os.environ.get("SUMO_COLOR_STATUS_FILE", "")
        self.status_error_reported = False
        self._write_status(force=True)

        # Keep the ROS node alive while hardware is unavailable. The brain's
        # existing freshness guard keeps velocity zero until a complete NEW
        # three-sensor sample arrives; cached readings never become fresh.
        self.timer = self.create_timer(self.loop_delay_sec, self.timer_callback)
        self._publish_unavailable()
        try:
            self._attempt_initialization()
        except BaseException:
            self._close_i2c(deactivate=False)
            super().destroy_node()
            raise

    def _initialize_sensors(self):
        enabled_channels = {
            channel for channel, config in self.channel_config.items() if config["enabled"]
        }
        try:
            initialization_deadline = time.monotonic() + I2C_INITIALIZATION_TIMEOUT_SEC
            self._set_stage("open I2C bus")
            self.i2c = BoundedI2C(busio.I2C(board.SCL, board.SDA))
            self._set_stage("TCA9548A address 0x70")
            self.tca = adafruit_tca9548a.TCA9548A(self.i2c)
            for channel in sorted(enabled_channels):
                config = self.channel_config[channel]
                self._set_stage(f"CH{channel} ({config['position']}) initialization")
                check_initialization_deadline(initialization_deadline)
                sensor = adafruit_tcs34725.TCS34725(BoundedTCAChannel(self.tca, channel))
                self.sensors[channel] = sensor
                self._set_stage(f"CH{channel} integration-time register")
                sensor.integration_time = self.integration_time_ms
                self._set_stage(f"CH{channel} gain register")
                sensor.gain = self.gain
                self._set_stage(f"CH{channel} ADC enable register")
                sensor.active = True

            # The ADCs integrate independently while the TCA channel is closed.
            # Wait for fresh conversions before the first burst read.
            actual_ms = next(iter(self.sensors.values())).integration_time
            time.sleep(max(0.025, 2 * actual_ms / 1000.0))
            for attempt in range(1, self.max_consecutive_failures + 1):
                not_ready = []
                for channel, sensor in self.sensors.items():
                    self._set_stage(f"CH{channel} first conversion")
                    check_initialization_deadline(initialization_deadline)
                    if not sensor._valid():
                        not_ready.append(channel)
                if not not_ready:
                    break
                if attempt < self.max_consecutive_failures:
                    time.sleep(self.loop_delay_sec)
            else:
                raise RuntimeError(f"First color conversion not valid: channels {not_ready}")

            self.rgbc_readers = {
                channel: LinuxRGBCReader.from_sensor(sensor)
                for channel, sensor in self.sensors.items()
            }
            # Conversion-ready flags alone are insufficient: verify an actual
            # complete read before declaring this hardware session ready.
            self._set_stage("verify complete three-sensor read")
            for channel, reader in self.rgbc_readers.items():
                check_initialization_deadline(initialization_deadline)
                self._set_stage(f"CH{channel} recovery verification read")
                reader.read_rgbc()
        except (OSError, RuntimeError, ValueError) as exc:
            raise ColorInitializationError(f"{self.initialization_stage}: {exc}") from exc

    def _attempt_initialization(self):
        self.recovery_attempts += 1
        self.state = "INITIALIZING"
        self.initialization_started_ns = time.monotonic_ns()
        self.initialized_ns = 0
        self.first_valid_published_ns = 0
        self._write_status(force=True)
        try:
            self._initialize_sensors()
        except ColorInitializationError as exc:
            self.sensors_ready = False
            self.state = "RECOVERY_WAIT"
            # A failed bus can cost one kernel timeout per cleanup write.
            # Close descriptors without repeatedly issuing known-failing writes.
            self._close_i2c(deactivate=False)
            now = time.monotonic()
            self.next_recovery_at = now + I2C_RECOVERY_RETRY_SEC
            reason = str(exc)
            if (reason != self.last_recovery_error
                    or now - self.last_recovery_log_at >= I2C_RECOVERY_LOG_INTERVAL_SEC):
                self.get_logger().error(
                    f"컬러센서 초기화/복구 대기 #{self.recovery_attempts}: {reason}; "
                    f"{I2C_RECOVERY_RETRY_SEC:g}초 후 재시도. 새 측정값 전까지 정지 유지. "
                    "지속되면 TCA·센서 RESET/전원 재투입 필요"
                )
                self.last_recovery_log_at = now
                self.last_recovery_error = reason
            self._publish_unavailable()
            self._write_status(force=True)
            return False
        self.sensors_ready = True
        self.initialized_ns = time.monotonic_ns()
        self.state = "VERIFYING"
        self.error_started_ns = 0
        self.next_probe_ns = 0
        self.fresh_confirmations = 0
        self.confirming_fresh = True
        self.consecutive_failures = 0
        self.last_recovery_error = None
        self.get_logger().info(
            f"컬러센서 3개 초기화·읽기 확인 완료 (시도 {self.recovery_attempts}); "
            f"새 묶음 {I2C_FRESH_CONFIRMATIONS}회 확인 후 유효 발행, period={self.loop_delay_sec:.3f}s"
        )
        self._write_status(force=True)
        return True

    def _set_stage(self, stage):
        self.initialization_stage = stage
        self._write_status(force=True)

    def _write_status(self, force=False):
        """Private supervisor IPC; do not change the ROS color message schema."""
        if not self.status_path:
            return
        now = time.monotonic_ns()
        if not force and now-self.last_status_write_ns < 250_000_000:
            return
        data = {"pid": os.getpid(), "progress_ns": now, "state": self.state,
                "last_valid_ns": self.last_complete_sample_ns,
                "error_started_ns": self.error_started_ns,
                "reinitialization_count": self.reinitialization_count,
                "initialization_attempts": self.recovery_attempts,
                "node_created_ns": getattr(self, "node_created_ns", 0),
                "initialization_started_ns": getattr(self, "initialization_started_ns", 0),
                "initialized_ns": getattr(self, "initialized_ns", 0),
                "first_valid_published_ns": getattr(self, "first_valid_published_ns", 0),
                "stage": self.initialization_stage}
        path = Path(self.status_path)
        temporary = path.with_name(path.name+f".{os.getpid()}.tmp")
        try:
            temporary.write_text(json.dumps(data))
            os.replace(temporary, path)
            self.last_status_write_ns = now
        except OSError as exc:
            if not self.status_error_reported:
                self.get_logger().warning(f"컬러 복구 관리자 상태 기록 실패: {exc}")
                self.status_error_reported = True

    def _publish_unavailable(self):
        colors = self.last_complete_colors or {
            config["position"]: BLACK for config in self.channel_config.values()
            if config["enabled"]
        }
        self._publish_colors(colors, False, self.last_complete_sample_ns)

    def _begin_recovery(self, reason):
        self.sensors_ready = False
        self.state = "RECOVERY_WAIT"
        self.reinitialization_count += 1
        self.recovery_attempts = 0
        self.last_recovery_error = None
        self._close_i2c(deactivate=False)
        # The two-second observation period already elapsed; do not add a
        # second two-second wait before the first reinitialization attempt.
        self.next_recovery_at = time.monotonic()
        self._write_status(force=True)
        self.get_logger().error(
            f"컬러센서 I²C 장애: {reason}; 센서 재초기화로 전환. "
            "새 3개 측정값 전까지 정지 유지, ROS 노드는 계속 실행"
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
        self._write_status()
        if not self.sensors_ready:
            self._publish_unavailable()
            if time.monotonic() >= self.next_recovery_at:
                self._attempt_initialization()
            return
        if time.monotonic_ns() < self.next_probe_ns:
            self._publish_unavailable()
            return
        # Defer automatic collection only until the new batch has been published.
        # Collection remains enabled between callbacks; real old data still times out.
        gc_was_enabled = gc.isenabled()
        if gc_was_enabled:
            gc.disable()
        try:
            sensor_colors = self._read_and_publish_batch()
        finally:
            if gc_was_enabled:
                gc.enable()
            self._write_status()
        if sensor_colors is not None and sensor_colors != self.last_logged_colors:
            self.get_logger().info(f"sensors: {sensor_colors}")
            self.last_logged_colors = dict(sensor_colors)

    def _read_and_publish_batch(self):
        # Use the start of the three-sensor read as a conservative batch age.
        batch_read_started_ns = time.monotonic_ns()
        sensor_colors = {}

        for channel, sensor in self.sensors.items():
            position = self.channel_config[channel]["position"]
            try:
                r, g, b, c = self.rgbc_readers[channel].read_rgbc()

                # 측정 비율을 BLACK=0, RED=1, BLUE=2로 변환한다.
                color = self.classify_color(r, g, b, c)
                sensor_colors[position] = color

            except OSError as e:
                self.consecutive_failures += 1
                self._publish_unavailable()
                reason = f"CH{channel} ({position}): {e}"
                now_ns = time.monotonic_ns()
                if not self.error_started_ns:
                    self.error_started_ns = self.last_complete_sample_ns or batch_read_started_ns
                    self.get_logger().warning(
                        f"컬러센서 오류 관찰 시작: {reason}; "
                        f"새 정상 데이터 {I2C_ERROR_GRACE_SEC:.0f}초 부재 시 재초기화"
                    )
                self.state = "DEGRADED"
                self.confirming_fresh = True
                self.fresh_confirmations = 0
                self.next_probe_ns = now_ns+int(I2C_ERROR_PROBE_SEC*1e9)
                if now_ns-self.error_started_ns >= I2C_ERROR_GRACE_SEC*1e9:
                    self._begin_recovery(reason)
                self._write_status(force=True)
                return

        self.consecutive_failures = 0
        self.next_probe_ns = 0
        if self.confirming_fresh:
            self.fresh_confirmations += 1
            if self.fresh_confirmations < I2C_FRESH_CONFIRMATIONS:
                self.state = "VERIFYING"
                self._publish_unavailable()
                return
            self.confirming_fresh = False
            self.get_logger().info("컬러센서 새 3축 묶음 연속 확인 완료: 유효 데이터 재개")
        self.state = "RUNNING"
        self.error_started_ns = 0
        self.last_complete_colors = dict(sensor_colors)
        self.last_complete_sample_ns = batch_read_started_ns
        self._publish_colors(sensor_colors, True, self.last_complete_sample_ns)
        return sensor_colors

    def _publish_colors(self, sensor_colors, valid, sample_ns):
        msg = String()
        msg.data = json.dumps({"sensors": sensor_colors, "valid": valid,
                               "sample_monotonic_ns": sample_ns})
        self.publisher.publish(msg)
        if valid and not getattr(self, "first_valid_published_ns", 0):
            self.first_valid_published_ns = time.monotonic_ns()
            self._write_status(force=True)

    def _close_i2c(self, deactivate=True):
        for channel, sensor in (self.sensors.items() if deactivate else ()):
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
        self.sensors = {}
        self.rgbc_readers = {}
        self.tca = None

    def destroy_node(self):
        # 센서 읽기 주기를 먼저 멈추고 I2C 장치를 해제한 뒤 ROS 자원을 정리한다.
        # The first Ctrl+C starts cleanup; further Ctrl+C must not interrupt it.
        previous_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if hasattr(self, "timer"):
                self.timer.cancel()
            self._close_i2c(deactivate=self.sensors_ready)
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
        # SIGINT can invalidate rosout between ok() and logger.info().
        # The supervisor already records the normal child exit.
        print("[INFO] [color_node] Color sensor node stopped by user", flush=True)
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
