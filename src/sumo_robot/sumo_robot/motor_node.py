"""ROS cmd_vel -> SteadyWin four-axis CANopen CSV bridge."""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .canopen_drive import CanopenDrive, DriveError
from .hfsm_brain.robot_settings import load_section

MOTOR_KEYS = (
    "wheel_base_m", "wheel_radius_m", "encoder_counts_per_motor_rev", "gear_ratio",
    "max_motor_output_rpm", "node_ids", "left_node_ids", "right_node_ids",
    "can_channel", "can_period_sec", "sdo_timeout_sec",
)


class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_node")
        # 모든 튜닝값은 robot_settings.yaml 의 motor 섹션에서 읽는다.
        settings = load_section("motor", MOTOR_KEYS)
        value = settings.__getitem__
        self.wheel_base = float(value("wheel_base_m"))
        radius = float(value("wheel_radius_m"))
        counts, ratio = float(value("encoder_counts_per_motor_rev")), float(value("gear_ratio"))
        max_rpm = float(value("max_motor_output_rpm"))
        if any(not math.isfinite(v) or v <= 0
               for v in (self.wheel_base, radius, counts, ratio, max_rpm)):
            raise ValueError("차체/엔코더 변환 파라미터는 유한한 양수여야 합니다")
        self.scale = counts*ratio/(2*math.pi*radius)
        self.max_wheel_speed = max_rpm*2*math.pi*radius/60
        self.max_motor_output_rpm = max_rpm
        self.limit_warning_reported = False
        nodes = tuple(int(n) for n in value("node_ids"))
        self.left, self.right = tuple(value("left_node_ids")), tuple(value("right_node_ids"))
        if (not self.left or not self.right or set(self.left) & set(self.right)
                or set(self.left) | set(self.right) != set(nodes)):
            raise ValueError("좌/우 모터 ID는 겹치지 않고 전체 모터를 포함해야 합니다")
        self.last_rejection = None
        try:
            self.drive = CanopenDrive(
                channel=value("can_channel"), nodes=nodes, period=float(value("can_period_sec")),
                timeout=float(value("sdo_timeout_sec")),
                info=lambda text: self._log("info", text), error=lambda text: self._log("error", text),
                warning=lambda text: self._log("warning", text))
            self.drive.start()
        except (Exception, KeyboardInterrupt):
            super().destroy_node()
            raise
        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.velocity_callback, 1)

    def _log(self, level, text):
        if rclpy.ok():
            try:
                # Jazzy caches severity per call site: keep INFO/ERROR on separate lines.
                if level == "error":
                    self.get_logger().error(text)
                elif level == "warning":
                    self.get_logger().warning(text)
                else:
                    self.get_logger().info(text)
                return
            except Exception:
                # ROS may shut down between ok() and the logger call.
                pass
        print(f"[{level.upper()}] [motor_node] {text}", flush=True)

    def velocity_callback(self, msg):
        try:
            linear, angular = float(msg.linear.x), float(msg.angular.z)
            if not math.isfinite(linear) or not math.isfinite(angular):
                raise ValueError("cmd_vel의 linear.x/angular.z가 유효한 숫자가 아닙니다")
            left_mps = linear-angular*self.wheel_base/2
            right_mps = linear+angular*self.wheel_base/2
            if any(not math.isfinite(v) for v in (left_mps, right_mps)):
                raise ValueError("cmd_vel 변환값이 CAN signed32bit 표현 범위를 벗어났습니다")
            peak = max(abs(left_mps), abs(right_mps))
            if peak > self.max_wheel_speed:
                command_scale = self.max_wheel_speed/peak
                left_mps *= command_scale
                right_mps *= command_scale
                if not self.limit_warning_reported:
                    self.get_logger().warning(
                        f"cmd_vel 바퀴 속도 제한 적용: 요청 v={linear:.3f}m/s, "
                        f"w={angular:.3f}rad/s, 비율={command_scale:.3f}, "
                        f"모터 한계={self.max_motor_output_rpm:.1f}rpm "
                        "(동일 경고는 재표시하지 않음)")
                    self.limit_warning_reported = True
            left = left_mps*self.scale
            right = right_mps*self.scale
            targets = {n: -round(left) for n in self.left}
            targets.update({n: round(right) for n in self.right})
            self.drive.set_targets(targets)
            self.last_rejection = None
        except (ValueError, DriveError) as exc:
            reason = str(exc)
            if reason != self.last_rejection:
                self.get_logger().error("속도 명령 거절: " + reason)
                self.last_rejection = reason

    def destroy_node(self):
        self.drive.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except DriveError:
        # The CAN layer has already logged the exact failing step/object.
        return 1
    except Exception as exc:
        if rclpy.ok():
            rclpy.logging.get_logger("motor_node").error("모터 노드 시작 실패: " + str(exc))
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
