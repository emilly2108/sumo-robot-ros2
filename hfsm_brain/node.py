import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool

from .actions import (
    Action,
    All_Blue_Recovery_Action,
    All_Red_Recovery_Action,
    Cruise_Action,
    Far_Green_Turn_Action,
    Front_Color_Avoid_Action,
    Green_Follow_Action,
    Opening_Action,
    Back_Single_Boost_Action,
    Wall_Avoid_Action,
)
from .helpers import now_seconds, wall_avoid_direction
from .models import (
    SENSOR_EVENTS,
    Brain_Config,
    Motion_Command,
    Sensor_Color,
    Sensor_Event,
    World_Model,
)

class Hfsm_Brain_Node(Node):
    def __init__(self):
        super().__init__("brain_hfsm_node")
        self.config = Brain_Config()
        self.world = World_Model()
        # 스위치가 켜져서 switch_mode=True 요청을 받을 때까지 로봇을 정지시킨다.
        self.switch_enabled = False
        self.opening_done = False
        self.far_green_turn_done = False
        self.active_action: Action = Opening_Action(self.config)
        self.active_action.enter(now_seconds(self), self.world)

        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_service(SetBool, "switch_mode", self.switch_mode_callback)

        self.create_subscription(Float32, "/target_error", self.target_error_callback, 10)
        self.create_subscription(
            Float32, "/target_distance", self.target_distance_callback, 10
        )
        self.create_subscription(Bool, "/target_found", self.target_found_callback, 10)
        self.create_subscription(
            Float32, "/wall_distance", self.wall_distance_callback, 10
        )
        self.create_subscription(
            Float32, "/wall_left_distance", self.wall_left_callback, 10
        )
        self.create_subscription(
            Float32, "/wall_right_distance", self.wall_right_callback, 10
        )
        # color_node.py가 두 앞 센서의 결과를 JSON String 하나로 publish한다.
        self.create_subscription(String, "/color_sensor", self.color_sensor_callback, 10)

        self.last_command_label = None
        self.timer = self.create_timer(self.config.control_period, self.control_loop)
        self.get_logger().info("HFSM brain started: action=OPENING")

    def target_error_callback(self, msg: Float32) -> None:
        self.world.target_error = float(msg.data)

    def target_distance_callback(self, msg: Float32) -> None:
        self.world.target_distance = float(msg.data)

    def target_found_callback(self, msg: Bool) -> None:
        if self.world.wall_frame_received:
            self.world.wall_missing_frames = 0
        else:
            self.world.wall_missing_frames += 1
        self.world.wall_frame_received = False

        if msg.data:
            self.world.target_found = True
            self.world.target_missing_frames = 0
            return

        self.world.target_missing_frames += 1

    def wall_distance_callback(self, msg: Float32) -> None:
        self.world.wall_distance = float(msg.data)
        self.world.wall_frame_received = True

    def wall_left_callback(self, msg: Float32) -> None:
        self.world.wall_left_distance = float(msg.data)

    def wall_right_callback(self, msg: Float32) -> None:
        self.world.wall_right_distance = float(msg.data)

    def switch_mode_callback(self, request: SetBool.Request, response: SetBool.Response):
        """switch_node.py의 물리 스위치 요청을 받아 Brain 실행 여부를 바꾼다."""
        enabled = bool(request.data)

        if enabled == self.switch_enabled:
            response.success = True
            response.message = "Brain mode is already " + ("ON" if enabled else "OFF")
            return response

        self.switch_enabled = enabled

        if enabled:
            # 다시 켜면 경기 시작 동작부터 새로 시작한다.
            self.opening_done = False
            self.far_green_turn_done = False
            self.active_action.exit(cancelled=True)
            self.active_action = Opening_Action(self.config)
            self.active_action.enter(now_seconds(self), self.world)
            response.message = "Brain mode enabled; opening action restarted"
        else:
            # OFF 상태에서는 control_loop가 Soft Stop을 계속 유지한다.
            self.active_action.exit(cancelled=True)
            response.message = "Brain mode disabled; soft stop requested"

        response.success = True
        return response

    def color_sensor_callback(self, msg: String) -> None:
        """color_node.py의 JSON 색상 결과를 기존 sensor1~3 모델로 변환한다."""
        try:
            data = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.get_logger().warning(
                f"Ignored invalid /color_sensor JSON: {msg.data!r}"
            )
            return

        if not isinstance(data, dict):
            self.get_logger().warning("Ignored /color_sensor message that is not an object")
            return

        # color_node.py의 위치 이름과 기존 World_Model 센서 번호를 연결한다.
        sensor_by_position = {
            "front_right": "sensor1",
            "front_left": "sensor2",
            # 현재 color_node.py에서는 CH2가 비활성화되어 있다.
            "back": "sensor3",
            "rear": "sensor3",
            "future_ch2": "sensor3",
        }
        color_by_name = {
            "BLACK": Sensor_Color.BLACK,
            "BLACK_OR_UNKNOWN": Sensor_Color.BLACK,
            "UNKNOWN": Sensor_Color.BLACK,
            "RED": Sensor_Color.RED,
            "BLUE": Sensor_Color.BLUE,
        }

        attr_name = sensor_by_position.get(data.get("position"))
        color_name = str(data.get("color", "")).upper()
        color = color_by_name.get(color_name)

        if attr_name is None or color is None:
            self.get_logger().warning(
                f"Ignored /color_sensor value: position={data.get('position')!r}, "
                f"color={data.get('color')!r}"
            )
            return

        self._set_sensor(attr_name, int(color))

    def _set_sensor(self, attr_name: str, value: int) -> None:
        try:
            color = Sensor_Color(value)
        except ValueError:
            self.get_logger().warning(
                f"Ignored invalid sensor value {value}; use BLACK=0, RED=1, BLUE=2"
            )
            return
        previous_color = getattr(self.world, attr_name)
        detected_at = now_seconds(self)
        if (
            attr_name == "sensor1"
            and color != Sensor_Color.BLACK
            and color != previous_color
        ):
            self.world.sensor1_detected_at = detected_at
        if (
            attr_name == "sensor2"
            and color != Sensor_Color.BLACK
            and color != previous_color
        ):
            self.world.sensor2_detected_at = detected_at
        setattr(self.world, attr_name, color)
        self.world.sensor_event = SENSOR_EVENTS.get(
            (self.world.sensor1, self.world.sensor2, self.world.sensor3),
            Sensor_Event.UNKNOWN,
        )
        self.world.sensor_updated_at = detected_at

    def refresh_timeouts(self, now: float) -> None:
        if self.world.target_missing_frames >= 10:
            self.world.target_found = False
            self.world.target_error = 0.0
            self.world.target_distance = 9.9
            self.far_green_turn_done = False
            self.world.target_missing_frames = 0

        if self.world.wall_missing_frames >= 10:
            self.world.wall_distance = self.config.no_wall_distance
            self.world.wall_left_distance = self.config.no_wall_distance
            self.world.wall_right_distance = self.config.no_wall_distance
            self.world.wall_missing_frames = 0

    def transition_to(self, next_action: Action, now: float, cancelled: bool = False) -> None:
        if self.active_action.key() == next_action.key():
            return
        old_name = self.active_action.name
        # 기존 행동에 정상 종료인지 강제 취소인지 알려 정리할 기회를 준다.
        self.active_action.exit(cancelled)
        # 현재 활성 행동 참조를 새 행동 객체로 교체한다.
        self.active_action = next_action
        # 새 행동의 단계와 시작 시각을 현재 World Model로 초기화한다.
        self.active_action.enter(now, self.world)
        self.get_logger().info(f"HFSM: {old_name} -> {self.active_action.name}")

    def choose_action(self) -> Action:
        # 벽 회피를 초록색과 센서 패턴보다 먼저 선택한다.
        direction = wall_avoid_direction(self.world, self.config)
        if direction is not None:
            return Wall_Avoid_Action(self.config, direction)

        if self.world.target_found:
            return Green_Follow_Action(self.config)

        event = self.world.sensor_event
        if event == Sensor_Event.FRONT_BOTH_RED:
            return Front_Color_Avoid_Action(self.config, Sensor_Color.RED, None)
        if event == Sensor_Event.FRONT_BOTH_BLUE:
            return Front_Color_Avoid_Action(self.config, Sensor_Color.BLUE, None)
        if event in (Sensor_Event.BACK_RED, Sensor_Event.BACK_BLUE):
            return Back_Single_Boost_Action(self.config, event)
        if event == Sensor_Event.ALL_RED:
            return All_Red_Recovery_Action(self.config)
        if event == Sensor_Event.ALL_BLUE:
            return All_Blue_Recovery_Action(self.config)

        return Cruise_Action(self.config)

    def publish_command(self, command: Motion_Command, now: float) -> None:
        if (
            self.world.target_found
            and 0.0 < self.world.target_distance <= 0.40
            and command.linear in (self.config.base_speed, self.config.max_speed)
        ):
            command = Motion_Command(
                linear=self.config.battle_speed,
                angular=command.angular,
                hard_stop=command.hard_stop,
                label="GREEN_BATTLE_OVERRIDE",
            )

        if command.hard_stop:
            # Hard Stop 서비스 대신 0 속도 명령을 계속 publish한다.
            # control_loop가 20ms마다 이 함수를 호출하므로 정지 명령도 반복 전송된다.
            soft_stop = Twist()
            soft_stop.linear.x = 0.0
            soft_stop.angular.z = 0.0
            self.pub_vel.publish(soft_stop)
            self._log_command(command)
            return

        twist = Twist()
        twist.linear.x = max(
            min(command.linear, self.config.max_speed), -self.config.max_speed
        )
        twist.angular.z = max(
            min(command.angular, self.config.max_angular_speed),
            -self.config.max_angular_speed,
        )
        self.pub_vel.publish(twist)
        self._log_command(command)

    def _log_command(self, command: Motion_Command) -> None:
        if command.label == self.last_command_label:
            return
        self.last_command_label = command.label
        self.get_logger().info(
            f"ACTION={self.active_action.name} COMMAND={command.label} "
            f"v={command.linear:.2f} w={command.angular:.2f} "
            f"green={self.world.target_found} "
            f"distance={self.world.target_distance:.2f}m "
            f"sensor={self.world.sensor_event.name}"
        )

    def control_loop(self) -> None:
        now = now_seconds(self)
        self.refresh_timeouts(now)

        if not self.switch_enabled:
            self.publish_command(
                Motion_Command(hard_stop=True, label="SWITCH_OFF"),
                now,
            )
            return

        # 초록색이 보여도 벽이 가까우면 벽 회피를 먼저 시작한다.
        wall_direction = wall_avoid_direction(self.world, self.config)
        if (
            wall_direction is not None
            and not isinstance(self.active_action, Wall_Avoid_Action)
        ):
            self.transition_to(
                Wall_Avoid_Action(self.config, wall_direction),
                now,
                cancelled=True,
            )

        if (
            self.active_action.locked
            and self.active_action.can_interrupt_for_green()
            and self.world.target_found
            and wall_direction is None
            # 이미 초록색 추격 중인 행동을 다시 같은 행동으로 바꾸지 않는다.
            and not isinstance(self.active_action, Green_Follow_Action)
        ):
            # 현재 행동에 cancelled=True를 알리고 초록색 추격 행동으로 전환한다.
            self.transition_to(Green_Follow_Action(self.config), now, cancelled=True)

        if not self.active_action.locked:
            self.transition_to(self.choose_action(), now)

        # 짧은 완료 상태 때문에 20ms를 추가 대기하지 않도록 한 주기에서 최대 3회 전환한다.
        # 무한 전환을 막기 위해 while문 대신 반복 횟수가 제한된 for문을 사용한다.
        for _ in range(3):
            # 현재 행동의 이번 단계 명령과 완료 여부를 계산한다.
            step = self.active_action.step(now, self.world)
            # 현재 행동이 아직 끝나지 않았는지 확인한다.
            if not step.finished:
                # 행동이 만든 이동 또는 Hard Stop 명령을 실제 ROS2 출력으로 보낸다.
                self.publish_command(step.command, now)
                # 이번 20ms 제어 주기에서 해야 할 일을 끝냈으므로 반환한다.
                return

            if isinstance(self.active_action, Opening_Action):
                self.opening_done = True
            # 방금 완료된 행동이 먼 초록색용 45도 회전인지 확인한다.
            if isinstance(self.active_action, Far_Green_Turn_Action):
                # 같은 초록색 목표에서 이 회전이 반복되지 않도록 완료 기록을 남긴다.
                self.far_green_turn_done = True

            # 현재 행동이 정상적으로 완료됐으므로 cancelled=False로 종료 처리한다.
            self.active_action.exit(cancelled=False)
            self.active_action = self.choose_action()
            self.active_action.enter(now, self.world)

        # 한 제어 주기에서 세 번 연속 즉시 완료되면 상태 전환 순환 가능성을 알린다.
        self.get_logger().error("HFSM exceeded the transition limit in one control tick")

def main(args=None):
    rclpy.init(args=args)
    node = Hfsm_Brain_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
