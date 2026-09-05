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
    Green_Close_All_Red_Action,
    Green_Close_Back_Blue_Action,
    Green_Close_Front_Color_Action,
    Green_Follow_Action,
    Green_Sensor_Follow_Action,
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

        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_service(SetBool, "switch_mode", self.switch_mode_callback)

        self.create_subscription(Float32, "/target_error", self.target_error_callback, 1)
        self.create_subscription(
            Float32, "/target_distance", self.target_distance_callback, 1
        )
        self.create_subscription(Bool, "/target_found", self.target_found_callback, 1)
        self.create_subscription(
            Float32, "/wall_distance", self.wall_distance_callback, 1
        )
        self.create_subscription(
            Float32, "/wall_left_distance", self.wall_left_callback, 1
        )
        self.create_subscription(
            Float32, "/wall_right_distance", self.wall_right_callback, 1
        )
        # color_node.py가 세 센서 결과를 JSON String 하나로 publish한다.
        self.create_subscription(String, "/color_sensor", self.color_sensor_callback, 1)

        # 같은 명령이 반복될 때 콘솔을 도배하지 않되, 행동·초록색·센서 상태 변화는 기록한다.
        self.last_command_state = None
        self.timer = self.create_timer(self.config.control_period, self.control_loop)
        self.get_logger().info("HFSM brain started: action=OPENING")

    def target_error_callback(self, msg: Float32) -> None:
        self.world.target_error = float(msg.data)

    def target_distance_callback(self, msg: Float32) -> None:
        self.world.target_distance = float(msg.data)

    def target_found_callback(self, msg: Bool) -> None:
        if msg.data:
            self.world.target_found = True
            self.world.target_missing_frames = 0
            return

        self.world.target_missing_frames += 1

    def wall_distance_callback(self, msg: Float32) -> None:
        self.world.wall_distance = float(msg.data)
        self.world.wall_last_received_at = now_seconds(self)

    def wall_left_callback(self, msg: Float32) -> None:
        self.world.wall_left_distance = float(msg.data)
        self.world.wall_last_received_at = now_seconds(self)

    def wall_right_callback(self, msg: Float32) -> None:
        self.world.wall_right_distance = float(msg.data)
        self.world.wall_last_received_at = now_seconds(self)

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
        """color_node.py의 색상 묶음을 World Model 센서 값으로 변환한다."""
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

        # 새 color_node.py는 세 센서를 {"sensors": {...}} 한 번에 보낸다.
        # 이전 단일 {"position": ..., "color": ...} 메시지도 계속 지원한다.
        batch = data.get("sensors")
        if isinstance(batch, dict):
            updates = {}
            for position, raw_color in batch.items():
                attr_name = sensor_by_position.get(position)
                color = self._parse_sensor_color(raw_color, color_by_name)
                if attr_name is None or color is None:
                    self.get_logger().warning(
                        f"Ignored /color_sensor batch value: position={position!r}, "
                        f"color={raw_color!r}"
                    )
                    continue
                updates[attr_name] = int(color)

            if updates:
                self._set_sensor_values(updates)
            else:
                self.get_logger().warning("Ignored /color_sensor batch without valid sensors")
            return

        attr_name = sensor_by_position.get(data.get("position"))
        raw_color = data.get("color")
        color = self._parse_sensor_color(raw_color, color_by_name)

        if attr_name is None or color is None:
            self.get_logger().warning(
                f"Ignored /color_sensor value: position={data.get('position')!r}, "
                f"color={raw_color!r}"
            )
            return

        self._set_sensor(attr_name, int(color))

    def _parse_sensor_color(self, raw_color, color_by_name):
        """문자열 또는 숫자 색상을 Sensor_Color로 안전하게 변환한다."""
        if isinstance(raw_color, bool):
            return None
        if isinstance(raw_color, (int, float)) and float(raw_color).is_integer():
            try:
                return Sensor_Color(int(raw_color))
            except ValueError:
                return None

        color_name = str(raw_color or "").upper()
        color = color_by_name.get(color_name)
        if color is None and color_name in {"0", "1", "2"}:
            return Sensor_Color(int(color_name))
        return color

    def _set_sensor(self, attr_name: str, value: int) -> None:
        self._set_sensor_values({attr_name: value})

    def _set_sensor_values(self, values) -> None:
        """한 색상 묶음을 동시에 반영하고 센서 이벤트를 한 번만 계산한다."""
        parsed_values = {}
        for attr_name, value in values.items():
            if attr_name not in {"sensor1", "sensor2", "sensor3"}:
                self.get_logger().warning(f"Ignored unknown sensor attribute {attr_name!r}")
                continue
            try:
                parsed_values[attr_name] = Sensor_Color(value)
            except ValueError:
                self.get_logger().warning(
                    f"Ignored invalid sensor value {value}; use BLACK=0, RED=1, BLUE=2"
                )

        if not parsed_values:
            return

        previous_event = self.world.sensor_event
        previous_values = (
            self.world.sensor1,
            self.world.sensor2,
            self.world.sensor3,
        )
        detected_at = now_seconds(self)
        for attr_name, color in parsed_values.items():
            previous_color = getattr(self.world, attr_name)
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

        current_values = (
            self.world.sensor1,
            self.world.sensor2,
            self.world.sensor3,
        )
        if current_values != previous_values or self.world.sensor_event != previous_event:
            self.get_logger().info(
                f"COLOR_INPUT={tuple(int(color) for color in current_values)} "
                f"SENSOR_EVENT={self.world.sensor_event.name} "
                f"green={self.world.target_found}"
            )

    def refresh_timeouts(self, now: float) -> None:
        # vision_node.py가 보내는 False 프레임 5개를 연속으로 받으면 초록색을 잃은 것으로 본다.
        if self.world.target_missing_frames >= 5:
            self.world.target_found = False
            self.world.target_error = 0.0
            self.world.target_distance = 9.9
            self.far_green_turn_done = False
            self.world.target_missing_frames = 0

        wall_is_stale = (
            self.world.wall_last_received_at == 0.0
            or now - self.world.wall_last_received_at > self.config.wall_timeout
        )
        if wall_is_stale:
            self.world.wall_distance = self.config.no_wall_distance
            self.world.wall_left_distance = self.config.no_wall_distance
            self.world.wall_right_distance = self.config.no_wall_distance

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
            return self.choose_green_action()

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

    def choose_green_action(self) -> Action:
        """초록색이 보일 때 거리와 센서 패턴에 맞는 행동을 선택한다."""
        event = self.world.sensor_event
        distance = self.world.target_distance
        green_close = 0.0 < distance <= 0.05
        green_far = distance >= 1.2
        front_right_colored = self.world.sensor1 != Sensor_Color.BLACK
        front_left_colored = self.world.sensor2 != Sensor_Color.BLACK
        front_both_colored = front_right_colored and front_left_colored

        # 뒤 파랑이 유지되는 중 앞 양쪽에 색이 함께 나타나면 먼저 안전하게 후진한다.
        # ALL_BLUE는 아래에서 일반적인 전체 파랑 초록색 행동으로 처리한다.
        if (
            green_close
            and self.world.sensor3 == Sensor_Color.BLUE
            and front_both_colored
            and event != Sensor_Event.ALL_BLUE
        ):
            return Green_Close_Back_Blue_Action(self.config)

        # 세 센서 전체 파랑은 가까우면 최대 속도, 아니면 초록색 추격 속도다.
        if event == Sensor_Event.ALL_BLUE:
            return Green_Sensor_Follow_Action(self.config)

        # 세 센서 전체 빨강은 가까울 때 현재 빨강 상태가 풀릴 때까지 후진한다.
        if event == Sensor_Event.ALL_RED:
            if green_close:
                return Green_Close_All_Red_Action(self.config)
            return Green_Follow_Action(self.config)

        # 뒤 센서만 빨강이면 가까울 때 최대 속도, 멀 때 추격 속도로 간다.
        if event == Sensor_Event.BACK_RED:
            return Green_Sensor_Follow_Action(self.config)

        # 뒤 센서만 파랑이면 가까울 때 최대 속도, 멀 때 추격 속도로 간다.
        if event == Sensor_Event.BACK_BLUE:
            return Green_Sensor_Follow_Action(self.config)

        # 앞 양쪽 빨강은 거리별로 기존 회피 행동 또는 45도 회전을 선택한다.
        if event == Sensor_Event.FRONT_BOTH_RED:
            if green_close:
                return Green_Close_Front_Color_Action(
                    self.config,
                    Sensor_Color.RED,
                )
            if green_far:
                if not self.far_green_turn_done:
                    return Far_Green_Turn_Action(self.config)
                return Green_Follow_Action(self.config)
            if distance > 0.05:
                return Front_Color_Avoid_Action(
                    self.config,
                    Sensor_Color.RED,
                    None,
                    interruptible_by_green=False,
                )
            return Green_Follow_Action(self.config)

        # 앞 양쪽 파랑은 가까우면 후진 후 초록색 중심 추격, 멀면 기존 회피다.
        if event == Sensor_Event.FRONT_BOTH_BLUE:
            if green_close:
                return Green_Close_Front_Color_Action(
                    self.config,
                    Sensor_Color.BLUE,
                )
            return Front_Color_Avoid_Action(
                self.config,
                Sensor_Color.BLUE,
                None,
                interruptible_by_green=False,
            )

        # 앞 한쪽 빨강만 보이면 1.2m 이상에서만 기존 45도 회전을 사용한다.
        if event in (Sensor_Event.FRONT_RIGHT_RED, Sensor_Event.FRONT_LEFT_RED):
            if green_far and not self.far_green_turn_done:
                return Far_Green_Turn_Action(self.config)
            return Green_Follow_Action(self.config)

        # 앞 한쪽 파랑 또는 000은 별도 회피 없이 기존 초록색 추격을 유지한다.
        return Green_Follow_Action(self.config)

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
        # 속도 값이 조금 달라져도 매 주기 출력하지 않는다. 대신 행동, 명령,
        # 초록색 발견 여부, 센서 이벤트 중 하나라도 바뀌면 새 상태를 출력한다.
        command_state = (
            self.active_action.name,
            command.label,
            self.world.target_found,
            self.world.sensor_event,
        )
        if command_state == self.last_command_state:
            return
        self.last_command_state = command_state
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

        # 초록색이 보이는 중 앞 양쪽이 같은 색이면, 일반 초록색 추격보다
        # 초록색용 색깔표를 먼저 다시 선택한다. 벽 회피와 시작 45도 회전은
        # 이 규칙보다 우선하므로 진행 중인 행동을 그대로 유지한다.
        front_both_color = self.world.sensor_event in (
            Sensor_Event.FRONT_BOTH_RED,
            Sensor_Event.FRONT_BOTH_BLUE,
        )
        opening_turning = (
            isinstance(self.active_action, Opening_Action)
            and not self.active_action.can_interrupt_for_green()
        )
        if (
            self.world.target_found
            and wall_direction is None
            and front_both_color
            and not isinstance(self.active_action, Wall_Avoid_Action)
            and not opening_turning
        ):
            self.transition_to(self.choose_green_action(), now, cancelled=True)

        if (
            self.active_action.locked
            and self.active_action.can_interrupt_for_green()
            and self.world.target_found
            and wall_direction is None
            # 이미 일반 초록색 추격 중인 행동은 아래의 unlocked 재선택으로 처리한다.
            and not isinstance(self.active_action, Green_Follow_Action)
        ):
            # 초록색을 발견해도 일반 추격으로 고정하지 않고,
            # 센서·거리별 초록색 행동표를 즉시 다시 선택한다.
            self.transition_to(self.choose_action(), now, cancelled=True)

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
