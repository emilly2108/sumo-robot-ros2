# -*- coding: utf-8 -*-
"""HFSM version of test_code.py.

The original test_code.py is intentionally left unchanged.

ROS inputs:
  /target_error, /target_distance, /target_found
  /wall_distance, /wall_left_distance, /wall_right_distance
  /red_between_green
  /test_sensor1, /test_sensor2, /test_sensor3

Motor outputs:
  /cmd_vel
  /motor_node/hard_stop (std_srvs/Trigger)
  /motor_node/release_hard_stop (std_srvs/Trigger)

Sensor positions:
  sensor1 = right front
  sensor2 = left front
  sensor3 = rear
"""

import math
from dataclasses import dataclass
from enum import Enum, IntEnum, auto
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32
from std_srvs.srv import Trigger


class SensorColor(IntEnum):
    BLACK = 0
    RED = 1
    BLUE = 2


class SensorEvent(Enum):
    UNKNOWN = auto()
    NONE = auto()
    FRONT_RIGHT_RED = auto()
    FRONT_LEFT_RED = auto()
    FRONT_BOTH_RED = auto()
    FRONT_RIGHT_BLUE = auto()
    FRONT_LEFT_BLUE = auto()
    FRONT_BOTH_BLUE = auto()
    REAR_RED = auto()
    REAR_BLUE = auto()
    ALL_RED = auto()
    ALL_BLUE = auto()


SENSOR_EVENTS = {
    (SensorColor.BLACK, SensorColor.BLACK, SensorColor.BLACK): SensorEvent.NONE,
    (SensorColor.RED, SensorColor.BLACK, SensorColor.BLACK): SensorEvent.FRONT_RIGHT_RED,
    (SensorColor.BLACK, SensorColor.RED, SensorColor.BLACK): SensorEvent.FRONT_LEFT_RED,
    (SensorColor.RED, SensorColor.RED, SensorColor.BLACK): SensorEvent.FRONT_BOTH_RED,
    (SensorColor.BLUE, SensorColor.BLACK, SensorColor.BLACK): SensorEvent.FRONT_RIGHT_BLUE,
    (SensorColor.BLACK, SensorColor.BLUE, SensorColor.BLACK): SensorEvent.FRONT_LEFT_BLUE,
    (SensorColor.BLUE, SensorColor.BLUE, SensorColor.BLACK): SensorEvent.FRONT_BOTH_BLUE,
    (SensorColor.BLACK, SensorColor.BLACK, SensorColor.RED): SensorEvent.REAR_RED,
    (SensorColor.BLACK, SensorColor.BLACK, SensorColor.BLUE): SensorEvent.REAR_BLUE,
    (SensorColor.RED, SensorColor.RED, SensorColor.RED): SensorEvent.ALL_RED,
    (SensorColor.BLUE, SensorColor.BLUE, SensorColor.BLUE): SensorEvent.ALL_BLUE,
}


@dataclass
class WorldModel:
    target_error: float = 0.0
    target_distance: float = 9.9
    target_found: bool = False
    target_updated_at: float = 0.0

    wall_distance: float = 9.9
    wall_left_distance: float = 9.9
    wall_right_distance: float = 9.9
    wall_updated_at: float = 0.0

    red_between_green: bool = False

    sensor1: SensorColor = SensorColor.BLACK
    sensor2: SensorColor = SensorColor.BLACK
    sensor3: SensorColor = SensorColor.BLACK
    sensor_event: SensorEvent = SensorEvent.NONE
    sensor_updated_at: float = 0.0


@dataclass(frozen=True)
class BrainConfig:
    base_speed: float = 0.6
    chase_speed: float = 0.7
    max_speed: float = 0.8
    reverse_speed: float = -0.3
    short_reverse_speed: float = -0.25
    short_forward_speed: float = 0.25

    max_angular_speed: float = 2.0
    turn_speed: float = 0.8
    kp: float = 1.8
    deadzone: float = 0.03

    max_speed_distance: float = 0.30
    front_green_distance: float = 0.40
    opponent_close_distance: float = 0.50
    far_green_distance: float = 1.20
    wall_close_distance: float = 0.20
    no_wall_distance: float = 9.9

    control_period: float = 0.02
    vision_timeout: float = 0.50
    wall_timeout: float = 0.50

    short_distance: float = 0.10
    turn_45_duration: float = (math.pi / 4.0) / 0.8
    turn_90_duration: float = (math.pi / 2.0) / 0.8
    all_color_push_duration: float = 5.0
    red_route_repeats: int = 3
    green_angle_error_limit: float = 0.85


@dataclass(frozen=True)
class MotionCommand:
    linear: float = 0.0
    angular: float = 0.0
    hard_stop: bool = False
    label: str = "STOP"


@dataclass(frozen=True)
class ActionStep:
    command: MotionCommand
    finished: bool = False


def now_seconds(node: Node) -> float:
    return node.get_clock().now().nanoseconds / 1e9


def green_follow_command(world: WorldModel, config: BrainConfig) -> MotionCommand:
    speed = config.chase_speed
    if 0.0 < world.target_distance <= config.max_speed_distance:
        speed = config.max_speed

    angular = -config.kp * world.target_error
    if abs(world.target_error) < config.deadzone:
        angular = 0.0
    angular = max(min(angular, config.max_angular_speed), -config.max_angular_speed)

    label = "GREEN_MAX" if speed == config.max_speed else "GREEN_CHASE"
    return MotionCommand(speed, angular, False, label)


def wall_avoid_direction(world: WorldModel, config: BrainConfig) -> Optional[float]:
    if (
        world.target_found
        and 0.0 < world.target_distance <= config.opponent_close_distance
    ):
        return None

    left_close = 0.0 < world.wall_left_distance <= config.wall_close_distance
    right_close = 0.0 < world.wall_right_distance <= config.wall_close_distance
    if not (left_close or right_close):
        return None
    if world.wall_left_distance > world.wall_right_distance:
        return 1.0
    if world.wall_right_distance > world.wall_left_distance:
        return -1.0
    return 1.0


class Action:
    name = "ACTION"
    locked = True
    interruptible_by_green = False

    def enter(self, now: float, world: WorldModel) -> None:
        del now, world

    def exit(self, cancelled: bool) -> None:
        del cancelled

    def can_interrupt_for_green(self) -> bool:
        return self.interruptible_by_green

    def key(self):
        return (type(self),)

    def step(self, now: float, world: WorldModel) -> ActionStep:
        raise NotImplementedError


class OpeningPhase(Enum):
    TURN_LEFT_45 = auto()
    GO_STRAIGHT = auto()


class OpeningAction(Action):
    name = "OPENING"

    def __init__(self, config: BrainConfig):
        self.config = config
        self.phase = OpeningPhase.TURN_LEFT_45
        self.started_at = 0.0

    def enter(self, now: float, world: WorldModel) -> None:
        del world
        self.phase = OpeningPhase.TURN_LEFT_45
        self.started_at = now

    def can_interrupt_for_green(self) -> bool:
        return self.phase == OpeningPhase.GO_STRAIGHT

    def step(self, now: float, world: WorldModel) -> ActionStep:
        if self.phase == OpeningPhase.TURN_LEFT_45:
            if now - self.started_at < self.config.turn_45_duration:
                return ActionStep(
                    MotionCommand(0.0, self.config.turn_speed, False, "OPENING_TURN_LEFT_45")
                )
            self.phase = OpeningPhase.GO_STRAIGHT
            self.started_at = now

        if world.target_found or wall_avoid_direction(world, self.config) is not None:
            return ActionStep(MotionCommand(label="OPENING_DONE"), True)

        return ActionStep(
            MotionCommand(self.config.base_speed, 0.0, False, "OPENING_STRAIGHT")
        )


class CruiseAction(Action):
    name = "CRUISE"
    locked = False
    interruptible_by_green = True

    def __init__(self, config: BrainConfig):
        self.config = config

    def step(self, now: float, world: WorldModel) -> ActionStep:
        del now, world
        return ActionStep(MotionCommand(self.config.base_speed, 0.0, False, "CRUISE"))


class GreenFollowAction(Action):
    name = "GREEN_FOLLOW"
    locked = False

    def __init__(self, config: BrainConfig):
        self.config = config

    def step(self, now: float, world: WorldModel) -> ActionStep:
        del now
        if not world.target_found:
            return ActionStep(MotionCommand(label="GREEN_LOST"), True)
        return ActionStep(green_follow_command(world, self.config))


class FarGreenTurnAction(Action):
    name = "FAR_GREEN_TURN_45"

    def __init__(self, config: BrainConfig, direction: float = 1.0):
        self.config = config
        self.direction = direction
        self.started_at = 0.0

    def key(self):
        return (type(self), self.direction)

    def enter(self, now: float, world: WorldModel) -> None:
        del world
        self.started_at = now

    def step(self, now: float, world: WorldModel) -> ActionStep:
        del world
        if now - self.started_at >= self.config.turn_45_duration:
            return ActionStep(MotionCommand(label="FAR_GREEN_TURN_DONE"), True)
        return ActionStep(
            MotionCommand(
                0.0,
                self.direction * self.config.turn_speed,
                False,
                "FAR_GREEN_TURN_45",
            )
        )


class WallAvoidAction(Action):
    name = "WALL_AVOID"

    def __init__(self, config: BrainConfig, direction: float):
        self.config = config
        self.direction = direction
        self.started_at = 0.0

    def key(self):
        return (type(self), self.direction)

    def enter(self, now: float, world: WorldModel) -> None:
        del world
        self.started_at = now

    def step(self, now: float, world: WorldModel) -> ActionStep:
        del world
        if now - self.started_at >= self.config.turn_90_duration:
            return ActionStep(MotionCommand(label="WALL_AVOID_DONE"), True)
        side = "LEFT" if self.direction > 0.0 else "RIGHT"
        return ActionStep(
            MotionCommand(
                0.0,
                self.direction * self.config.turn_speed,
                False,
                f"WALL_AVOID_{side}_90",
            )
        )


class FrontEscapePhase(Enum):
    BACK_10CM = auto()
    TURN_45 = auto()


class FrontBlueEscapeAction(Action):
    name = "FRONT_BLUE_ESCAPE"
    interruptible_by_green = True

    def __init__(self, config: BrainConfig, direction: float):
        self.config = config
        self.direction = direction
        self.phase = FrontEscapePhase.BACK_10CM
        self.started_at = 0.0
        self.back_duration = config.short_distance / abs(config.short_reverse_speed)

    def key(self):
        return (type(self), self.direction)

    def enter(self, now: float, world: WorldModel) -> None:
        del world
        self.phase = FrontEscapePhase.BACK_10CM
        self.started_at = now

    def step(self, now: float, world: WorldModel) -> ActionStep:
        del world
        elapsed = now - self.started_at
        if self.phase == FrontEscapePhase.BACK_10CM:
            if elapsed < self.back_duration:
                return ActionStep(
                    MotionCommand(
                        self.config.short_reverse_speed,
                        0.0,
                        False,
                        "FRONT_BLUE_BACK_10CM",
                    )
                )
            self.phase = FrontEscapePhase.TURN_45
            self.started_at = now

        if now - self.started_at < self.config.turn_45_duration:
            side = "LEFT" if self.direction > 0.0 else "RIGHT"
            return ActionStep(
                MotionCommand(
                    0.0,
                    self.direction * self.config.turn_speed,
                    False,
                    f"FRONT_BLUE_TURN_{side}_45",
                )
            )
        return ActionStep(MotionCommand(label="FRONT_BLUE_ESCAPE_DONE"), True)


class RedEscapePhase(Enum):
    BACK_10CM = auto()
    TURN_45 = auto()
    FORWARD_10CM = auto()
    CHECK_GREEN = auto()
    APPROACH_TURN_45 = auto()
    APPROACH_FORWARD_10CM = auto()
    CHARGE = auto()


class FrontRedEscapeAction(Action):
    name = "FRONT_RED_ESCAPE"

    def __init__(self, config: BrainConfig, direction: float, full_route: bool):
        self.config = config
        self.direction = direction
        self.full_route = full_route
        self.phase = RedEscapePhase.BACK_10CM
        self.started_at = 0.0
        self.repeat_count = 0
        self.back_duration = config.short_distance / abs(config.short_reverse_speed)
        self.forward_duration = config.short_distance / config.short_forward_speed

    def key(self):
        return (type(self), self.direction, self.full_route)

    def can_interrupt_for_green(self) -> bool:
        return not self.full_route

    def enter(self, now: float, world: WorldModel) -> None:
        del world
        self.phase = RedEscapePhase.BACK_10CM
        self.started_at = now
        self.repeat_count = 0

    def step(self, now: float, world: WorldModel) -> ActionStep:
        elapsed = now - self.started_at

        if self.phase == RedEscapePhase.BACK_10CM:
            if elapsed < self.back_duration:
                return ActionStep(
                    MotionCommand(
                        self.config.short_reverse_speed,
                        0.0,
                        False,
                        "FRONT_RED_BACK_10CM",
                    )
                )
            self.phase = RedEscapePhase.TURN_45
            self.started_at = now

        if self.phase in (RedEscapePhase.TURN_45, RedEscapePhase.APPROACH_TURN_45):
            if now - self.started_at < self.config.turn_45_duration:
                side = "LEFT" if self.direction > 0.0 else "RIGHT"
                return ActionStep(
                    MotionCommand(
                        0.0,
                        self.direction * self.config.turn_speed,
                        False,
                        f"FRONT_RED_TURN_{side}_45",
                    )
                )

            self.phase = (
                RedEscapePhase.FORWARD_10CM
                if self.phase == RedEscapePhase.TURN_45
                else RedEscapePhase.APPROACH_FORWARD_10CM
            )
            self.started_at = now

        if self.phase in (
            RedEscapePhase.FORWARD_10CM,
            RedEscapePhase.APPROACH_FORWARD_10CM,
        ):
            if now - self.started_at < self.forward_duration:
                return ActionStep(
                    MotionCommand(
                        self.config.short_forward_speed,
                        0.0,
                        False,
                        "FRONT_RED_FORWARD_10CM",
                    )
                )

            if self.phase == RedEscapePhase.FORWARD_10CM:
                if not self.full_route:
                    return ActionStep(MotionCommand(label="SIMPLE_RED_ESCAPE_DONE"), True)
                self.repeat_count += 1
                if self.repeat_count < self.config.red_route_repeats:
                    self.phase = RedEscapePhase.TURN_45
                    self.started_at = now
                    return self.step(now, world)

            self.phase = RedEscapePhase.CHECK_GREEN

        if self.phase == RedEscapePhase.CHECK_GREEN:
            if not world.target_found:
                return ActionStep(MotionCommand(label="RED_ROUTE_GREEN_LOST"), True)
            if 0.0 < world.target_distance <= self.config.front_green_distance:
                self.phase = RedEscapePhase.CHARGE
                return ActionStep(
                    MotionCommand(self.config.max_speed, 0.0, False, "RED_ROUTE_CHARGE")
                )
            if abs(world.target_error) <= self.config.green_angle_error_limit:
                self.phase = RedEscapePhase.APPROACH_TURN_45
                self.started_at = now
                return self.step(now, world)
            return ActionStep(MotionCommand(label="RED_ROUTE_TO_GREEN_FOLLOW"), True)

        if self.phase == RedEscapePhase.CHARGE:
            if not world.target_found:
                return ActionStep(MotionCommand(label="RED_ROUTE_CHARGE_DONE"), True)
            return ActionStep(
                MotionCommand(self.config.max_speed, 0.0, False, "RED_ROUTE_CHARGE")
            )

        return ActionStep(MotionCommand(label="RED_ESCAPE_DONE"), True)


class RearSingleBoostAction(Action):
    name = "REAR_SINGLE_BOOST"

    def __init__(self, config: BrainConfig, expected_event: SensorEvent):
        self.config = config
        self.expected_event = expected_event

    def key(self):
        return (type(self), self.expected_event)

    def step(self, now: float, world: WorldModel) -> ActionStep:
        del now
        if world.sensor_event != self.expected_event:
            return ActionStep(MotionCommand(label="REAR_SINGLE_BOOST_DONE"), True)
        return ActionStep(
            MotionCommand(self.config.max_speed, 0.0, False, "REAR_SINGLE_BOOST")
        )


class AllRedPhase(Enum):
    PUSH = auto()
    BACK_UNTIL_FRONT_RED = auto()
    HARD_STOP_HOLD = auto()


class AllRedRecoveryAction(Action):
    name = "ALL_RED_RECOVERY"

    def __init__(self, config: BrainConfig):
        self.config = config
        self.phase = AllRedPhase.PUSH
        self.started_at = 0.0

    def enter(self, now: float, world: WorldModel) -> None:
        del world
        self.phase = AllRedPhase.PUSH
        self.started_at = now

    def step(self, now: float, world: WorldModel) -> ActionStep:
        if self.phase == AllRedPhase.PUSH:
            if world.sensor_event != SensorEvent.ALL_RED:
                return ActionStep(MotionCommand(label="ALL_RED_CANCELLED"), True)
            if now - self.started_at < self.config.all_color_push_duration:
                return ActionStep(
                    MotionCommand(self.config.max_speed, 0.0, False, "ALL_RED_MAX_PUSH")
                )
            self.phase = AllRedPhase.BACK_UNTIL_FRONT_RED

        if self.phase == AllRedPhase.BACK_UNTIL_FRONT_RED:
            if world.sensor_event == SensorEvent.FRONT_BOTH_RED:
                self.phase = AllRedPhase.HARD_STOP_HOLD
            else:
                return ActionStep(
                    MotionCommand(self.config.reverse_speed, 0.0, False, "ALL_RED_REVERSE")
                )

        if self.phase == AllRedPhase.HARD_STOP_HOLD:
            if world.sensor_event != SensorEvent.FRONT_BOTH_RED:
                return ActionStep(MotionCommand(label="ALL_RED_HOLD_DONE"), True)
            return ActionStep(MotionCommand(hard_stop=True, label="ALL_RED_HARD_STOP"))

        return ActionStep(MotionCommand(label="ALL_RED_DONE"), True)


class AllBluePhase(Enum):
    PUSH = auto()
    BACK_UNTIL_BLACK = auto()


class AllBlueRecoveryAction(Action):
    name = "ALL_BLUE_RECOVERY"

    def __init__(self, config: BrainConfig):
        self.config = config
        self.phase = AllBluePhase.PUSH
        self.started_at = 0.0

    def enter(self, now: float, world: WorldModel) -> None:
        del world
        self.phase = AllBluePhase.PUSH
        self.started_at = now

    def step(self, now: float, world: WorldModel) -> ActionStep:
        if self.phase == AllBluePhase.PUSH:
            if world.sensor_event != SensorEvent.ALL_BLUE:
                return ActionStep(MotionCommand(label="ALL_BLUE_CANCELLED"), True)
            if now - self.started_at < self.config.all_color_push_duration:
                return ActionStep(
                    MotionCommand(self.config.max_speed, 0.0, False, "ALL_BLUE_MAX_PUSH")
                )
            self.phase = AllBluePhase.BACK_UNTIL_BLACK

        if world.sensor_event == SensorEvent.NONE:
            return ActionStep(MotionCommand(label="ALL_BLUE_REVERSE_DONE"), True)
        return ActionStep(
            MotionCommand(self.config.reverse_speed, 0.0, False, "ALL_BLUE_REVERSE")
        )


class FrontBothBlueHoldAction(Action):
    name = "FRONT_BOTH_BLUE_HOLD"

    def __init__(self, config: BrainConfig):
        self.config = config

    def step(self, now: float, world: WorldModel) -> ActionStep:
        del now
        if world.sensor_event != SensorEvent.FRONT_BOTH_BLUE:
            return ActionStep(MotionCommand(label="FRONT_BOTH_BLUE_RELEASE"), True)
        if world.target_found and world.target_distance >= self.config.front_green_distance:
            return ActionStep(MotionCommand(label="FRONT_BOTH_BLUE_TO_GREEN"), True)
        return ActionStep(MotionCommand(hard_stop=True, label="FRONT_BOTH_BLUE_HARD_STOP"))


class HfsmBrainNode(Node):
    def __init__(self):
        super().__init__("brain_hfsm_node")
        self.config = BrainConfig()
        self.world = WorldModel()
        self.opening_done = False
        self.far_green_turn_done = False
        self.green_preempted_sensor_event: Optional[SensorEvent] = None

        self.active_action: Action = OpeningAction(self.config)
        self.active_action.enter(now_seconds(self), self.world)

        self.hard_stop_client = self.create_client(Trigger, "/motor_node/hard_stop")
        self.release_hard_stop_client = self.create_client(
            Trigger, "/motor_node/release_hard_stop"
        )
        self.hard_stop_expected = False
        self.hard_stop_future = None
        self.release_hard_stop_future = None
        self.last_release_attempt = -1.0
        self.last_service_warning = -1.0

        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 10)

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
        self.create_subscription(
            Bool, "/red_between_green", self.red_between_green_callback, 10
        )

        # Temporary Int32 topics can be replaced by the real color-sensor topics later.
        self.create_subscription(Int32, "/test_sensor1", self.sensor1_callback, 10)
        self.create_subscription(Int32, "/test_sensor2", self.sensor2_callback, 10)
        self.create_subscription(Int32, "/test_sensor3", self.sensor3_callback, 10)

        self.last_command_label = None
        self.timer = self.create_timer(self.config.control_period, self.control_loop)
        self.get_logger().info("HFSM brain started: action=OPENING")

    def target_error_callback(self, msg: Float32) -> None:
        self.world.target_error = float(msg.data)
        self.world.target_updated_at = now_seconds(self)

    def target_distance_callback(self, msg: Float32) -> None:
        self.world.target_distance = float(msg.data)
        self.world.target_updated_at = now_seconds(self)

    def target_found_callback(self, msg: Bool) -> None:
        self.world.target_found = bool(msg.data)
        self.world.target_updated_at = now_seconds(self)
        if not msg.data:
            self.world.target_error = 0.0
            self.world.target_distance = 9.9
            self.far_green_turn_done = False
            self.green_preempted_sensor_event = None

    def wall_distance_callback(self, msg: Float32) -> None:
        self.world.wall_distance = float(msg.data)
        self.world.wall_updated_at = now_seconds(self)

    def wall_left_callback(self, msg: Float32) -> None:
        self.world.wall_left_distance = float(msg.data)
        self.world.wall_updated_at = now_seconds(self)

    def wall_right_callback(self, msg: Float32) -> None:
        self.world.wall_right_distance = float(msg.data)
        self.world.wall_updated_at = now_seconds(self)

    def red_between_green_callback(self, msg: Bool) -> None:
        self.world.red_between_green = bool(msg.data)

    def _set_sensor(self, attr_name: str, value: int) -> None:
        try:
            color = SensorColor(value)
        except ValueError:
            self.get_logger().warning(
                f"Ignored invalid sensor value {value}; use BLACK=0, RED=1, BLUE=2"
            )
            return
        setattr(self.world, attr_name, color)
        self.world.sensor_event = SENSOR_EVENTS.get(
            (self.world.sensor1, self.world.sensor2, self.world.sensor3),
            SensorEvent.UNKNOWN,
        )
        if (
            self.green_preempted_sensor_event is not None
            and self.world.sensor_event != self.green_preempted_sensor_event
        ):
            self.green_preempted_sensor_event = None
        self.world.sensor_updated_at = now_seconds(self)

    def sensor1_callback(self, msg: Int32) -> None:
        self._set_sensor("sensor1", int(msg.data))

    def sensor2_callback(self, msg: Int32) -> None:
        self._set_sensor("sensor2", int(msg.data))

    def sensor3_callback(self, msg: Int32) -> None:
        self._set_sensor("sensor3", int(msg.data))

    def refresh_timeouts(self, now: float) -> None:
        if now - self.world.target_updated_at > self.config.vision_timeout:
            self.world.target_found = False
            self.world.target_error = 0.0
            self.world.target_distance = 9.9
            self.far_green_turn_done = False
            self.green_preempted_sensor_event = None

        if now - self.world.wall_updated_at > self.config.wall_timeout:
            self.world.wall_distance = self.config.no_wall_distance
            self.world.wall_left_distance = self.config.no_wall_distance
            self.world.wall_right_distance = self.config.no_wall_distance

    def transition_to(self, next_action: Action, now: float, cancelled: bool = False) -> None:
        if self.active_action.key() == next_action.key():
            return
        old_name = self.active_action.name
        self.active_action.exit(cancelled)
        self.active_action = next_action
        self.active_action.enter(now, self.world)
        self.get_logger().info(f"HFSM: {old_name} -> {self.active_action.name}")

    def choose_action(self) -> Action:
        event = self.world.sensor_event
        sensor_was_preempted_for_green = (
            self.world.target_found
            and self.green_preempted_sensor_event is not None
            and event == self.green_preempted_sensor_event
        )

        if not sensor_was_preempted_for_green:
            if event == SensorEvent.FRONT_RIGHT_RED:
                return FrontRedEscapeAction(
                    self.config, 1.0, self.world.red_between_green
                )
            if event == SensorEvent.FRONT_LEFT_RED:
                return FrontRedEscapeAction(
                    self.config, -1.0, self.world.red_between_green
                )
            if event == SensorEvent.FRONT_BOTH_RED:
                return CruiseAction(self.config)
            if event == SensorEvent.FRONT_RIGHT_BLUE:
                return FrontBlueEscapeAction(self.config, 1.0)
            if event == SensorEvent.FRONT_LEFT_BLUE:
                return FrontBlueEscapeAction(self.config, -1.0)
            if event == SensorEvent.FRONT_BOTH_BLUE:
                return FrontBothBlueHoldAction(self.config)
            if event in (SensorEvent.REAR_RED, SensorEvent.REAR_BLUE):
                return RearSingleBoostAction(self.config, event)
            if event == SensorEvent.ALL_RED:
                return AllRedRecoveryAction(self.config)
            if event == SensorEvent.ALL_BLUE:
                return AllBlueRecoveryAction(self.config)

        direction = wall_avoid_direction(self.world, self.config)
        if direction is not None:
            return WallAvoidAction(self.config, direction)

        if self.world.target_found:
            if (
                self.world.target_distance > self.config.far_green_distance
                and not self.far_green_turn_done
                and event == SensorEvent.NONE
            ):
                return FarGreenTurnAction(self.config, 1.0)
            return GreenFollowAction(self.config)

        return CruiseAction(self.config)

    def _service_warning(self, now: float, message: str) -> None:
        if now - self.last_service_warning >= 1.0:
            self.last_service_warning = now
            self.get_logger().error(message)

    def request_hard_stop(self, now: float) -> None:
        if self.hard_stop_expected or self.hard_stop_future is not None:
            return
        if not self.hard_stop_client.service_is_ready():
            self._service_warning(
                now,
                "Hard stop service is unavailable: /motor_node/hard_stop",
            )
            return

        self.hard_stop_expected = True
        self.hard_stop_future = self.hard_stop_client.call_async(Trigger.Request())
        self.hard_stop_future.add_done_callback(self._hard_stop_response)

    def _hard_stop_response(self, future) -> None:
        self.hard_stop_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.hard_stop_expected = False
            self.get_logger().error(f"Hard stop service failed: {exc}")
            return
        if not response.success:
            self.hard_stop_expected = False
            self.get_logger().error(f"Hard stop rejected: {response.message}")
            return
        self.get_logger().warning(f"Hard stop requested: {response.message}")

    def ensure_hard_stop_released(self, now: float) -> bool:
        if not self.hard_stop_expected:
            return True
        if self.hard_stop_future is not None or self.release_hard_stop_future is not None:
            return False
        if now - self.last_release_attempt < 0.10:
            return False
        if not self.release_hard_stop_client.service_is_ready():
            self._service_warning(
                now,
                "Hard stop release service is unavailable: /motor_node/release_hard_stop",
            )
            return False

        self.last_release_attempt = now
        self.release_hard_stop_future = self.release_hard_stop_client.call_async(
            Trigger.Request()
        )
        self.release_hard_stop_future.add_done_callback(self._release_hard_stop_response)
        return False

    def _release_hard_stop_response(self, future) -> None:
        self.release_hard_stop_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Hard stop release service failed: {exc}")
            return
        if response.success:
            self.hard_stop_expected = False
            self.get_logger().info(f"Hard stop released: {response.message}")

    def publish_command(self, command: MotionCommand, now: float) -> None:
        if command.hard_stop:
            self.request_hard_stop(now)
            self._log_command(command)
            return

        if not self.ensure_hard_stop_released(now):
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

    def _log_command(self, command: MotionCommand) -> None:
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

        if (
            self.active_action.locked
            and self.active_action.can_interrupt_for_green()
            and self.world.target_found
            and not isinstance(self.active_action, GreenFollowAction)
        ):
            self.green_preempted_sensor_event = self.world.sensor_event
            self.transition_to(GreenFollowAction(self.config), now, cancelled=True)

        if not self.active_action.locked:
            self.transition_to(self.choose_action(), now)

        # A completed state can move to the next state in the same 20 ms tick.
        for _ in range(3):
            step = self.active_action.step(now, self.world)
            if not step.finished:
                self.publish_command(step.command, now)
                return

            if isinstance(self.active_action, OpeningAction):
                self.opening_done = True
            if isinstance(self.active_action, FarGreenTurnAction):
                self.far_green_turn_done = True

            self.active_action.exit(cancelled=False)
            self.active_action = self.choose_action()
            self.active_action.enter(now, self.world)

        self.get_logger().error("HFSM exceeded the transition limit in one control tick")


def main(args=None):
    rclpy.init(args=args)
    node = HfsmBrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
