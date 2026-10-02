from enum import Enum, auto

from ..helpers import green_sensor_follow_command
from ..models import (
    Action_Step,
    Brain_Config,
    Motion_Command,
    Sensor_Color,
    Sensor_Event,
    World_Model,
)
from .base import Action


class Green_Close_Front_Phase(Enum):
    # 앞 센서의 색이 모두 사라질 때까지 후진하는 내부 단계 이름이다.
    BACK_UNTIL_CLEAR = auto()


class Green_Close_Front_Color_Action(Action):
    """Reverse until the front color clears, then return to green following."""

    name = "GREEN_CLOSE_FRONT_COLOR"
    interruptible_by_green = False

    def __init__(self, config: Brain_Config, color: Sensor_Color):
        # 공통 속도 설정과 로그에 표시할 대상 색을 보관한다.
        self.config = config
        self.color = color

    def key(self):
        # 빨강 전용과 파랑 전용 행동은 서로 다른 상태 전환으로 구분한다.
        return (type(self), self.color)

    def step(self, now: float, world: World_Model) -> Action_Step:
        # 이 동작은 시간보다 목표·앞 센서의 최신 상태로 진행된다.
        del now
        if not world.target_found:
            return Action_Step(
                Motion_Command(label=f"GREEN_CLOSE_{self.color.name}_TARGET_LOST"),
                finished=True,
            )

        # 앞에 빨강·파랑 중 무엇이 남아 있어도 두 센서가 검정이 될 때까지 후진한다.
        front_color_present = (
            world.sensor1 != Sensor_Color.BLACK
            or world.sensor2 != Sensor_Color.BLACK
        )
        if front_color_present:
            return Action_Step(
                Motion_Command(
                    self.config.reverse_speed,
                    0.0,
                    False,
                    f"GREEN_CLOSE_{self.color.name}_BACK_UNTIL_CLEAR",
                )
            )

        # 앞 색이 사라지면 거리와 관계없이 전용 행동을 끝내고 일반 초록 추격으로 돌아간다.
        return Action_Step(
            Motion_Command(label=f"GREEN_CLOSE_{self.color.name}_RELEASE"),
            finished=True,
        )

class Green_Close_Front_Red_Action(Action):
    """Reverse until front red clears, then return to green following."""

    name = "GREEN_CLOSE_FRONT_RED"
    interruptible_by_green = False

    def __init__(self, config: Brain_Config):
        # 전용 빨강 행동에서 쓸 공통 속도 설정을 저장한다.
        self.config = config

    def step(self, now: float, world: World_Model) -> Action_Step:
        # 시간 경과가 아닌 목표와 센서 상태로만 진행한다.
        del now
        if not world.target_found:
            return Action_Step(
                Motion_Command(label="GREEN_CLOSE_FRONT_RED_TARGET_LOST"),
                finished=True,
            )

        # 앞 센서에 빨강이 하나라도 남아 있으면 빨강이 모두 사라질 때까지 후진한다.
        front_red_present = (
            world.sensor1 == Sensor_Color.RED
            or world.sensor2 == Sensor_Color.RED
        )
        if front_red_present:
            return Action_Step(
                Motion_Command(
                    self.config.reverse_speed,
                    0.0,
                    False,
                    "GREEN_CLOSE_FRONT_RED_BACK_UNTIL_CLEAR",
                )
            )

        return Action_Step(
            Motion_Command(label="GREEN_CLOSE_FRONT_RED_RELEASE"),
            finished=True,
        )

class Green_Close_Back_Blue_Phase(Enum):
    # 앞이 비어 있을 때 일반 초록 추격을 하는 단계다.
    FOLLOW_GREEN = auto()
    # 뒤 파랑과 앞 양쪽 색이 겹칠 때 앞을 비우는 후진 단계다.
    BACK_UNTIL_FRONT_CLEAR = auto()


class Green_Close_Back_Blue_Action(Action):
    """Handle a close green target while the rear blue sensor and both front sensors are active."""

    name = "GREEN_CLOSE_BACK_BLUE"
    interruptible_by_green = False

    def __init__(self, config: Brain_Config):
        # 공통 설정을 저장하고 일반 추격 단계에서 시작한다.
        self.config = config
        self.phase = Green_Close_Back_Blue_Phase.FOLLOW_GREEN

    def step(self, now: float, world: World_Model) -> Action_Step:
        # 이 행동도 시간보다 최신 센서·거리 상태만 사용한다.
        del now

        # 목표 또는 뒤 파랑 조건이 사라지면 전용 행동을 종료한다.
        if not world.target_found or world.sensor3 != Sensor_Color.BLUE:
            return Action_Step(Motion_Command(label="GREEN_CLOSE_BACK_BLUE_DONE"), True)

        # 초근접 5 cm 조건이 깨져도 일반 초록 판단으로 돌아간다.
        if not (0.0 < world.target_distance <= 0.05):
            return Action_Step(Motion_Command(label="GREEN_CLOSE_BACK_BLUE_DISTANCE_DONE"), True)

        # 앞 양쪽에 모두 색이 있는 경우에만 안전 후진 단계로 바꾼다.
        front_has_color = (
            world.sensor1 != Sensor_Color.BLACK
            and world.sensor2 != Sensor_Color.BLACK
        )
        if front_has_color:
            self.phase = Green_Close_Back_Blue_Phase.BACK_UNTIL_FRONT_CLEAR

        # 후진 단계에서는 앞 양쪽이 검정이 될 때까지 목표 추격을 보류한다.
        if self.phase == Green_Close_Back_Blue_Phase.BACK_UNTIL_FRONT_CLEAR:
            front_is_clear = (
                world.sensor1 == Sensor_Color.BLACK
                and world.sensor2 == Sensor_Color.BLACK
            )
            if not front_is_clear:
                return Action_Step(
                    Motion_Command(
                        self.config.reverse_speed,
                        0.0,
                        False,
                        "GREEN_CLOSE_BACK_BLUE_BACK_UNTIL_FRONT_CLEAR",
                    )
                )
            # 앞이 비워지면 다시 초록 추격 단계로 되돌린다.
            self.phase = Green_Close_Back_Blue_Phase.FOLLOW_GREEN

        # 남은 경우에는 센서 전용 초록 추격 명령을 계산해 반환한다.
        return Action_Step(green_sensor_follow_command(world, self.config))
