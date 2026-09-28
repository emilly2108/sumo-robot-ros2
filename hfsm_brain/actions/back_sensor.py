from enum import Enum, auto
from ..models import Action_Step, Brain_Config, Motion_Command, Sensor_Event, World_Model
from .base import Action

class Back_Single_Boost_Action(Action):
    name = "BACK_SINGLE_BOOST"

    def __init__(self, config: Brain_Config, expected_event: Sensor_Event):
        self.config = config
        self.expected_event = expected_event

    def key(self):
        return (type(self), self.expected_event)

    def step(self, now: float, world: World_Model) -> Action_Step:
        del now
        if world.sensor_event != self.expected_event:
            return Action_Step(Motion_Command(label="BACK_SINGLE_BOOST_DONE"), True)
        return Action_Step(
            Motion_Command(self.config.max_speed, 0.0, False, "BACK_SINGLE_BOOST")
        )


class All_Red_Phase(Enum):
    PUSH = auto()
    BACK_UNTIL_BLACK = auto()

class All_Red_Recovery_Action(Action):
    name = "ALL_RED_RECOVERY"

    def __init__(self, config: Brain_Config):
        # 속도·시간 공통 설정과 현재 내부 단계를 준비한다.
        self.config = config
        self.phase = All_Red_Phase.PUSH
        self.started_at = 0.0

    def enter(self, now: float, world: World_Model) -> None:
        # 상태 전환 때마다 5초 측정을 처음부터 다시 시작한다.
        del world
        self.phase = All_Red_Phase.PUSH
        self.started_at = now

    def step(self, now: float, world: World_Model) -> Action_Step:
        if self.phase == All_Red_Phase.PUSH:
            if world.sensor_event != Sensor_Event.ALL_RED:
                # 더 이상 111이 아니면 최대 속도 밀기를 취소하고 행동을 종료한다.
                return Action_Step(Motion_Command(label="ALL_RED_CANCELLED"), True)
            # 111이 유지된 시간이 설정된 5초보다 짧은지 확인한다.
            if now - self.started_at < self.config.all_color_push_duration:
                # 회전하지 않고 최대 속도로 계속 전진한다.
                return Action_Step(
                    Motion_Command(self.config.max_speed, 0.0, False, "ALL_RED_MAX_PUSH")
                )
            # 111이 5초 이상 계속됐으므로 000이 될 때까지 후진하는 단계로 바꾼다.
            self.phase = All_Red_Phase.BACK_UNTIL_BLACK

        # After the five-second push, reverse until all three color sensors
        # have returned to black (000).
        if self.phase == All_Red_Phase.BACK_UNTIL_BLACK:
            if world.sensor_event == Sensor_Event.NONE:
                return Action_Step(Motion_Command(label="ALL_RED_REVERSE_DONE"), True)
            return Action_Step(
                Motion_Command(self.config.reverse_speed, 0.0, False, "ALL_RED_REVERSE")
            )

        return Action_Step(Motion_Command(label="ALL_RED_DONE"), True)

############################################
class All_Blue_Recovery_Action(Action):
    # HFSM 로그에 표시할 행동 이름이다.
    name = "ALL_BLUE_RECOVERY"

    def __init__(self, config: Brain_Config):
        # 전체 파랑 동안 사용할 최대 속도 설정을 저장한다.
        self.config = config

    def step(self, now: float, world: World_Model) -> Action_Step:
        # 전체 파랑 상태 유지만 판단하므로 시간은 쓰지 않는다.
        del now
        if world.sensor_event != Sensor_Event.ALL_BLUE:
            return Action_Step(Motion_Command(label="ALL_BLUE_DONE"), True)
        # 전체 파랑이 유지되는 동안에는 회전 없이 최대 속도로 직진한다.
        return Action_Step(
            Motion_Command(self.config.max_speed, 0.0, False, "ALL_BLUE_MAX_PUSH")
        )
