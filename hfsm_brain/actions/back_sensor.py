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
    BACK_UNTIL_FRONT_RED = auto()
    HARD_STOP_HOLD = auto()

class All_Red_Recovery_Action(Action):
    name = "ALL_RED_RECOVERY"

    def __init__(self, config: Brain_Config):
        self.config = config
        self.phase = All_Red_Phase.PUSH
        self.started_at = 0.0

    def enter(self, now: float, world: World_Model) -> None:
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
            # 111이 5초 이상 계속됐으므로 110이 될 때까지 후진하는 단계로 바꾼다.
            self.phase = All_Red_Phase.BACK_UNTIL_FRONT_RED

        # 현재가 앞 센서만 빨강인 110 상태를 찾으며 후진하는 단계인지 확인한다.
        if self.phase == All_Red_Phase.BACK_UNTIL_FRONT_RED:
            # 센서 사건이 110에 해당하는 FRONT_BOTH_RED가 됐는지 확인한다.
            if world.sensor_event == Sensor_Event.FRONT_BOTH_RED:
                # 목표 센서 상태에 도달했으므로 Hard Stop 유지 단계로 바꾼다.
                self.phase = All_Red_Phase.HARD_STOP_HOLD
            # 아직 110이 아니면 계속 후진해야 한다.
            else:
                # 설정된 일반 후진 속도로 회전 없이 후진한다.
                return Action_Step(
                    Motion_Command(self.config.reverse_speed, 0.0, False, "ALL_RED_REVERSE")
                )

        # 110 상태에서 모터 토크를 유지하는 단계인지 확인한다.
        if self.phase == All_Red_Phase.HARD_STOP_HOLD:
            # Hard Stop 유지 중 센서 상태가 110에서 벗어났는지 확인한다.
            if world.sensor_event != Sensor_Event.FRONT_BOTH_RED:
                # 110이 아니면 유지 행동을 끝내고 Hard Stop 해제를 진행하게 한다.
                return Action_Step(Motion_Command(label="ALL_RED_HOLD_DONE"), True)
            # 110이 유지되는 동안 기존 모터 노드의 Hard Stop 서비스를 요청한다.
            return Action_Step(Motion_Command(hard_stop=True, label="ALL_RED_HARD_STOP"))

        return Action_Step(Motion_Command(label="ALL_RED_DONE"), True)

############################################
class All_Blue_Phase(Enum):
    # 222가 처음 감지되면 최대 속도로 5초 동안 미는 단계다.
    PUSH = auto()
    # 5초가 지나면 모든 센서가 검은색인 000이 될 때까지 후진하는 단계다.
    BACK_UNTIL_BLACK = auto()


# 222 상태가 계속될 때 5초 밀고 000이 될 때까지 후진하는 행동이다.
class All_Blue_Recovery_Action(Action):
    # HFSM 로그에 표시할 행동 이름이다.
    name = "ALL_BLUE_RECOVERY"

    # 222 복구 행동에 필요한 설정과 내부 단계를 준비한다.
    def __init__(self, config: Brain_Config):
        # 최대 속도, 후진 속도, 5초 유지 시간이 들어 있는 설정이다.
        self.config = config
        # 행동은 최대 속도 밀기 단계에서 시작한다.
        self.phase = All_Blue_Phase.PUSH
        # enter() 전에는 시작 시각을 0으로 둔다.
        self.started_at = 0.0

    # 222 복구 행동으로 진입할 때 단계와 시간을 초기화한다.
    def enter(self, now: float, world: World_Model) -> None:
        # 진입 초기화에는 World Model 값을 직접 사용하지 않는다.
        del world
        # 재진입할 때도 최대 속도 밀기부터 다시 시작한다.
        self.phase = All_Blue_Phase.PUSH
        # 5초를 측정할 기준 시각으로 현재 시간을 저장한다.
        self.started_at = now

    # 현재 단계와 센서 사건에 맞는 최대 전진 또는 후진 명령을 만든다.
    def step(self, now: float, world: World_Model) -> Action_Step:
        # 현재가 최대 속도로 5초 동안 미는 단계인지 확인한다.
        if self.phase == All_Blue_Phase.PUSH:
            # 5초가 되기 전에 222 센서 사건이 사라졌는지 확인한다.
            if world.sensor_event != Sensor_Event.ALL_BLUE:
                # 더 이상 222가 아니면 최대 속도 행동을 즉시 종료한다.
                return Action_Step(Motion_Command(label="ALL_BLUE_CANCELLED"), True)
            # 222가 유지된 시간이 설정된 5초보다 짧은지 확인한다.
            if now - self.started_at < self.config.all_color_push_duration:
                # 회전 없이 최대 속도로 계속 전진한다.
                return Action_Step(
                    Motion_Command(self.config.max_speed, 0.0, False, "ALL_BLUE_MAX_PUSH")
                )
            # 222가 5초 이상 계속됐으므로 000을 찾는 후진 단계로 바꾼다.
            self.phase = All_Blue_Phase.BACK_UNTIL_BLACK

        # 세 센서가 모두 검은색인 000 사건에 도달했는지 확인한다.
        if world.sensor_event == Sensor_Event.NONE:
            # 000에 도달했으므로 후진을 끝내고 다음 HFSM 판단으로 돌아간다.
            return Action_Step(Motion_Command(label="ALL_BLUE_REVERSE_DONE"), True)
        # 아직 000이 아니면 설정된 후진 속도로 계속 직선 후진한다.
        return Action_Step(
            Motion_Command(self.config.reverse_speed, 0.0, False, "ALL_BLUE_REVERSE")
        )
