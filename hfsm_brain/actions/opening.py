from enum import Enum, auto

from ..helpers import wall_avoid_direction
from ..models import (
    Action_Step,
    Brain_Config,
    Motion_Command,
    Sensor_Event,
    World_Model,
)
from .base import Action

class Opening_Phase(Enum):
    # 스위치를 켠 직후 반드시 수행할 약 45도 좌회전 단계다.
    TURN_LEFT_45 = auto()
    # 회전 후 목표·벽·앞 색상 조건을 만날 때까지 직진하는 단계다.
    GO_STRAIGHT = auto()

class Opening_Action(Action):
    name = "OPENING"

    def __init__(self, config: Brain_Config):
        # 회전 시간·속도와 직진 속도를 읽을 공통 설정을 저장한다.
        self.config = config
        self.phase = Opening_Phase.TURN_LEFT_45
        self.started_at = 0.0

    def enter(self, now: float, world: World_Model) -> None:
        # 행동 재진입 때마다 첫 회전부터 다시 시작하고 시작 시각을 기록한다.
        del world
        self.phase = Opening_Phase.TURN_LEFT_45
        self.started_at = now

    def can_interrupt_for_green(self) -> bool:
        # 시작 45도 회전은 먼저 끝낸 뒤 초록색 추격 여부를 판단한다.
        return self.phase == Opening_Phase.GO_STRAIGHT

    def step(self, now: float, world: World_Model) -> Action_Step:
        if self.phase == Opening_Phase.TURN_LEFT_45:
            if now - self.started_at < self.config.turn_45_duration:
                return Action_Step(
                    Motion_Command(
                        0.0,
                        self.config.turn_speed,
                        False,
                        "OPENING_TURN_LEFT_45",
                    )
                )
            # 정해진 회전 시간이 끝나면 다음 제어 주기부터 직진 단계로 바꾼다.
            self.phase = Opening_Phase.GO_STRAIGHT
            self.started_at = now

        if world.target_found or wall_avoid_direction(world, self.config) is not None:
            return Action_Step(Motion_Command(label="OPENING_DONE"), True)

        # 첫 45도 회전은 반드시 끝낸다. 그 뒤 직진 중 앞 양쪽이 같은 색이면
        # 기존 앞 센서 후진 회피 행동을 선택할 수 있도록 오프닝을 끝낸다.
        if world.sensor_event in (
            Sensor_Event.FRONT_BOTH_RED,
            Sensor_Event.FRONT_BOTH_BLUE,
        ):
            return Action_Step(
                Motion_Command(label="OPENING_FRONT_BOTH_COLOR"),
                finished=True,
            )

        return Action_Step(
            Motion_Command(self.config.base_speed, 0.0, False, "OPENING_STRAIGHT")
        )
