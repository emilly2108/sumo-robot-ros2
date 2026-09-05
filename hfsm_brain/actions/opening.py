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
    TURN_LEFT_45 = auto()
    GO_STRAIGHT = auto()

class Opening_Action(Action):
    name = "OPENING"

    def __init__(self, config: Brain_Config):
        self.config = config
        self.phase = Opening_Phase.TURN_LEFT_45
        self.started_at = 0.0

    def enter(self, now: float, world: World_Model) -> None:
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
