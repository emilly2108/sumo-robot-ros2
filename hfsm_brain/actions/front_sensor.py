#######################################
from enum import Enum, auto
from typing import Optional

from ..models import Action_Step, Brain_Config, Motion_Command, Sensor_Color, World_Model
from .base import Action


class Front_Side(Enum):
    RIGHT = auto()
    LEFT = auto()

class Front_Color_Phase(Enum):
    FORWARD_WITH_ONE = auto()
    BACK_UNTIL_CLEAR = auto()
    TURN_TO_LATER_SIDE = auto()

class Front_Color_Avoid_Action(Action):
    name = "FRONT_COLOR_AVOID"

    def __init__(
        self,
        config: Brain_Config,
        color: Sensor_Color,
        first_side: Optional[Front_Side],
        interruptible_by_green: bool = True,
    ):
        self.config = config
        self.color = color
        self.first_side = first_side
        self.interruptible_by_green = interruptible_by_green
        self.phase = (
            Front_Color_Phase.BACK_UNTIL_CLEAR
            if first_side is None
            else Front_Color_Phase.FORWARD_WITH_ONE
        )
        self.turn_direction = 1.0
        self.started_at = 0.0

    def key(self):
        return (type(self), self.color, self.first_side)

    def enter(self, now: float, world: World_Model) -> None:
        # 한쪽부터 감지했다면 직진 단계, 양쪽 감지로 시작하면 후진 단계다.
        self.phase = (
            Front_Color_Phase.BACK_UNTIL_CLEAR
            if self.first_side is None
            else Front_Color_Phase.FORWARD_WITH_ONE
        )
        # 양쪽 감지로 바로 시작한 경우 콜백 감지 시각으로 나중 센서를 결정한다.
        if self.first_side is None:
            self.turn_direction = self._later_side_direction(world)
        self.started_at = now

    # 두 센서가 거의 동시에 같은 색으로 시작했을 때 더 늦게 감지된 쪽을 정한다.
    def _later_side_direction(self, world: World_Model) -> float:
        # 오른쪽 센서의 색 감지 시작 시각이 더 늦으면 오른쪽으로 회전한다.
        if world.sensor1_detected_at > world.sensor2_detected_at:
            # 음수 각속도 방향인 오른쪽 회전을 뜻한다.
            return -1.0
        # 왼쪽 센서의 색 감지 시작 시각이 더 늦으면 왼쪽으로 회전한다.
        if world.sensor2_detected_at > world.sensor1_detected_at:
            # 양수 각속도 방향인 왼쪽 회전을 뜻한다.
            return 1.0
        # 감지 시각도 같아 순서를 알 수 없으면 기본값으로 왼쪽을 선택한다.
        return 1.0

    # 한쪽에서 시작해 같은 색이 양쪽이 됐을 때 나중 센서 방향을 정한다.
    def _direction_from_first_side(self, world: World_Model) -> float:
        # 오른쪽이 먼저였다면 나중에 감지된 쪽은 왼쪽이다.
        if self.first_side == Front_Side.RIGHT:
            # 양수 각속도 방향인 왼쪽 회전을 반환한다.
            return 1.0
        # 왼쪽이 먼저였다면 나중에 감지된 쪽은 오른쪽이다.
        if self.first_side == Front_Side.LEFT:
            # 음수 각속도 방향인 오른쪽 회전을 반환한다.
            return -1.0
        # 첫 센서 기록이 없다면 실제 감지 시작 시각을 비교한다.
        return self._later_side_direction(world)

    def step(self, now: float, world: World_Model) -> Action_Step:
        right_has_color = world.sensor1 == self.color
        left_has_color = world.sensor2 == self.color

        # 한쪽 센서만 감지된 채 기본 속도로 직진하는 단계인지 확인한다.
        if self.phase == Front_Color_Phase.FORWARD_WITH_ONE:
            if right_has_color and left_has_color:
                # 먼저 감지한 센서의 반대쪽이 나중에 감지된 센서 방향이다.
                self.turn_direction = self._direction_from_first_side(world)
                # 이제 양쪽 앞 센서가 모두 검정이 될 때까지 후진하는 단계로 간다.
                self.phase = Front_Color_Phase.BACK_UNTIL_CLEAR
                # 단계가 바뀐 즉시 공통 후진 속도로 직선 후진한다.
                return Action_Step(
                    Motion_Command(
                        self.config.reverse_speed,
                        0.0,
                        False,
                        f"FRONT_{self.color.name}_BOTH_BACK",
                    )
                )

            # 행동이 추적하던 색이 양쪽 센서에서 모두 사라졌는지 확인한다.
            if not right_has_color and not left_has_color:
                # 한쪽 감지 상태가 끝났으므로 현재 행동을 종료하고 전체 판단으로 돌아간다.
                return Action_Step(
                    Motion_Command(label=f"FRONT_{self.color.name}_ONE_CLEARED"),
                    finished=True,
                )

            # 오른쪽 센서만 현재 색을 보고 있으면 오른쪽을 첫 감지 방향으로 유지한다.
            if right_has_color and not left_has_color:
                self.first_side = Front_Side.RIGHT
            # 왼쪽 센서만 현재 색을 보고 있으면 왼쪽을 첫 감지 방향으로 유지한다.
            elif left_has_color and not right_has_color:
                self.first_side = Front_Side.LEFT

            # 한쪽 센서만 같은 색을 보는 동안 회전 없이 기본 속도로 계속 직진한다.
            return Action_Step(
                Motion_Command(
                    self.config.base_speed,
                    0.0,
                    False,
                    f"FRONT_{self.color.name}_ONE_FORWARD",
                )
            )

        # 같은 색을 양쪽에서 본 뒤 색 영역을 빠져나가기 위해 후진하는 단계다.
        if self.phase == Front_Color_Phase.BACK_UNTIL_CLEAR:
            # 색 종류와 관계없이 양쪽 앞 센서가 모두 검정인지 확인한다.
            front_is_clear = (
                world.sensor1 == Sensor_Color.BLACK
                and world.sensor2 == Sensor_Color.BLACK
            )
            # 아직 한쪽이라도 색을 감지하면 공통 후진 속도로 계속 후진한다.
            if not front_is_clear:
                return Action_Step(
                    Motion_Command(
                        self.config.reverse_speed,
                        0.0,
                        False,
                        f"FRONT_{self.color.name}_BACK_UNTIL_CLEAR",
                    )
                )
            # 양쪽 센서가 모두 검정이 됐으므로 나중 감지 방향 45도 회전 단계로 간다.
            self.phase = Front_Color_Phase.TURN_TO_LATER_SIDE
            # 45도 회전 시간을 측정할 기준 시각을 현재 시간으로 저장한다.
            self.started_at = now

        # 나중에 같은 색을 감지했던 센서 쪽으로 45도 회전하는 단계다.
        if self.phase == Front_Color_Phase.TURN_TO_LATER_SIDE:
            # 설정된 45도 회전 시간이 아직 남았는지 확인한다.
            if now - self.started_at < self.config.turn_45_duration:
                # 로그에 표시할 실제 왼쪽 또는 오른쪽 회전 이름을 만든다.
                side = "LEFT" if self.turn_direction > 0.0 else "RIGHT"
                # 선속도 0으로 제자리에서 나중 감지 방향으로 회전한다.
                return Action_Step(
                    Motion_Command(
                        0.0,
                        self.turn_direction * self.config.turn_speed,
                        False,
                        f"FRONT_{self.color.name}_TURN_{side}_45",
                    )
                )
            # 45도 회전이 끝났으므로 행동을 종료해 일반 직진 판단으로 넘어간다.
            return Action_Step(
                Motion_Command(label=f"FRONT_{self.color.name}_AVOID_DONE"),
                finished=True,
            )

        return Action_Step(Motion_Command(label="FRONT_COLOR_AVOID_DONE"), True)
