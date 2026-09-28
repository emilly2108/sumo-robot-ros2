#############################################
from enum import Enum, auto

from ..helpers import green_follow_command, green_sensor_follow_command
from ..models import Action_Step, Brain_Config, Motion_Command, World_Model
from .base import Action

class Cruise_Action(Action):
    name = "CRUISE"
    locked = False
    interruptible_by_green = True

    def __init__(self, config: Brain_Config):
        self.config = config

    # 현재 상황과 관계없이 이번 제어 주기의 기본 직진 명령을 만든다.
    def step(self, now: float, world: World_Model) -> Action_Step:
        # Cruise 자체에서는 시간과 World Model을 직접 사용하지 않는다.
        del now, world
        # 기본 선속도와 회전 속도 0으로 계속 직진하라고 반환한다.
        return Action_Step(Motion_Command(self.config.base_speed, 0.0, False, "CRUISE"))


class Green_Follow_Action(Action):
    name = "GREEN_FOLLOW"
    locked = False

    def __init__(self, config: Brain_Config):
        self.config = config

    # 최신 World Model을 이용해 이번 추격 명령을 만든다.
    def step(self, now: float, world: World_Model) -> Action_Step:
        # 추격 계산은 시간 대신 최신 목표 정보만 사용한다.
        del now
        if not world.target_found:
            # 현재 행동을 끝내서 node.py가 다시 Cruise 등 다음 행동을 고르게 한다.
            return Action_Step(Motion_Command(label="GREEN_LOST"), True)
        # 목표가 있으면 helpers.py에서 선속도와 각속도를 계산해 반환한다.
        return Action_Step(green_follow_command(world, self.config))


class Green_Sensor_Follow_Action(Action):
    """초록색이 보이고 센서 분기에서 계속 추격해야 할 때의 행동이다."""

    name = "GREEN_SENSOR_FOLLOW"
    locked = False

    def __init__(self, config: Brain_Config, force_max_speed: bool = False):
        # 행동 공통 속도 설정과 강제 최대 속도 옵션을 보관한다.
        self.config = config
        self.force_max_speed = force_max_speed

    def key(self):
        # 같은 클래스라도 강제 최대 속도 여부가 다르면 다른 행동으로 취급한다.
        return (type(self), self.force_max_speed)

    def step(self, now: float, world: World_Model) -> Action_Step:
        # 이번 판단에는 시간보다 최신 목표·센서 상태만 필요하다.
        del now
        if not world.target_found:
            return Action_Step(Motion_Command(label="GREEN_SENSOR_LOST"), True)
        return Action_Step(
            green_sensor_follow_command(
                world,
                self.config,
                force_max_speed=self.force_max_speed,
            )
        )

class Far_Green_Phase(Enum):
    # 먼 목표 쪽으로 먼저 고정된 45도 좌회전을 수행하는 단계다.
    TURN_LEFT_45 = auto()
    # 회전 뒤 10 cm만 직진해 시야와 위치를 바꾸는 단계다.
    FORWARD_SHORT = auto()


class Far_Green_Turn_Action(Action):
    name = "FAR_GREEN_TURN_45"
    interruptible_by_green = False

    # 회전 설정과 방향을 받아 내부 상태를 준비한다.
    def __init__(self, config: Brain_Config, direction: float = 1.0):
        # 45도 회전 시간과 회전 속도를 얻기 위한 설정이다.
        self.config = config
        # +1.0은 왼쪽, -1.0은 오른쪽 회전
        self.direction = direction
        # 행동 진입 전에는 시작 시각을 0으로 둔다.
        self.started_at = 0.0
        self.phase = Far_Green_Phase.TURN_LEFT_45

    # 같은 클래스라도 회전 방향이 다르면 다른 행동으로 구분한다.
    def key(self):
        # 클래스 종류와 회전 방향을 함께 비교 키로 반환한다.
        return (type(self), self.direction)

    # 행동으로 전환된 순간 회전 기준 시각을 기록한다.
    def enter(self, now: float, world: World_Model) -> None:
        # 이 행동은 중간에 목표 방향을 다시 읽지 않으므로 World Model은 쓰지 않는다.
        del world
        self.phase = Far_Green_Phase.TURN_LEFT_45
        self.started_at = now

    # 경과 시간으로 45도 회전의 진행과 종료를 결정한다.
    def step(self, now: float, world: World_Model) -> Action_Step:
        # This action uses its stored phases instead of changing direction from
        # the latest target observation while it is in progress.
        del world
        if self.phase == Far_Green_Phase.TURN_LEFT_45:
            if now - self.started_at < self.config.turn_45_duration:
                return Action_Step(
                    Motion_Command(
                        0.0,
                        self.direction * self.config.turn_speed,
                        False,
                        "FAR_GREEN_TURN_45",
                    )
                )
            self.phase = Far_Green_Phase.FORWARD_SHORT
            # 회전이 끝난 시각부터 짧은 직진 시간을 새로 잰다.
            self.started_at = now

        # 설정된 10 cm를 기본 속도로 갈 때 필요한 시간을 계산한다.
        forward_duration = self.config.short_distance / self.config.base_speed
        # 짧은 전진이 끝나기 전에는 방향 보정 없이 직진한다.
        if now - self.started_at < forward_duration:
            return Action_Step(
                Motion_Command(
                    self.config.base_speed,
                    0.0,
                    False,
                    "FAR_GREEN_FORWARD_SHORT",
                )
            )

        # 회전과 짧은 전진이 모두 끝나면 다음 일반 추격 판단을 허용한다.
        return Action_Step(Motion_Command(label="FAR_GREEN_TURN_DONE"), True)


class Wall_Avoid_Action(Action):
    name = "WALL_AVOID"
    # 초록색이 보이면 벽 판단을 끄고 초록색 추격으로 즉시 전환한다.
    interruptible_by_green = True

    def __init__(self, config: Brain_Config, direction: float):
        # 90도 회전 시간과 회전 속도가 들어 있는 설정이다.
        self.config = config
        # +1.0은 왼쪽, -1.0은 오른쪽 벽 회피 회전을 뜻한다.
        self.direction = direction
        # enter()에서 실제 시작 시각을 저장하기 전까지 0으로 둔다.
        self.started_at = 0.0

    # 같은 벽 회피라도 회전 방향이 다르면 별개의 행동으로 구분한다.
    def key(self):
        # 클래스 종류와 방향을 하나의 식별 키로 반환한다.
        return (type(self), self.direction)

    # 벽 회피 행동이 시작될 때 회전 시작 시각을 저장한다.
    def enter(self, now: float, world: World_Model) -> None:
        # 회전 방향은 이미 생성 시 결정됐으므로 World Model을 사용하지 않는다.
        del world
        # 현재 시간을 90도 회전의 기준 시각으로 저장한다.
        self.started_at = now

    # 경과 시간에 따라 90도 회전을 유지하거나 종료한다.
    def step(self, now: float, world: World_Model) -> Action_Step:
        # 회전 중에는 벽 거리를 다시 계산하지 않고 정해진 행동을 끝까지 수행한다.
        del world
        # 90도 회전에 필요한 시간이 모두 지났는지 확인한다.
        if now - self.started_at >= self.config.turn_90_duration:
            # 벽 회피를 끝내고 다음 행동을 다시 선택하게 한다.
            return Action_Step(Motion_Command(label="WALL_AVOID_DONE"), True)
        # 로그에 왼쪽과 오른쪽 중 실제 회전 방향을 표시할 문자열을 만든다.
        side = "LEFT" if self.direction > 0.0 else "RIGHT"
        # 선속도 0과 방향이 적용된 각속도로 제자리 90도 회전을 유지한다.
        return Action_Step(
            Motion_Command(
                0.0,
                self.direction * self.config.turn_speed,
                False,
                f"WALL_AVOID_{side}_90",
            )
        )
