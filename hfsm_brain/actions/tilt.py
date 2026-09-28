from ..models import Action_Step, Brain_Config, Motion_Command, World_Model
from .base import Action


class Tilt_Turn_Action(Action):
    # 로그와 상태 전환에 쓸 기울기 복구 행동의 이름이다.
    name = "IMU_TILT_TURN"
    # 기울기 복구는 완료 조건 전까지 다른 일반 행동으로 바꾸지 않는다.
    locked = True
    # 초록 목표가 나타나도 기울기 안전 동작을 중단하지 않는다.
    interruptible_by_green = False

    def __init__(self, config: Brain_Config):
        # 회전 속도를 읽을 공통 설정을 보관한다.
        self.config = config

    def step(self, now: float, world: World_Model) -> Action_Step:
        # 이 행동은 시간 경과가 아니라 최신 기울기 True/False로만 끝난다.
        del now
        # 필터가 해제되면 행동을 완료해 일반 우선순위 판단으로 돌려보낸다.
        if not world.tilt_detected:
            return Action_Step(Motion_Command(label="IMU_TILT_CLEARED"), True)

        # 기울기가 유지되는 동안 선속도 없이 고정된 좌회전을 계속 지시한다.
        return Action_Step(
            Motion_Command(
                0.0,
                self.config.turn_speed,
                False,
                "IMU_TILT_TURN_LEFT",
            )
        )
