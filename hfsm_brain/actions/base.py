from ..models import Action_Step, World_Model

class Action:
    # 각 하위 행동이 로그에 제공할 기본 이름이다.
    name = "ACTION"
    # True면 node가 매 주기 일반 행동으로 다시 선택하지 않고 완료를 기다린다.
    locked = True
    # True면 초록 목표 발견 시 현재 행동을 중단할 수 있다.
    interruptible_by_green = True

    def enter(self, now: float, world: World_Model) -> None:
        # 기본 행동은 초기화할 내부 상태가 없으므로 인자를 명시적으로 사용하지 않는다.
        del now, world

    def exit(self, cancelled: bool) -> None:
        # 기본 행동은 정상 완료·취소 때 별도 정리 작업이 없다.
        del cancelled

    def can_interrupt_for_green(self) -> bool:
        return self.interruptible_by_green

    def key(self):
        # 동일 클래스 행동은 기본적으로 같은 상태 전환 대상으로 비교한다.
        return (type(self),)

    def step(self, now: float, world: World_Model) -> Action_Step:
        # 실제 행동 클래스가 반드시 이번 제어 주기의 명령을 구현해야 한다.
        raise NotImplementedError
