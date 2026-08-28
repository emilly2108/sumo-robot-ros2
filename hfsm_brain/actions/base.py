from ..models import Action_Step, World_Model

class Action:
    name = "ACTION"
    locked = True
    interruptible_by_green = True

    def enter(self, now: float, world: World_Model) -> None:
        del now, world

    def exit(self, cancelled: bool) -> None:
        del cancelled

    def can_interrupt_for_green(self) -> bool:
        return self.interruptible_by_green

    def key(self):
        return (type(self),)

    def step(self, now: float, world: World_Model) -> Action_Step:
        raise NotImplementedError
