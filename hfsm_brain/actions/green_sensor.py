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
    BACK_UNTIL_CLEAR = auto()


class Green_Close_Front_Color_Action(Action):
    """Handle a same-color front sensor pair while the green target is very close."""

    name = "GREEN_CLOSE_FRONT_COLOR"
    interruptible_by_green = False

    def __init__(self, config: Brain_Config, color: Sensor_Color):
        self.config = config
        self.color = color
        self.phase = Green_Close_Front_Phase.BACK_UNTIL_CLEAR

    def key(self):
        return (type(self), self.color)

    def step(self, now: float, world: World_Model) -> Action_Step:
        del now
        if self.phase == Green_Close_Front_Phase.BACK_UNTIL_CLEAR:
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
                        f"GREEN_CLOSE_{self.color.name}_BACK_UNTIL_CLEAR",
                    )
                )

            return Action_Step(
                Motion_Command(label=f"GREEN_CLOSE_{self.color.name}_CLEAR"),
                finished=True,
            )

        return Action_Step(Motion_Command(label="GREEN_CLOSE_FRONT_DONE"), True)


class Green_Close_Back_Blue_Phase(Enum):
    FOLLOW_GREEN = auto()
    BACK_UNTIL_FRONT_CLEAR = auto()


class Green_Close_Back_Blue_Action(Action):
    """Handle a close green target while the rear blue sensor and both front sensors are active."""

    name = "GREEN_CLOSE_BACK_BLUE"
    interruptible_by_green = False

    def __init__(self, config: Brain_Config):
        self.config = config
        self.phase = Green_Close_Back_Blue_Phase.FOLLOW_GREEN

    def step(self, now: float, world: World_Model) -> Action_Step:
        del now

        if not world.target_found or world.sensor3 != Sensor_Color.BLUE:
            return Action_Step(Motion_Command(label="GREEN_CLOSE_BACK_BLUE_DONE"), True)

        if not (0.0 < world.target_distance <= 0.05):
            return Action_Step(Motion_Command(label="GREEN_CLOSE_BACK_BLUE_DISTANCE_DONE"), True)

        front_has_color = (
            world.sensor1 != Sensor_Color.BLACK
            and world.sensor2 != Sensor_Color.BLACK
        )
        if front_has_color:
            self.phase = Green_Close_Back_Blue_Phase.BACK_UNTIL_FRONT_CLEAR

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
            self.phase = Green_Close_Back_Blue_Phase.FOLLOW_GREEN

        return Action_Step(green_sensor_follow_command(world, self.config))


class Green_Close_All_Red_Action(Action):
    """Reverse while a close green target and all-red sensor state are both active."""

    name = "GREEN_CLOSE_ALL_RED"
    interruptible_by_green = False

    def __init__(self, config: Brain_Config):
        self.config = config

    def step(self, now: float, world: World_Model) -> Action_Step:
        del now
        if world.sensor_event != Sensor_Event.ALL_RED:
            return Action_Step(Motion_Command(label="GREEN_CLOSE_ALL_RED_DONE"), True)
        return Action_Step(
            Motion_Command(
                self.config.reverse_speed,
                0.0,
                False,
                "GREEN_CLOSE_ALL_RED_REVERSE",
            )
        )
