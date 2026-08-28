import math
from dataclasses import dataclass
from enum import Enum, IntEnum, auto


class Sensor_Color(IntEnum):
    BLACK = 0
    RED = 1
    BLUE = 2

class Sensor_Event(Enum):
    UNKNOWN = auto()
    NONE = auto()
    FRONT_RIGHT_RED = auto()
    FRONT_LEFT_RED = auto()
    FRONT_BOTH_RED = auto()
    FRONT_RIGHT_BLUE = auto()
    FRONT_LEFT_BLUE = auto()
    FRONT_BOTH_BLUE = auto()
    BACK_RED = auto()
    BACK_BLUE = auto()
    ALL_RED = auto()
    ALL_BLUE = auto()

# (오른쪽 앞, 왼쪽 앞, 뒤)
SENSOR_EVENTS = {
    #모든 센서가 검은색이다.
    (Sensor_Color.BLACK, Sensor_Color.BLACK, Sensor_Color.BLACK): Sensor_Event.NONE,
    # 오른쪽 앞 센서만 빨간색
    (Sensor_Color.RED, Sensor_Color.BLACK, Sensor_Color.BLACK): Sensor_Event.FRONT_RIGHT_RED,
    #왼쪽 앞 센서만 빨간색
    (Sensor_Color.BLACK, Sensor_Color.RED, Sensor_Color.BLACK): Sensor_Event.FRONT_LEFT_RED,
    #양쪽 앞 센서가 빨간색이고 뒤 센서는 검은색
    (Sensor_Color.RED, Sensor_Color.RED, Sensor_Color.BLACK): Sensor_Event.FRONT_BOTH_RED,
    #오른쪽 앞 센서만 파란색
    (Sensor_Color.BLUE, Sensor_Color.BLACK, Sensor_Color.BLACK): Sensor_Event.FRONT_RIGHT_BLUE,
    #왼쪽 앞 센서만 파란색
    (Sensor_Color.BLACK, Sensor_Color.BLUE, Sensor_Color.BLACK): Sensor_Event.FRONT_LEFT_BLUE,
    #양쪽 앞 센서가 파란색이고 뒤 센서는 검은색
    (Sensor_Color.BLUE, Sensor_Color.BLUE, Sensor_Color.BLACK): Sensor_Event.FRONT_BOTH_BLUE,
    #뒤 센서만 빨간색
    (Sensor_Color.BLACK, Sensor_Color.BLACK, Sensor_Color.RED): Sensor_Event.BACK_RED,
    #뒤 센서만 파란색
    (Sensor_Color.BLACK, Sensor_Color.BLACK, Sensor_Color.BLUE): Sensor_Event.BACK_BLUE,
    #모든 센서가 빨간색
    (Sensor_Color.RED, Sensor_Color.RED, Sensor_Color.RED): Sensor_Event.ALL_RED,
    #모든 센서가 파란색
    (Sensor_Color.BLUE, Sensor_Color.BLUE, Sensor_Color.BLUE): Sensor_Event.ALL_BLUE,
}

@dataclass
class World_Model:
    target_error: float = 0.0
    target_distance: float = 9.9
    target_found: bool = False
    target_missing_frames: int = 0

    wall_distance: float = 9.9
    wall_left_distance: float = 9.9
    wall_right_distance: float = 9.9
    wall_missing_frames: int = 0
    wall_frame_received: bool = False

    # 오른쪽 앞 센서
    sensor1: Sensor_Color = Sensor_Color.BLACK
    # 오른쪽 앞 센서가 검정에서 빨강 또는 파랑으로 바뀐 마지막 시각이다.
    sensor1_detected_at: float = 0.0
    # 왼쪽 앞 센서
    sensor2: Sensor_Color = Sensor_Color.BLACK
    # 왼쪽 앞 센서가 검정에서 빨강 또는 파랑으로 바뀐 마지막 시각이다.
    sensor2_detected_at: float = 0.0
    # 뒤 센서
    sensor3: Sensor_Color = Sensor_Color.BLACK
    sensor_event: Sensor_Event = Sensor_Event.NONE
    sensor_updated_at: float = 0.0

@dataclass(frozen=True)
class Brain_Config:
    base_speed: float = 0.6
    chase_speed: float = 0.7
    battle_speed: float = 0.8
    max_speed: float = 0.8
    reverse_speed: float = -0.6

    max_angular_speed: float = 2.0
    turn_speed: float = 0.8
    kp: float = 1.8
    deadzone: float = 0.03

    opponent_close_distance: float = 0.50
    wall_close_distance: float = 0.20
    no_wall_distance: float = 9.9

    # HFSM 제어 루프를 0.02초
    control_period: float = 0.02
    # 기준 거리 10cm
    short_distance: float = 0.10
    turn_45_duration: float = (math.pi / 4.0) / 0.8
    turn_90_duration: float = (math.pi / 2.0) / 0.8
    # 후진 할지 말지 결정하는 5초
    all_color_push_duration: float = 5.0


@dataclass(frozen=True)
class Motion_Command:
    # Twist.linear.x에 들어갈 전진 또는 후진 속도
    linear: float = 0.0
    # Twist.angular.z에 들어갈 왼쪽 또는 오른쪽 회전 속도
    angular: float = 0.0
    hard_stop: bool = False
    label: str = "STOP"


@dataclass(frozen=True)
class Action_Step:
    command: Motion_Command
    finished: bool = False
