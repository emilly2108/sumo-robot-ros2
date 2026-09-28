import math
from dataclasses import dataclass
from enum import Enum, IntEnum, auto


class Sensor_Color(IntEnum):
    # 색을 감지하지 않았거나 불명확한 바닥 상태를 뜻한다.
    BLACK = 0
    # 빨간 바닥 색을 뜻한다.
    RED = 1
    # 파란 바닥 색을 뜻한다.
    BLUE = 2

class Sensor_Event(Enum):
    # 패턴표에 없는 색 조합으로, 전용 행동을 선택하지 않는 상태다.
    UNKNOWN = auto()
    # 세 센서가 모두 검정인 기본 주행 상태다.
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
    # 카메라가 계산한 목표의 좌우 정규화 오차다.
    target_error: float = 0.0
    # 카메라가 계산한 목표까지의 거리이며 9.9는 목표 없음의 관례값이다.
    target_distance: float = 9.9
    target_found: bool = False
    target_missing_frames: int = 0

    # vision_node.py의 IMU 필터가 계산해 전달한 연속 기울기 각도다.
    tilt_angle_deg: float = 0.0
    # 시간·각도 조건까지 만족해 확정된 기울기 안전 상태다.
    tilt_detected: bool = False

    # 전체·좌·우 벽 거리는 비전 노드가 미터 단위로 갱신한다.
    wall_distance: float = 9.9
    wall_left_distance: float = 9.9
    wall_right_distance: float = 9.9
    wall_last_received_at: float = 0.0

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
    # 평상시 순항, 목표 추격, 근거리 전투, 최대, 후진 선속도다.
    base_speed: float = 0.6
    chase_speed: float = 0.7
    battle_speed: float = 0.8
    max_speed: float = 1.0
    reverse_speed: float = -0.6

    # 허용할 최대 각속도와 고정 회전 행동 속도다.
    max_angular_speed: float = 2.0
    turn_speed: float = 0.8
    kp: float = 1.8
    deadzone: float = 0.03

    # 목표가 가깝다고 보는 거리와 벽 회피를 시작할 거리다.
    opponent_close_distance: float = 0.50
    wall_close_distance: float = 0.20
    no_wall_distance: float = 9.9

    # HFSM 제어 루프를 0.05초로 실행해 /cmd_vel을 초당 20회 발행한다.
    control_period: float = 0.05
    # 기준 거리 10cm
    short_distance: float = 0.10
    turn_45_duration: float = (math.pi / 4.0) / 0.8
    turn_90_duration: float = (math.pi / 2.0) / 0.8
    # 후진 할지 말지 결정하는 5초
    all_color_push_duration: float = 5.0
    # 실제 벽 토픽을 이 시간 이상 받지 못하면 벽 정보를 오래된 것으로 본다.
    wall_timeout: float = 0.5


@dataclass(frozen=True)
class Motion_Command:
    # Twist.linear.x에 들어갈 전진 또는 후진 속도
    linear: float = 0.0
    # Twist.angular.z에 들어갈 왼쪽 또는 오른쪽 회전 속도
    angular: float = 0.0
    # True이면 Brain은 현재 구현에서 0 Twist를 반복 발행한다.
    hard_stop: bool = False
    # 행동·속도 상태를 사람이 식별할 수 있는 로그용 이름이다.
    label: str = "STOP"


@dataclass(frozen=True)
class Action_Step:
    # 이번 제어 주기에 실제 발행할 속도 또는 정지 명령이다.
    command: Motion_Command
    # True이면 node.py가 즉시 다음 행동을 다시 선택한다.
    finished: bool = False
