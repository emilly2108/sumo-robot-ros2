from typing import Optional
from rclpy.node import Node
from .models import Brain_Config, Motion_Command, Sensor_Event, World_Model

def now_seconds(node: Node) -> float:
    # ROS clock의 나노초를 행동 시간 비교에 쉬운 초 단위 float로 바꾼다.
    return node.get_clock().now().nanoseconds / 1e9

def green_follow_command(world: World_Model, config: Brain_Config) -> Motion_Command:
    # Green tracking normally uses chase or battle speed.  When only the rear
    # color sensor is active, keep the same green steering but use max speed.
    if world.sensor_event == Sensor_Event.BACK_RED:
        # 뒤 빨강은 상대와의 접촉 상황으로 보고 최대 속도 추격을 쓴다.
        speed = config.max_speed
        label = "GREEN_BACK_RED_MAX"
    elif world.sensor_event == Sensor_Event.BACK_BLUE:
        # 뒤 파랑도 같은 최대 속도 추격 규칙을 적용한다.
        speed = config.max_speed
        label = "GREEN_BACK_BLUE_MAX"
    elif 0.0 < world.target_distance <= 0.40:
        # 상대가 40 cm 안이면 제어 가능한 전투 속도로 낮춘다.
        speed = config.battle_speed
        label = "GREEN_BATTLE"
    else:
        # 나머지 유효 목표에는 일반 추격 속도를 쓴다.
        speed = config.chase_speed
        label = "GREEN_CHASE"

    # 목표가 오른쪽이면 음수, 왼쪽이면 양수가 되도록 비례 조향을 계산한다.
    angular = -config.kp * world.target_error
    if abs(world.target_error) < config.deadzone:
        angular = 0.0
    # 비례 제어값이 모터에 과도한 회전 명령을 내지 않도록 포화시킨다.
    angular = max(min(angular, config.max_angular_speed), -config.max_angular_speed)

    return Motion_Command(speed, angular, False, label)

def green_sensor_follow_command(
    world: World_Model,
    config: Brain_Config,
    force_max_speed: bool = False,
) -> Motion_Command:
    """센서 행동 중 초록색을 추격할 때 사용하는 명령이다."""
    # 기본값은 일반 초록 추격 속도다.
    speed = config.chase_speed
    label = "GREEN_SENSOR_CHASE"
    if force_max_speed:
        # 호출자가 강제 최대 속도를 요청하면 거리 조건보다 우선한다.
        speed = config.max_speed
        label = (
            "GREEN_BACK_BLUE_MAX"
            if world.sensor_event == Sensor_Event.BACK_BLUE
            else "GREEN_BACK_RED_MAX"
        )
    elif 0.0 < world.target_distance <= 0.05:
        # 센서 전용 근접 추격은 5 cm 안에서 최대 속도를 쓴다.
        speed = config.max_speed
        label = "GREEN_SENSOR_MAX"

    # 일반 추격과 같은 비례 조향식을 사용한다.
    angular = -config.kp * world.target_error
    if abs(world.target_error) < config.deadzone:
        angular = 0.0
    angular = max(min(angular, config.max_angular_speed), -config.max_angular_speed)

    return Motion_Command(speed, angular, False, label)

def wall_avoid_direction(world: World_Model, config: Brain_Config) -> Optional[float]:
    # 좌우 깊이 중 하나라도 벽 기준 거리 안에 들어왔는지 확인한다.
    left_close = 0.0 < world.wall_left_distance <= config.wall_close_distance
    right_close = 0.0 < world.wall_right_distance <= config.wall_close_distance
    if not (left_close or right_close):
        # 가까운 벽이 없으면 회피 행동을 선택하지 않음을 뜻하는 None을 반환한다.
        return None
    if world.wall_left_distance > world.wall_right_distance:
        # 왼쪽 여유가 더 크면 왼쪽으로 회전한다.
        return 1.0
    if world.wall_right_distance > world.wall_left_distance:
        # 오른쪽 여유가 더 크면 오른쪽으로 회전한다.
        return -1.0
    # 양쪽 여유가 같거나 불명확하면 재현 가능한 기본값인 왼쪽을 선택한다.
    return 1.0
