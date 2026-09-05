from typing import Optional
from rclpy.node import Node
from .models import Brain_Config, Motion_Command, World_Model

def now_seconds(node: Node) -> float:
    return node.get_clock().now().nanoseconds / 1e9

def green_follow_command(world: World_Model, config: Brain_Config) -> Motion_Command:
    speed = config.chase_speed
    label = "GREEN_CHASE"
    if 0.0 < world.target_distance <= 0.40:
        speed = config.battle_speed
        label = "GREEN_BATTLE"

    angular = -config.kp * world.target_error
    if abs(world.target_error) < config.deadzone:
        angular = 0.0
    angular = max(min(angular, config.max_angular_speed), -config.max_angular_speed)

    return Motion_Command(speed, angular, False, label)


def green_sensor_follow_command(world: World_Model, config: Brain_Config) -> Motion_Command:
    """센서 행동 중 초록색을 추격할 때 사용하는 50mm 기준 명령이다."""
    speed = config.chase_speed
    label = "GREEN_SENSOR_CHASE"
    if 0.0 < world.target_distance <= 0.05:
        speed = config.max_speed
        label = "GREEN_SENSOR_MAX"

    angular = -config.kp * world.target_error
    if abs(world.target_error) < config.deadzone:
        angular = 0.0
    angular = max(min(angular, config.max_angular_speed), -config.max_angular_speed)

    return Motion_Command(speed, angular, False, label)

def wall_avoid_direction(world: World_Model, config: Brain_Config) -> Optional[float]:
    left_close = 0.0 < world.wall_left_distance <= config.wall_close_distance
    right_close = 0.0 < world.wall_right_distance <= config.wall_close_distance
    if not (left_close or right_close):

        return None
    if world.wall_left_distance > world.wall_right_distance:
        return 1.0
    if world.wall_right_distance > world.wall_left_distance:
        return -1.0
    return 1.0
