# -*- coding: utf-8 -*-
import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


# 초록색 인식
class GreenFollowerNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        self.init_green_params()

        self.create_subscription(Float32, '/target_error', self.error_callback, 10)
        self.create_subscription(Float32, '/target_distance', self.distance_callback, 10)
        self.create_subscription(Bool, '/target_found', self.found_callback, 10)

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('Green follower brain node started.')

    def init_green_params(self):
        #속도 제한값
        # limit_linear_vel은 일반 속력시
        # boost_linear_vel은 최대 속력시
        self.limit_linear_vel = 0.6
        self.normal_linear_vel = self.limit_linear_vel
        self.boost_linear_vel = 0.8
        self.max_linear_vel = self.boost_linear_vel
        self.limit_angular_vel = 2.0

        # 가속도 제한
        self.accel_linear = 0.6
        self.accel_angular = 1.5

        # 초록색 추격 제어값
        # Kp가 클수록 초록색이 중앙에서 벗어났을 때 더 강하게 회전한다.
        # deadzone 안쪽이면 거의 중앙이라고 보고 회전 명령을 0으로 만든다.
        self.Kp = 1.8
        self.deadzone = 0.03
        self.dt = 0.02

        self.target_error = 0.0
        self.target_distance = 9.9
        self.target_found = False
        self.camera_target_found = False

        # last_seen 일정 시간 이상 갱신이 없으면 초록색을 놓친 것으로 처리
        self.last_seen = self.get_clock().now()
        self.last_update_time = self.get_clock().now()

        # 실제로 마지막에 publish한 속도 상태
        self.current_v = 0.0
        self.current_w = 0.0

    def error_callback(self, msg):
        self.target_error = msg.data
        self.last_seen = self.get_clock().now()

    def distance_callback(self, msg):
        self.target_distance = msg.data
        self.last_seen = self.get_clock().now()

    # False가 들어오면 예전 거리값이 남아 오류나지 않도록 reset_green_target()으로 초기화
    def green_found_callback(self, msg):
        if msg.data:
            self.camera_target_found = True
            self.target_found = True
        else:
            self.reset_green_target()
        # 상대가 안보이기 시작하면 시간 재기
        self.last_update_time = self.get_clock().now()
        self.last_seen = self.get_clock().now()

    # 초록색을 잃었을 때 공통으로 부르는 초기화 함수
    # 초록색을 잃었을때 이전 거리가 남아서 오류 발생 시킬 때가 있음 => 안보이면 9.9로 초기화
    def reset_green_target(self):
        self.camera_target_found = False
        self.target_found = False
        self.target_error = 0.0
        self.target_distance = 9.9

    # 초록색을 추격할 때 목표 속도(target_v)와 회전 속도(target_w)를 계산한다.
    def compute_green_follow_command(self):
        target_v = self.limit_linear_vel
        target_w = -self.Kp * self.target_error

        if abs(self.target_error) < self.deadzone:
            target_w = 0.0

        return target_v, target_w

    # 일반 직진 명령
    def normal_forward_command(self):
        return self.normal_linear_vel, 0.0

    # 최대 속도 직진 명령
    def boost_forward_command(self):
        return self.boost_linear_vel, 0.0

    def control_loop(self):
        stale_time = (self.get_clock().now() - self.last_seen).nanoseconds / 1e9
        if stale_time > 0.5:
            self.reset_green_target()

        if self.target_found:
            target_v, target_w = self.compute_green_follow_command()
        else:
            # 초록색이 안 보이면 빠져나가는 것이 아니라 제자리 회전하며 탐색
            target_v, target_w = 0.0, 0.8

        self.publish_smoothed_command(target_v, target_w)

    # 목표 속도를 바로 내보내지 않고 조금씩 따라가게 만든다.
    def publish_smoothed_command(self, target_v, target_w):
        twist = Twist()

        # 전진/후진 속도를 목표값까지 한 번에 바꾸지 않고 max_v_step만큼만 변화시킨다.
        v_diff = target_v - self.current_v
        max_v_step = self.accel_linear * self.dt
        if abs(v_diff) > max_v_step:
            self.current_v += math.copysign(max_v_step, v_diff)
        else:
            self.current_v = target_v

        # 회전 속도도 같은 방식으로 부드럽게 변화시킨다.
        w_diff = target_w - self.current_w
        max_w_step = self.accel_angular * self.dt
        if abs(w_diff) > max_w_step:
            self.current_w += math.copysign(max_w_step, w_diff)
        else:
            self.current_w = target_w

        # 전체 최대 속도 제한을 적용(안전책)
        self.current_v = max(min(self.current_v, self.max_linear_vel), -self.max_linear_vel)
        self.current_w = max(min(self.current_w, self.limit_angular_vel), -self.limit_angular_vel)

        # Twist 메시지에 실제 속도값을 넣어서 /cmd_vel로 publish한다.
        twist.linear.x = self.current_v
        twist.angular.z = self.current_w
        self.pub_vel.publish(twist)



# 배틀 모드
class BattleMode(GreenFollowerNode):

    def __init__(self):
        super().__init__()
        self.init_battle_params()

        # vision_node.py에서 계산한 값들
        self.create_subscription(Float32, '/wall_distance', lambda msg: self.update_sensor_value(msg, 'wall_distance'), 10)
        self.create_subscription(Float32, '/wall_left_distance', lambda msg: self.update_sensor_value(msg, 'wall_left_distance'), 10)
        self.create_subscription(Float32, '/wall_right_distance', lambda msg: self.update_sensor_value(msg, 'wall_right_distance'), 10)
        self.create_subscription(Bool, '/red_between_green', self.red_between_green_callback, 10)

        self.get_logger().info('Battle mode ready.')

    # 벽 회피
    # None = 벽 회피 안 함/1.0 = 왼쪽으로 회피/-1.0 = 오른쪽으로 회피
    def get_wall_avoid_direction(self):
        if self.target_found and 0.0 < self.target_distance <= self.opponent_close_distance:
            return None

        left_close = 0.0 < self.wall_left_distance <= self.wall_close_distance
        right_close = 0.0 < self.wall_right_distance <= self.wall_close_distance

        if not (left_close or right_close):
            return None
        if self.wall_left_distance > self.wall_right_distance:
            return 1.0
        if self.wall_right_distance > self.wall_left_distance:
            return -1.0
        return 1.0

    # 벽 회피 90도 회전을 시작한다.
    # 시작 시간과 방향을 저장하고 wall_turning=True로 만들어 control_loop가 회전을 유지하게 한다.
    def start_wall_turn(self, wall_direction):
        self.wall_turn_direction = wall_direction
        self.wall_turn_started_at = self.get_clock().now()
        self.wall_turning = True

        direction_text = 'left' if self.wall_turn_direction > 0 else 'right'
        self.get_logger().warn(
            f'Wall close. Turning {direction_text} 90 degrees. '
            f'L={self.wall_left_distance:.2f}m R={self.wall_right_distance:.2f}m'
        )

    # 시작 직후 동작.
    def run_opening_sequence(self):
        now = self.get_clock().now()

        if self.opening_state == "TURN_LEFT_45":
            elapsed = (now - self.opening_started_at).nanoseconds / 1e9
            if elapsed < self.turn_45_duration:
                self.publish_smoothed_command(0.0, self.opening_turn_direction * self.turn_45_speed)
                return True

            self.opening_state = "GO_STRAIGHT"
            self.opening_started_at = now
            self.publish_smoothed_command(self.normal_linear_vel, 0.0)
            return True

        if self.opening_state == "GO_STRAIGHT":
            if self.target_found or self.get_wall_avoid_direction() is not None:
                self.opening_state = "DONE"
                return False

            self.publish_smoothed_command(self.normal_linear_vel, 0.0)
            return True

        return False

    def control_loop(self):
        stale_time = (self.get_clock().now() - self.last_seen).nanoseconds / 1e9
        if stale_time > 0.5:
            self.reset_green_target()

        if self.run_opening_sequence():
            return

        if self.wall_turning:
            wall_turn_elapsed = (self.get_clock().now() - self.wall_turn_started_at).nanoseconds / 1e9
            if wall_turn_elapsed >= self.wall_turn_duration:
                self.wall_turning = False
                self.publish_smoothed_command(0.0, 0.0)
            else:
                self.publish_smoothed_command(0.0, self.wall_turn_direction * self.wall_turn_speed)
            return

        wall_direction = self.get_wall_avoid_direction()
        if wall_direction is not None:
            self.start_wall_turn(wall_direction)
            self.publish_smoothed_command(0.0, self.wall_turn_direction * self.wall_turn_speed)
            return

        if self.state == "START_TURN_RIGHT_45":
            elapsed = (self.get_clock().now() - self.state_started_at).nanoseconds / 1e9
            if elapsed < self.turn_45_duration:
                self.publish_smoothed_command(0.0, self.turn_45_direction * self.turn_45_speed)
            else:
                self.start_state("GO_STRAIGHT")
                self.publish_smoothed_command(0.6, 0.0)
            return

        if self.target_found:
            target_v, target_w = self.compute_green_follow_command()
            self.publish_smoothed_command(target_v, target_w)
            return

        self.publish_smoothed_command(0.6, 0.0)


    # 설정 및 초기화 담당
    def init_battle_params(self):
        # 벽 거리 기본값
        self.no_wall_distance = 9.9
        self.wall_close_distance = 0.20
        self.opponent_close_distance = 0.50
        self.wall_distance = self.no_wall_distance
        self.wall_left_distance = self.no_wall_distance
        self.wall_right_distance = self.no_wall_distance

        # 벽 회피 90도 회전에 필요한 상태값.
        self.wall_turning = False
        self.wall_turn_direction = 0.0
        self.wall_turn_speed = 0.8
        self.wall_turn_duration = (math.pi / 2.0) / self.wall_turn_speed
        self.wall_turn_started_at = self.get_clock().now()

        # 45도 회전 상태에 사용하는 값.
        self.turn_45_direction = 1.0
        self.turn_45_speed = 0.8
        self.turn_45_duration = (math.pi / 4.0) / self.turn_45_speed

        # 시작하자마자 실행되는 오프닝 동작 상태.
        self.opening_state = "TURN_LEFT_45"
        self.opening_started_at = self.get_clock().now()
        self.opening_turn_direction = 1.0

        # 뒤쪽 색깔 센서에서 all_blue가 오래 유지될 때 후진/토크 유지에 쓰는 상태.
        self.all_blue_started_at = None
        self.backing_until_forward_bule = False
        self.holding_torque_after_forward_bule = False
        self.all_sensor_blue_started_at = None
        self.backing_until_no_color = False
        self.reverse_linear_vel = -0.3

        # 앞쪽 색깔 센서 회피에 쓰는 값.
        self.front_green_distance_threshold = 0.40 
        self.front_escape_state = None
        self.front_escape_started_at = self.get_clock().now()
        self.front_turn_direction = self.turn_45_direction
        self.front_reverse_distance = 0.10
        self.front_reverse_linear_vel = -0.25
        self.wheel_radius = 0.04
        self.front_reverse_wheel_turns = self.front_reverse_distance / (2.0 * math.pi * self.wheel_radius)
        self.front_reverse_duration = self.front_reverse_distance / abs(self.front_reverse_linear_vel)

        # 빨간색이 초록색과 카메라 사이에 있을 때 우회하는 상태.
        self.red_between_green = False
        self.red_front_state = None
        self.red_front_started_at = self.get_clock().now()
        self.red_front_turn_direction = 1.0
        self.red_front_repeat_count = 0
        self.red_front_max_repeats = 3
        self.red_front_forward_linear_vel = 0.25
        self.red_front_forward_duration = self.front_reverse_distance / self.red_front_forward_linear_vel
        self.red_charge_mode = False
        self.green_angle_error_limit = 0.85

        # BattleMode 전체 상태
        self.state = "IDLE"
        self.state_started_at = self.get_clock().now()
        self.color_escape_state = None
        self.color_escape_started_at = self.get_clock().now()
        self.floor_color = 0
        self.far_opening_done = False

    # 거리/센서 토픽 값을 내부 변수에 저장하는 공통 콜백
    def update_sensor_value(self, msg, attr_name):
        setattr(self, attr_name, msg.data)

    def red_between_green_callback(self, msg):
        self.red_between_green = msg.data

    # 색깔 센서 경우
    # sensor1 : 로봇 기준 오른쪽 앞쪽 컬러센서
    # sensor2 : 로봇 기준 왼쪽 앞쪽 컬러센서
    # sensor3 : 로봇 뒤쪽 컬러센서
    # 인식 안될 때 ==0, 빨강일 때 ==1, 파랑일 때 ==2

    def start_state(self, state):
        self.state = state
        self.state_started_at = self.get_clock().now()

    def green_color_detected(self, sensor1, sensor2, sensor3):
        self.target_found = True

        if self.target_distance > 1.2 and sensor1 == 0 and sensor2 == 0 and sensor3 == 0:
            self.start_state("START_TURN_RIGHT_45")
            return 0.0, self.turn_45_direction * self.turn_45_speed
        
        elif self.target_distance < 1.2 and sensor1 == 0 and sensor2 == 0 and sensor3 == 0:
            return self.compute_green_follow_command()
        
        elif sensor1 != 0 or sensor2 != 0 and sensor3 == 0:
            return self.green_color_sensor_forward(sensor1, sensor2)

        elif sensor1 == 0 and sensor2 == 0 and sensor3 != 0:
            return self.green_color_sensor_back(sensor1, sensor2, sensor3)

        elif sensor1 != 0 and sensor2 != 0 and sensor3 != 0:
            return self.green_color_sensor_back(sensor1, sensor2, sensor3)
        return 0.0, 0.0
    


    def green_not_detected(self, sensor1, sensor2, sensor3):
        self.reset_green_target()

        if sensor1 == 0 and sensor2 == 0 and sensor3 == 0:
            return self.normal_forward_command()
        
        elif sensor1 != 0 or sensor2 != 0 and sensor3 == 0:
            return self.green_color_sensor_forward(sensor1, sensor2)

        elif sensor1 == 0 and sensor2 == 0 and sensor3 != 0:
            return self.green_color_sensor_back(sensor1, sensor2, sensor3)

        elif sensor1 != 0 and sensor2 != 0 and sensor3 != 0:
            return self.green_color_sensor_back(sensor1, sensor2, sensor3)
        return self.normal_forward_command()

    def start_red_front_escape(self, turn_direction, red_between_green):
        self.red_front_turn_direction = turn_direction
        self.red_front_repeat_count = 0
        self.red_charge_mode = False
        self.red_front_started_at = self.get_clock().now()
        if red_between_green:
            self.red_front_state = "RED_BACK_10CM"
            self.target_found = True
        else:
            self.red_front_state = "SIMPLE_BACK_10CM"
        return self.front_reverse_linear_vel, 0.0

    # 초록색이 40cm 이하로 가까우면 빨간색 3번 반복 후 돌진할때 색깔 센서를 무시하고 돌진
    def green_ready_to_charge(self):
        return self.camera_target_found and 0.0 < self.target_distance <= self.front_green_distance_threshold

    # target_error가 일정 범위 안이면 초록색이 대략 정면 방향에 있다고 본다. ********나중에 다시 한 번 검사**********
    def green_in_approach_angle(self):
        return self.camera_target_found and abs(self.target_error) <= self.green_angle_error_limit

    # 빨간색 전방 회피 
    def run_red_front_escape(self):
        now = self.get_clock().now()

        # red_charge_mode이면 초록색이 보이는 동안 강한 속도로 밀고 들어간다.
        if self.red_charge_mode:
            if self.camera_target_found:
                return self.boost_forward_command()
            self.red_charge_mode = False
            self.reset_green_target()
            return self.normal_forward_command()

        # 뒤로 약 10cm 물러난다.
        if self.red_front_state in ("RED_BACK_10CM", "SIMPLE_BACK_10CM"):
            elapsed = (now - self.red_front_started_at).nanoseconds / 1e9
            if elapsed < self.front_reverse_duration:
                if self.red_front_state == "RED_BACK_10CM":
                    self.target_found = True
                return self.front_reverse_linear_vel, 0.0

            if self.red_front_state == "RED_BACK_10CM":
                self.red_front_state = "RED_TURN_45"
            else:
                self.red_front_state = "SIMPLE_TURN_45"
            self.red_front_started_at = now
            return 0.0, self.red_front_turn_direction * self.turn_45_speed

        # 정해진 방향으로 약 45도 회전한다.
        if self.red_front_state in ("RED_TURN_45", "SIMPLE_TURN_45", "GREEN_APPROACH_TURN"):
            elapsed = (now - self.red_front_started_at).nanoseconds / 1e9
            if elapsed < self.turn_45_duration:
                if self.red_front_state == "RED_TURN_45":
                    self.target_found = True
                return 0.0, self.red_front_turn_direction * self.turn_45_speed

            if self.red_front_state == "SIMPLE_TURN_45":
                self.red_front_state = None
                return self.normal_forward_command()

            if self.red_front_state == "GREEN_APPROACH_TURN":
                self.red_front_state = "GREEN_APPROACH_FORWARD_10CM"
            else:
                self.red_front_state = "RED_FORWARD_10CM"
            self.red_front_started_at = now
            return self.red_front_forward_linear_vel, 0.0

        # 약 10cm 전진한다.
        # RED_FORWARD_10CM은 최대 3회 반복하고, 이후 초록색이 다시 보이는지 확인한다.
        if self.red_front_state in ("RED_FORWARD_10CM", "GREEN_APPROACH_FORWARD_10CM"):
            elapsed = (now - self.red_front_started_at).nanoseconds / 1e9
            if elapsed < self.red_front_forward_duration:
                if self.red_front_state == "RED_FORWARD_10CM":
                    self.target_found = True
                return self.red_front_forward_linear_vel, 0.0

            if self.red_front_state == "RED_FORWARD_10CM":
                self.red_front_repeat_count += 1
                if self.red_front_repeat_count < self.red_front_max_repeats:
                    self.red_front_state = "RED_TURN_45"
                    self.red_front_started_at = now
                    return 0.0, self.red_front_turn_direction * self.turn_45_speed

            self.red_front_state = "RED_CHECK_GREEN"

        # 마지막 단계: 초록색을 다시 확인한다.
        # 안 보이면 일반 주행으로 돌아가고, 가까우면 돌진 모드로 들어간다.
        if self.red_front_state == "RED_CHECK_GREEN":
            if not self.camera_target_found:
                self.reset_green_target()
                self.red_front_state = None
                return self.normal_forward_command()

            self.target_found = True
            if self.green_ready_to_charge():
                self.red_charge_mode = True
                self.red_front_state = None
                return self.boost_forward_command()

            if self.green_in_approach_angle():
                self.red_front_state = "GREEN_APPROACH_TURN"
                self.red_front_started_at = now
                return 0.0, self.red_front_turn_direction * self.turn_45_speed

            self.red_front_state = None
            return self.compute_green_follow_command()

        return self.normal_forward_command()

    def green_color_sensor_forward(self, sensor1, sensor2):
        # 빨간색 회피가 이미 진행 중이면 새 판단을 하지 않고 계속 진행
        if self.red_front_state is not None or self.red_charge_mode:
            return self.run_red_front_escape()

        if self.front_escape_state == "BACK_10CM":
            elapsed = (self.get_clock().now() - self.front_escape_started_at).nanoseconds / 1e9
            if elapsed < self.front_reverse_duration:
                return self.front_reverse_linear_vel, 0.0

            self.front_escape_state = "TURN_45"
            self.front_escape_started_at = self.get_clock().now()
            return 0.0, self.front_turn_direction * self.turn_45_speed

        if self.front_escape_state == "TURN_45":
            elapsed = (self.get_clock().now() - self.front_escape_started_at).nanoseconds / 1e9
            if elapsed < self.turn_45_duration:
                return 0.0, self.front_turn_direction * self.turn_45_speed

            self.front_escape_state = None
            return self.compute_green_follow_command()

        # 오른쪽 앞 센서가 빨강이면 반대쪽으로 피하기 위해 left 방향 회피
        if sensor1 == 1 and sensor2 == 0:
            return self.start_red_front_escape(1.0, self.red_between_green)

        # 왼쪽 앞 센서가 빨강이면 반대쪽으로 피하기 위해 right 방향 회피
        elif sensor1 == 0 and sensor2 == 1:
            return self.start_red_front_escape(-1.0, self.red_between_green)

        # 양쪽 앞 센서가 빨강이면 특별 회피 없이 일단 직진한다.+++++++++++++++++++
        elif sensor1 == 1 and sensor2 == 1:
            return self.normal_forward_command()

        # 오른쪽 앞 센서가 파랑이면 뒤로 물러난 뒤 45도 회전하는 회피
        elif sensor1 == 2 and sensor2 == 0:
            self.front_turn_direction = 1.0
            self.front_escape_state = "BACK_10CM"
            self.front_escape_started_at = self.get_clock().now()
            return self.front_reverse_linear_vel, 0.0

        # 왼쪽 앞 센서가 파랑인 경우도 뒤로 물러난 뒤 반대 방향 회피
        elif sensor1 == 0 and sensor2 == 2:
            self.front_turn_direction = -1.0
            self.front_escape_state = "BACK_10CM"
            self.front_escape_started_at = self.get_clock().now()
            return self.front_reverse_linear_vel, 0.0

        # 양쪽 앞 센서가 파랑이면 초록색이 가까우면 추격, 멀면 회피
        elif sensor1 == 2 and sensor2 == 2:
            self.front_escape_state = None
            if self.target_found and self.target_distance >= self.front_green_distance_threshold:
                return self.compute_green_follow_command()

            return 0.0, 0.0

        self.front_escape_state = None
        return self.normal_forward_command()

    # 검은색 상태에서 앞쪽 센서를 어떻게 처리할지 적어둘 자리.
    # 아직 실제 동작은 pass로 남아 있다.
    def black_color_sensor_forward(self, sensor1, sensor2):
        if sensor1 == 1 and sensor2 == 0:
            pass
        elif sensor1 == 0 and sensor2 == 1:
            pass
        elif sensor1 == 2 and sensor2 == 0:
            pass
        elif sensor1 == 0 and sensor2 == 2:
            pass

    # 뒤쪽 색깔 센서(sensor3)까지 포함한 처리.
    # 뒤쪽이 파랑/빨강을 밟는 상황에서 밀기, 후진, 토크 유지 같은 동작을 결정한다.
    def green_color_sensor_back(self, sensor1, sensor2, sensor3):
        # forward_bule(110) 상태에 도달한 뒤에는 0,0 명령으로 토크를 걸듯이 버틴다.
        # 그 상태가 깨지면 holding 상태를 해제한다.
        if self.holding_torque_after_forward_bule:
            if self.forward_bule(sensor1, sensor2, sensor3):
                return 0.0, 0.0
            self.holding_torque_after_forward_bule = False

        # 모든 센서가 파랑(222)인 상태가 5초 이상 유지된 뒤에는
        # 아무 색도 안 보이는 000 상태가 될 때까지 후진한다.
        if self.backing_until_no_color:
            if sensor1 == 0 and sensor2 == 0 and sensor3 == 0:
                self.backing_until_no_color = False
                self.all_sensor_blue_started_at = None
                return self.normal_forward_command()
            return self.reverse_linear_vel, 0.0

        # all_blue(111) 상태가 5초 이상 지속되면 후진을 시작한다.
        # 후진하다가 forward_bule(110) 상태가 되면 후진을 멈추고 토크 유지 상태로 들어간다.
        if self.backing_until_forward_bule:
            if self.forward_bule(sensor1, sensor2, sensor3):
                self.backing_until_forward_bule = False
                self.holding_torque_after_forward_bule = True
                self.all_blue_started_at = None
                return 0.0, 0.0
            return self.reverse_linear_vel, 0.0

        # 뒤쪽 센서만 1 또는 2일 때는 상대를 밀기 위해 boost_forward_command()를 사용한다.
        if sensor1 == 0 and sensor2 == 0 and sensor3 == 1:
            self.all_blue_started_at = None
            self.all_sensor_blue_started_at = None
            return self.boost_forward_command()

        elif sensor1 == 0 and sensor2 == 0 and sensor3 == 2:
            self.all_blue_started_at = None
            self.all_sensor_blue_started_at = None
            return self.boost_forward_command()
        
        # all_blue(111) 상태가 처음 잡히면 시간을 재기 시작한다.
        # 5초 미만이면 계속 boost, 5초 이상이면 forward_bule가 나올 때까지 후진한다.
        elif self.all_blue(sensor1, sensor2, sensor3):
            self.all_sensor_blue_started_at = None
            now = self.get_clock().now()
            if self.all_blue_started_at is None:
                self.all_blue_started_at = now

            elapsed = (now - self.all_blue_started_at).nanoseconds / 1e9

            if elapsed >= 5.0:
                self.backing_until_forward_bule = True
                return self.reverse_linear_vel, 0.0

            return self.boost_forward_command()
        
        # 모든 센서가 파랑(222)이면 처음 5초 동안은 강하게 직진한다.
        # 5초 이상 계속 222이면 000이 될 때까지 후진한다.
        elif sensor1 ==2 and sensor2 == 2 and sensor3 == 2:
            self.all_blue_started_at = None
            now = self.get_clock().now()
            if self.all_sensor_blue_started_at is None:
                self.all_sensor_blue_started_at = now

            elapsed = (now - self.all_sensor_blue_started_at).nanoseconds / 1e9
            if elapsed >= 5.0:
                self.backing_until_no_color = True
                return self.reverse_linear_vel, 0.0

            return self.boost_forward_command()
        
        # 그 외에는 all_blue 타이머를 초기화하고 일반 직진으로 돌아간다.
        self.all_blue_started_at = None
        self.all_sensor_blue_started_at = None
        return self.normal_forward_command()

    # 사용자 요청에서 all_blue라고 부르기로 한 111 상태를 확인하는 helper.
    def all_blue(self, sensor1, sensor2, sensor3):
        return sensor1 == 1 and sensor2 == 1 and sensor3 == 1

    # 사용자 요청에서 forward_bule라고 부르기로 한 110 상태.
    # 사용자 요청에서 forward_bule라는 이름으로 지정한 상태라 철자를 그대로 유지한다.
    def forward_bule(self, sensor1, sensor2, sensor3):
        return sensor1 == 1 and sensor2 == 1 and sensor3 == 0

    def black_color_sensor_back(self, sensor1, sensor2, sensor3):
        if sensor1 == 0 and sensor2 == 0 and sensor3 == 1:
            return self.boost_forward_command()
        elif sensor1 == 0 and sensor2 == 0 and sensor3 == 2:
            return self.boost_forward_command() 
        elif sensor1 == 1 and sensor2 == 1 and sensor3 == 1:
            return self.boost_forward_command()
        elif sensor1 ==2 and sensor2 == 2 and sensor3 == 2:
            return self.boost_forward_command()
            # 5초 이상 일시 후진 기능 추가 
        return self.normal_forward_command()


def main(args=None):
    rclpy.init(args=args)

    node = BattleMode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
