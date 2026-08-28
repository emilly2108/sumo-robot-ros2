# 씨름 로봇 ROS2 제어 코드

라즈베리 파이 5에서 실행하는 씨름 로봇용 ROS 2 Python 코드 모음입니다.
Intel RealSense D435i 비전 노드, TCS34725 컬러 센서 노드, CANopen 모터 노드,
물리 스위치 노드와 HFSM 기반 brain 노드를 포함합니다.

## 실행 구조

```text
vision_node.py  -> /target_error, /target_distance, /target_found
                -> /wall_distance, /wall_left_distance, /wall_right_distance
color_node.py   -> /color_sensor
switch_node.py  -> switch_mode 서비스 요청
hfsm_brain      -> /cmd_vel
motor_node.py   -> CANopen 모터 4개
```

HFSM brain은 스위치가 켜지기 전에는 정지 명령을 유지합니다. 스위치가 켜진 뒤에는
벽 회피를 먼저 판단하고, 벽이 없으면 초록색 추격, 컬러 센서 패턴 행동, 기본 주행
순서로 판단합니다. 실제 동작 우선순위와 센서 조합은 `PROJECT_STATUS.html`에서
표로 정리했습니다.

## 센서 위치와 값

`color_node.py`의 TCS34725 채널은 다음 위치로 사용합니다.

| 채널 | 위치 | brain 내부 센서 |
|---|---|---|
| CH1 | 앞 오른쪽 | sensor1 |
| CH3 | 앞 왼쪽 | sensor2 |
| CH2 | 뒤쪽 | sensor3 |

컬러 값은 `0=BLACK`, `1=RED`, `2=BLUE`입니다. 컬러 센서 결과는 `/color_sensor`
토픽으로 JSON 문자열을 보냅니다.

## 주요 파일

- `hfsm_brain/node.py`: 실제 HFSM brain ROS 2 노드
- `hfsm_brain/models.py`: World Model, 설정, 명령 데이터 구조
- `hfsm_brain/actions/`: 오프닝, 초록색 추격, 벽 회피, 센서 행동
- `vision_node.py`: D435i 초록색 목표와 벽 거리 인식
- `color_node.py`: TCS34725 컬러 센서 입력 및 색상 분류
- `motor_node.py`: `/cmd_vel`을 CANopen 모터 명령으로 변환
- `switch_node.py`: GPIO 물리 스위치와 `switch_mode` 서비스 연결
- `PROJECT_STATUS.html`: 파일별 역할, 토픽 흐름, 색상 패턴별 동작 정리
- `brain_test_*.py`, `test_*.py`: 기능별 테스트 또는 이전 구현 참고 코드
- `calibrate_center.py`, `center_calibration.py`, `center_calibration.json`: 카메라 중심 보정

## 준비 환경

- Raspberry Pi 5
- ROS 2 Jazzy
- Intel RealSense SDK와 D435i
- `rclpy`, `geometry_msgs`, `std_msgs`, `std_srvs`
- `numpy`, `opencv-python`, `pyrealsense2`
- 컬러 센서와 모터에 필요한 GPIO, SMBus, CANopen Python 의존성

각 장치의 실제 포트와 라이브러리 설치 상태는 사용하는 라즈베리 파이 환경에 맞춰
확인해야 합니다. 이 저장소는 Python 패키지 설치 파일을 포함하지 않으므로 ROS 2를
먼저 source한 뒤 실행합니다.

## 실행 예시

각 노드는 별도 터미널에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/src/sumo_robot/sumo_robot

python3 vision_node.py
python3 color_node.py
python3 motor_node.py
python3 -m hfsm_brain.node
python3 switch_node.py
```

실행 전 카메라, I2C 컬러 센서, CAN 인터페이스를 연결하고 모터를 들어 올린
상태에서 토픽을 확인하는 것을 권장합니다.

```bash
ros2 topic echo /target_found
ros2 topic echo /target_distance
ros2 topic echo /color_sensor
ros2 topic echo /cmd_vel
```

`test_code.py`, `test_code_hfsm.py` 등은 실제 모터를 직접 대신하는 통합 실행 파일이
아니라 기능 확인과 이전 구조 비교를 위한 파일입니다. 실행 대상은 현재 채택한
`hfsm_brain` 모듈이며, 모터를 연결하지 않은 상태에서 먼저 `/cmd_vel`을 확인해야
합니다.

## 주의

- `motor_node.py`는 실제 모터와 CAN 버스에 명령을 보낼 수 있습니다.
- 처음 시험할 때는 바퀴를 지면에서 띄우고 `/cmd_vel` 출력부터 확인합니다.
- 카메라와 센서가 보내는 토픽 이름 및 단위가 brain 노드의 기대값과 일치해야 합니다.
- 속도와 거리 기준은 대회장과 하드웨어에 맞춰 별도 검증해야 합니다.
