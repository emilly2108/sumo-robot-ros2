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
순서로 판단합니다. 실제 동작 우선순위와 센서 조합은 아래 표로 정리했습니다.

현재 기본 발행 주기는 Brain `/cmd_vel` 20Hz, 비전 처리·발행 10Hz, 컬러 센서 10Hz,
모터 CAN 전송 20Hz입니다. 모터가 `command-ready`가 아니면 주기를 낮춰도 명령은
거부되므로, 그 경우에는 모터 노드의 CANopen 상태와 fault를 별도로 확인해야 합니다.

## 센서 위치와 값

`color_node.py`의 TCS34725 채널은 다음 위치로 사용합니다.

| 채널 | 위치 | brain 내부 센서 |
|---|---|---|
| CH0 | 앞 왼쪽 | sensor2 |
| CH1 | 앞 오른쪽 | sensor1 |
| CH2 | 뒤쪽 | sensor3 |

컬러 값은 `0=BLACK`, `1=RED`, `2=BLUE`입니다. 현재 `color_node.py`는 세 센서 값을
`/color_sensor` 토픽 한 메시지에 묶어 보냅니다.

```json
{"sensors":{"front_left":1,"front_right":1,"back":0}}
```

따라서 앞 양쪽이 빨강이면 Brain은 메시지 하나만 받아도 즉시
`SENSOR_EVENT=FRONT_BOTH_RED values=(1, 1, 0)`으로 갱신합니다. 이전의
`{"position":"front_left","color":1}` 단일 메시지 형식도 Brain에서 계속 지원합니다.

## 색상 센서 패턴별 행동표

패턴 순서는 항상 `(sensor1, sensor2, sensor3)`, 즉 `(오른쪽 앞, 왼쪽 앞, 뒤)`입니다.
아래 표는 스위치가 ON이고 벽이 가까이 있지 않은 상태를 기준으로 합니다.

| 센서 조합 | 색깔 상태 | 초록색 없음·벽 없음 | 초록색 인식 | 동작 |
|---|---|---|---|---|
| `0, 0, 0` | 앞·뒤 모두 검정 | 기본 속도로 직진 | 초록색 추격 | `Cruise_Action` |
| `1, 0, 0` | 앞 오른쪽 빨강만 | 원래 경로로 계속 주행 | 초록색 추격 | 한쪽만 감지했으므로 회피하지 않음 |
| `0, 1, 0` | 앞 왼쪽 빨강만 | 원래 경로로 계속 주행 | 초록색 추격 | 한쪽만 감지했으므로 회피하지 않음 |
| `2, 0, 0` | 앞 오른쪽 파랑만 | 원래 경로로 계속 주행 | 초록색 추격 | 한쪽만 감지했으므로 회피하지 않음 |
| `0, 2, 0` | 앞 왼쪽 파랑만 | 원래 경로로 계속 주행 | 초록색 추격 | 한쪽만 감지했으므로 회피하지 않음 |
| `1, 1, 0` | 앞 양쪽 빨강 | 후진 후 45도 회전 | 초록색 추격으로 전환 | `Front_Color_Avoid_Action` |
| `2, 2, 0` | 앞 양쪽 파랑 | 후진 후 45도 회전 | 초록색 추격으로 전환 | `Front_Color_Avoid_Action` |
| `0, 0, 1` | 뒤쪽 빨강만 | 최대 또는 추격 보조 속도 | 초록색 추격 | `Back_Single_Boost_Action` |
| `0, 0, 2` | 뒤쪽 파랑만 | 최대 또는 추격 보조 속도 | 초록색 추격 | `Back_Single_Boost_Action` |
| `1, 1, 1` | 모든 센서 빨강 | 빨강 복구 행동 | 초록색 추격으로 전환 | `All_Red_Recovery_Action` (미검증) |
| `2, 2, 2` | 모든 센서 파랑 | 최대 속도 유지 후 5초 이상이면 후진 | 초록색 추격으로 전환 | `All_Blue_Recovery_Action` (미검증) |

## 초록색이 보일 때 색상 센서 행동표

초록색이 보이면 아래 표를 사용합니다. 거리는 `vision_node.py`가 보내는 미터 단위이며,
`0.05m=50mm`입니다. 표에 없는 혼합 색상은 초록색 추격을 기본값으로 사용합니다.
벽이 가까우면 아래 표보다 `Wall_Avoid_Action`이 먼저 실행됩니다.

| 센서 조합 `(sensor1, sensor2, sensor3)` | `거리 <= 50mm` | `50mm < 거리 < 1.2m` | `거리 >= 1.2m` | 선택 행동 |
|---|---|---|---|---|
| `0, 0, 0` | 기존 초록색 추격 | 기존 초록색 추격 | 기존 초록색 추격 | `Green_Follow_Action` |
| `1, 0, 0` 또는 `0, 1, 0` | 원래 경로로 초록색 추격 | 원래 경로로 초록색 추격 | 45도 회전 후 초록색 추격 | `Far_Green_Turn_Action` 또는 `Green_Follow_Action` |
| `2, 0, 0` 또는 `0, 2, 0` | 원래 경로로 초록색 추격 | 원래 경로로 초록색 추격 | 원래 경로로 초록색 추격 | `Green_Follow_Action` |
| `1, 1, 0` | 앞 양쪽이 검정이 될 때까지 후진 후 초록색 중심 추격 | 기존처럼 후진 후 나중 감지 방향으로 45도 회전 | 45도 회전 후 초록색 추격 | `Green_Close_Front_Color_Action`, `Front_Color_Avoid_Action` 또는 `Far_Green_Turn_Action` |
| `2, 2, 0` | 앞 양쪽이 검정이 될 때까지 후진 후 초록색 중심 추격 | 기존처럼 후진 후 나중 감지 방향으로 45도 회전 | 기존 회피 행동 | `Green_Close_Front_Color_Action` 또는 `Front_Color_Avoid_Action` |
| `0, 0, 1` | 최대 속도로 초록색 추격 | 초록색 추격 | 초록색 추격 | `Green_Sensor_Follow_Action` |
| `0, 0, 2` | 앞 양쪽이 모두 색이면 검정이 될 때까지 후진, 아니면 최대 속도 추격 | 초록색 추격 | 초록색 추격 | `Green_Close_Back_Blue_Action` 또는 `Green_Sensor_Follow_Action` |
| `1, 1, 1` | 전체 빨강 상태가 풀릴 때까지 후진 | 초록색 추격 | 초록색 추격 | `Green_Close_All_Red_Action` 또는 `Green_Follow_Action` |
| `2, 2, 2` | 최대 속도로 초록색 추격 | 초록색 추격 | 초록색 추격 | `Green_Sensor_Follow_Action` |

앞 센서 한쪽만 빨강 또는 파랑인 경우에는 색상 회피를 시작하지 않고 초록색 중심을
따라갑니다. 뒤 파랑 상태에서 앞 양쪽에 색이 동시에 나타나면 정확한 패턴표에 없는
혼합 상태라도 `Green_Close_Back_Blue_Action`이 우선하여 앞이 검정이 될 때까지 후진합니다.
초록색 추격 명령은 비전 노드가 계산한 중심 오차를 사용하므로 캘리브레이션된 중심 기준은
기존과 같습니다.

벽이 가까우면 위 표보다 `Wall_Avoid_Action`이 먼저 실행됩니다. 표에 없는 서로 다른
색의 혼합 조합은 현재 전용 색상 행동이 등록되어 있지 않습니다.

검증 상태: `(1, 1, 1)`과 `(2, 2, 2)`를 제외한 색상 센서 조합은 확인되었습니다.
두 전체 센서 복구 행동은 현재 확인 환경이 준비되지 않아 아직 실환경에서 검증하지
않았으며, 표의 내용은 구현 예정 동작을 설명한 것입니다.

## 주요 파일

- `hfsm_brain/node.py`: 실제 HFSM brain ROS 2 노드
- `hfsm_brain/models.py`: World Model, 설정, 명령 데이터 구조
- `hfsm_brain/actions/`: 오프닝, 초록색 추격·센서 분기, 벽 회피, 센서 행동
- `vision_node.py`: D435i 초록색 목표와 벽 거리 인식
- `color_node.py`: TCS34725 컬러 센서 입력 및 색상 분류
- `motor_node.py`: `/cmd_vel`을 CANopen 모터 명령으로 변환
- `switch_node.py`: GPIO 물리 스위치와 `switch_mode` 서비스 연결
- `PROJECT_STATUS.html`: 파일별 역할, 토픽 흐름, 색상 패턴별 동작 정리

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

앞 양쪽에 빨강 판을 놓았을 때 `/color_sensor`에는 위 JSON의 `front_left=1`,
`front_right=1`이 함께 보여야 하며, Brain 터미널에는
`SENSOR_EVENT=FRONT_BOTH_RED`와 행동 전환 로그가 표시되어야 합니다.

실행 대상은 현재 채택한 `hfsm_brain` 모듈이며, 모터를 연결하지 않은 상태에서 먼저
`/cmd_vel`을 확인해야 합니다.

## 주의

- `motor_node.py`는 실제 모터와 CAN 버스에 명령을 보낼 수 있습니다.
- 처음 시험할 때는 바퀴를 지면에서 띄우고 `/cmd_vel` 출력부터 확인합니다.
- 카메라와 센서가 보내는 토픽 이름 및 단위가 brain 노드의 기대값과 일치해야 합니다.
- 속도와 거리 기준은 대회장과 하드웨어에 맞춰 별도 검증해야 합니다.


## 현재 구현 상황

- 초록색이 안보일 때


| 센서 조합 | 색깔 상태 | 초록색 없음·벽 없음 | 초록색 인식 | 동작 |
|---|---|---|---|---|
| `0, 0, 0` | 앞·뒤 모두 검정 | 기본 속도로 직진 | 초록색 추격 | `Cruise_Action` |
| `1, 0, 0` | 앞 오른쪽 빨강만 | 원래 경로로 계속 주행 | 초록색 추격 | 한쪽만 감지했으므로 회피하지 않음 |
| `0, 1, 0` | 앞 왼쪽 빨강만 | 원래 경로로 계속 주행 | 초록색 추격 | 한쪽만 감지했으므로 회피하지 않음 |
| `2, 0, 0` | 앞 오른쪽 파랑만 | 원래 경로로 계속 주행 | 초록색 추격 | 한쪽만 감지했으므로 회피하지 않음 |
| `0, 2, 0` | 앞 왼쪽 파랑만 | 원래 경로로 계속 주행 | 초록색 추격 | 한쪽만 감지했으므로 회피하지 않음 |
| `1, 1, 0` | 앞 양쪽 빨강 | 후진 후 45도 회전 | 초록색 추격으로 전환 | `Front_Color_Avoid_Action` |
| `2, 2, 0` | 앞 양쪽 파랑 | 후진 후 45도 회전 | 초록색 추격으로 전환 | `Front_Color_Avoid_Action` |
| `0, 0, 1` | 뒤쪽 빨강만 | 최대 또는 추격 보조 속도 | 초록색 추격 | `Back_Single_Boost_Action` |
| `0, 0, 2` | 뒤쪽 파랑만 | 최대 또는 추격 보조 속도 | 초록색 추격 | `Back_Single_Boost_Action` |
| `1, 1, 1` | 모든 센서 빨강 | 빨강 복구 행동 | 초록색 추격으로 전환 | `All_Red_Recovery_Action` (미검증 - 코드는 구현되었으나 실험 불가) |
| `2, 2, 2` | 모든 센서 파랑 | 최대 속도 유지 후 5초 이상이면 후진 | 초록색 추격으로 전환 | `All_Blue_Recovery_Action` (미검증 - 코드는 구현되었으나 실험 불가) |

-초록색이 보일 때

| 센서 조합 | 색깔 상태 | 초록색 없음·벽 없음 | 초록색 인식 | 동작 |
|---|---|---|---|---|
| `0, 0, 0` | 앞·뒤 모두 검정 | 기본 속도로 직진 | 초록색 추격 | `Cruise_Action` |
| `1, 0, 0` | 앞 오른쪽 빨강만 | 원래 경로로 계속 주행 | 초록색 추격 | 한쪽만 감지했으므로 회피하지 않음 (미검증 - 코드는 구현되었으나 실험 불가)|
| `0, 1, 0` | 앞 왼쪽 빨강만 | 원래 경로로 계속 주행 | 초록색 추격 | 한쪽만 감지했으므로 회피하지 않음 (미검증 - 코드는 구현되었으나 실험 불가)|
| `2, 0, 0` | 앞 오른쪽 파랑만 | 원래 경로로 계속 주행 | 초록색 추격 | 한쪽만 감지했으므로 회피하지 않음(미검증 - 코드는 구현되었으나 실험 불가) |
| `0, 2, 0` | 앞 왼쪽 파랑만 | 원래 경로로 계속 주행 | 초록색 추격 | 한쪽만 감지했으므로 회피하지 않음 (미검증 - 코드는 구현되었으나 실험 불가)|
| `1, 1, 0` | 앞 양쪽 빨강 | 후진 후 45도 회전 | 초록색 추격으로 전환 | `Front_Color_Avoid_Action`(미검증 - 코드는 구현되었으나 실험 불가) |
| `2, 2, 0` | 앞 양쪽 파랑 | 후진 후 45도 회전 | 초록색 추격으로 전환 | `Front_Color_Avoid_Action`(미검증 - 코드는 구현되었으나 실험 불가) |
| `0, 0, 1` | 뒤쪽 빨강만 | 최대 또는 추격 보조 속도 | 초록색 추격 | `Back_Single_Boost_Action` (미검증 - 코드는 구현되었으나 실험 불가)|
| `0, 0, 2` | 뒤쪽 파랑만 | 최대 또는 추격 보조 속도 | 초록색 추격 | `Back_Single_Boost_Action` (미검증 - 코드는 구현되었으나 실험 불가)|
| `1, 1, 1` | 모든 센서 빨강 | 빨강 복구 행동 | 초록색 추격으로 전환 | `All_Red_Recovery_Action` (미검증 - 코드는 구현되었으나 실험 불가) |
| `2, 2, 2` | 모든 센서 파랑 | 최대 속도 유지 후 5초 이상이면 후진 | 초록색 추격으로 전환 | `All_Blue_Recovery_Action` (미검증 - 코드는 구현되었으나 실험 불가) |

현재 발생한 문제

- 버퍼링 걸려서 로봇이 바로바로 행동을 못함 <= 토픽쏘는 속도를 낮춰 버퍼링을 줄임 ( 해결 8.29)
- 시작 후 45도 회전과 빨간색과 파란색 영역에서 의 45도 회전이 잘 행동이 안 됌
- can 통신이 자꾸 오류가 나는 중
