#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SteadyWin 드라이버 4개를 CANopen으로 제어하는 ROS 2 모터 노드.

로봇이 언제 움직일지는 브레인 노드가 판단하고 /cmd_vel을 발행한다. 이 노드는
그 명령을 모터별 목표 속도로 변환하고, CANopen 설정 응답 확인, fault 고정,
hard stop, 정상 종료 시 토크 해제를 담당한다.

주의 사항
---------
* 이 코드에서는 RPDO3 매핑을 바꾸지 않는다. EDS/오브젝트 딕셔너리로 실제
  드라이버 매핑을 확인하기 전에는 0x1602 매핑을 수정하면 안 된다.
* Twist 속도 0은 토크를 유지하는 일반 정지이며, 토크 해제가 아니다.
* SIGKILL이나 전원 차단 시에는 종료 절차가 실행될 수 없다. 물리 E-stop과
  watchdog 같은 하드웨어 안전장치는 별도로 필요하다.
"""

from __future__ import annotations

import json
import math
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional, Tuple

import can
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


SDO_ABORT_CODES = {
    0x05040000: "SDO protocol timed out",
    0x06010000: "Unsupported access",
    0x06010001: "Read access to write-only object",
    0x06010002: "Write access to read-only object",
    0x06020000: "Object does not exist",
    0x06090011: "Sub-index does not exist",
    0x06090030: "Value range exceeded",
    0x08000000: "General error",
}


class CanopenError(RuntimeError):
    """CANopen 요청 거부, 시간 초과, 비정상 응답을 나타내는 예외."""


def cia402_state(statusword: int) -> str:
    """상태word 0x6041의 비트를 사람이 읽을 수 있는 CiA-402 상태로 변환한다."""
    checks = (
        (0x004F, 0x0000, "NOT_READY_TO_SWITCH_ON"),
        (0x004F, 0x0040, "SWITCH_ON_DISABLED"),
        (0x006F, 0x0021, "READY_TO_SWITCH_ON"),
        (0x006F, 0x0023, "SWITCHED_ON"),
        (0x006F, 0x0027, "OPERATION_ENABLED"),
        (0x006F, 0x0007, "QUICK_STOP_ACTIVE"),
        (0x004F, 0x000F, "FAULT_REACTION_ACTIVE"),
        (0x004F, 0x0008, "FAULT"),
    )
    for mask, expected, name in checks:
        if statusword & mask == expected:
            return name
    return "UNKNOWN"


@dataclass
class DriveStatus:
    """CANopen 드라이버 1개에서 마지막으로 확인한 상태를 저장한다."""

    heartbeat_state: Optional[int] = None
    heartbeat_time: Optional[float] = None
    emcy_code: Optional[int] = None
    emcy_register: Optional[int] = None
    statusword: Optional[int] = None
    cia402: str = "UNKNOWN"
    mode_display: Optional[int] = None
    actual_velocity: Optional[int] = None
    actual_velocity_time: Optional[float] = None
    poll_failures: int = 0


class _CanListener(can.Listener):
    """python-can 수신 스레드의 프레임을 모터 노드로 전달하는 수신기."""

    def __init__(self, owner: "SafeCanopenMotorNode") -> None:
        self.owner = owner

    def on_message_received(self, msg: can.Message) -> None:
        """수신된 CAN 프레임을 실제 처리 함수로 넘긴다."""
        self.owner._on_can_message(msg)


class SafeCanopenMotorNode(Node):
    """응답 확인, fault 처리, hard stop을 포함한 4모터 CSV 제어 노드."""

    def __init__(self) -> None:
        super().__init__("motor_node")

        # 차체 기하값과 엔코더 변환값이다. 0x60FF의 실제 단위는 실기에서
        # 확인해야 하므로 코드 상단 파라미터로 모아 둔다.
        self.declare_parameter("wheel_base_m", 0.20)
        self.declare_parameter("wheel_radius_m", 0.04)
        self.declare_parameter("encoder_counts_per_motor_rev", 16384)
        self.declare_parameter("gear_ratio", 8.0)
        self.declare_parameter("node_ids", [1, 2, 3, 4])
        self.declare_parameter("left_node_ids", [1, 3])
        self.declare_parameter("right_node_ids", [2, 4])

        # CAN 인터페이스 연결, SDO 응답 대기, 상태 조회 주기 관련 파라미터.
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("can_interface", "socketcan")
        self.declare_parameter("can_period_sec", 0.02)
        self.declare_parameter("sdo_timeout_sec", 0.25)
        self.declare_parameter("sdo_retries", 3)
        self.declare_parameter("status_poll_period_sec", 0.50)
        self.declare_parameter("require_heartbeat", False)
        self.declare_parameter("heartbeat_timeout_sec", 0.50)
        self.declare_parameter("max_consecutive_can_tx_failures", 3)

        # 강제 정지는 Halt 비트를 유지하는 잠금 상태다. 일반 속도 0 명령은
        # 즉시 재출발할 수 있어야 하므로 강제 정지와 별개로 처리한다.
        self.declare_parameter("hard_stop_controlword", 0x010F)
        self.declare_parameter("hard_stop_monitor_period_sec", 0.05)
        self.declare_parameter("hard_stop_velocity_threshold_raw", 1000)
        self.declare_parameter("hard_stop_timeout_sec", 2.0)

        # 소프트웨어 속도/가감속 제한은 적용하지 않는다. /cmd_vel은 바로 각
        # 바퀴 목표로 변환한다. 아래 명령 시간 초과, fault, 강제 정지는 속도
        # 제한이 아니라 통신/안전 상태 처리다.
        self.declare_parameter("command_timeout_sec", 0.20)

        self.wheel_base_m = float(self.get_parameter("wheel_base_m").value)
        self.wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        encoder_counts = float(self.get_parameter("encoder_counts_per_motor_rev").value)
        gear_ratio = float(self.get_parameter("gear_ratio").value)
        self.velocity_scale = encoder_counts * gear_ratio / (2.0 * math.pi * self.wheel_radius_m)
        self.node_ids = tuple(int(node_id) for node_id in self.get_parameter("node_ids").value)
        self.left_node_ids = tuple(
            int(node_id) for node_id in self.get_parameter("left_node_ids").value
        )
        self.right_node_ids = tuple(
            int(node_id) for node_id in self.get_parameter("right_node_ids").value
        )
        if not self.node_ids or any(not 1 <= node_id <= 127 for node_id in self.node_ids):
            raise ValueError("node_ids must contain CANopen IDs in the range 1..127")
        if (
            not self.left_node_ids
            or not self.right_node_ids
            or set(self.left_node_ids) & set(self.right_node_ids)
            or set(self.left_node_ids) | set(self.right_node_ids) != set(self.node_ids)
        ):
            raise ValueError(
                "left_node_ids and right_node_ids must be non-empty, disjoint, "
                "and cover node_ids exactly"
            )

        self.rpdo3_ids = {node_id: 0x400 + node_id for node_id in self.node_ids}
        self.can_period_sec = float(self.get_parameter("can_period_sec").value)
        self.sdo_timeout_sec = float(self.get_parameter("sdo_timeout_sec").value)
        self.sdo_retries = int(self.get_parameter("sdo_retries").value)
        self.status_poll_period_sec = float(self.get_parameter("status_poll_period_sec").value)
        self.require_heartbeat = bool(self.get_parameter("require_heartbeat").value)
        self.heartbeat_timeout_sec = float(self.get_parameter("heartbeat_timeout_sec").value)
        self.max_consecutive_can_tx_failures = max(
            1, int(self.get_parameter("max_consecutive_can_tx_failures").value)
        )
        self.hard_stop_controlword = int(
            self.get_parameter("hard_stop_controlword").value
        )
        self.hard_stop_monitor_period_sec = float(
            self.get_parameter("hard_stop_monitor_period_sec").value
        )
        self.hard_stop_velocity_threshold_raw = abs(
            int(self.get_parameter("hard_stop_velocity_threshold_raw").value)
        )
        self.hard_stop_timeout_sec = float(
            self.get_parameter("hard_stop_timeout_sec").value
        )
        self.command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)

        # ROS 콜백, CAN 수신 스레드, 상태 조회 스레드가 함께 접근하는 상태다.
        self._lock = threading.RLock()
        self._sdo_lock = threading.Lock()
        self._sdo_cv = threading.Condition(self._lock)
        self._sdo_replies: Dict[Tuple[int, int, int], bytes] = {}
        self._events: Deque[Tuple[str, str]] = deque(maxlen=100)
        self.drive_status = {node_id: DriveStatus() for node_id in self.node_ids}

        self.state = "INIT"
        self.fault_latched = False
        self.fault_reason = ""
        self.accept_commands = True
        self.desired_linear_mps = 0.0
        self.desired_angular_radps = 0.0
        self.last_cmd_monotonic = time.monotonic()
        self.latest_targets = {node_id: 0 for node_id in self.node_ids}
        self.consecutive_can_tx_failures = 0
        self.hard_stop_active = False
        self.hard_stop_complete = False
        self.hard_stop_started_monotonic: Optional[float] = None

        channel = str(self.get_parameter("can_channel").value)
        interface = str(self.get_parameter("can_interface").value)
        try:
            self.bus = can.interface.Bus(channel=channel, interface=interface)
        except Exception as exc:
            self.get_logger().fatal(f"CAN bus connection failed: {exc}")
            raise

        self._listener = _CanListener(self)
        self._notifier = can.Notifier(self.bus, [self._listener], timeout=0.1)
        self.get_logger().info(f"CAN bus connected: {interface}:{channel}")

        # 큐 깊이를 1로 둬서 과거 속도 명령이 쌓이지 않고 최신 명령만 실행한다.
        self.sub_vel = self.create_subscription(Twist, "/cmd_vel", self.velocity_callback, 1)
        self.diag_pub = self.create_publisher(String, "~/diagnostics", 10)
        self.reset_service = self.create_service(Trigger, "~/reset_fault", self.reset_fault_callback)
        self.hard_stop_service = self.create_service(
            Trigger, "~/hard_stop", self.hard_stop_callback
        )
        self.release_hard_stop_service = self.create_service(
            Trigger, "~/release_hard_stop", self.release_hard_stop_callback
        )
        self.can_timer = self.create_timer(self.can_period_sec, self.can_send_loop)
        self.diag_timer = self.create_timer(0.2, self.publish_diagnostics)

        self._status_stop = threading.Event()
        self._status_thread = threading.Thread(
            target=self._status_poll_loop,
            name="canopen-status-poll",
            daemon=True,
        )

        if self.initialize_drives("startup"):
            self._set_state("READY", "CANopen initialization verified")
        else:
            self._latch_fault("startup initialization failed")

        self._status_thread.start()
        self.get_logger().info(
            "Safe CANopen motor node ready. Fault recovery service: "
            "/safe_motor_node/reset_fault"
        )

    # ------------------------------------------------------------------
    # CAN 수신, SDO 요청/응답, 진단 이벤트 처리
    # ------------------------------------------------------------------

    def _on_can_message(self, msg: can.Message) -> None:
        """수신 스레드에서 Heartbeat, EMCY, SDO 응답 프레임을 처리한다."""
        arbitration_id = msg.arbitration_id
        data = bytes(msg.data)
        now = time.monotonic()

        with self._sdo_cv:
            if 0x580 <= arbitration_id <= 0x5FF and len(data) == 8:
                node_id = arbitration_id - 0x580
                index = data[1] | (data[2] << 8)
                self._sdo_replies[(node_id, index, data[3])] = data
                self._sdo_cv.notify_all()
                return

            if 0x700 <= arbitration_id <= 0x77F and data:
                node_id = arbitration_id - 0x700
                if node_id in self.drive_status:
                    status = self.drive_status[node_id]
                    status.heartbeat_state = data[0]
                    status.heartbeat_time = now
                return

            if 0x081 <= arbitration_id <= 0x0FF and len(data) >= 3:
                node_id = arbitration_id - 0x080
                if node_id in self.drive_status:
                    status = self.drive_status[node_id]
                    status.emcy_code = int.from_bytes(data[0:2], "little")
                    status.emcy_register = data[2]
                    self._latch_fault_locked(
                        f"Node {node_id} EMCY 0x{status.emcy_code:04X} "
                        f"(register 0x{status.emcy_register:02X})"
                    )

    def _queue_event_locked(self, level: str, message: str) -> None:
        """잠금을 이미 잡은 상태에서 나중에 출력할 진단 이벤트를 저장한다."""
        self._events.append((level, message))

    def _set_state(self, state: str, reason: str) -> None:
        """노드 상태가 달라질 때만 상태와 변경 이유를 기록한다."""
        with self._lock:
            if self.state != state:
                self.state = state
                self._queue_event_locked("INFO", f"state={state}: {reason}")

    def _latch_fault_locked(self, reason: str) -> None:
        """잠금을 잡은 상태에서 fault를 고정하고 목표 속도를 0으로 만든다."""
        if not self.fault_latched:
            self._queue_event_locked("ERROR", f"FAULT: {reason}")
        self.fault_latched = True
        self.fault_reason = reason
        self.state = "FAULT"
        self.desired_linear_mps = 0.0
        self.desired_angular_radps = 0.0
        self.latest_targets = {node_id: 0 for node_id in self.node_ids}

    def _latch_fault(self, reason: str) -> None:
        """다른 스레드에서도 안전하게 fault를 고정하는 함수."""
        with self._lock:
            self._latch_fault_locked(reason)

    def _send_can(self, arbitration_id: int, data: bytes = b"") -> bool:
        """주기 제어용 CAN 프레임을 한 번 전송하고 성공 여부를 반환한다."""
        try:
            self.bus.send(
                can.Message(
                    arbitration_id=arbitration_id,
                    data=data,
                    is_extended_id=False,
                ),
                timeout=0.0,
            )
            return True
        except can.CanError as exc:
            with self._lock:
                self._queue_event_locked(
                    "ERROR", f"CAN TX failed id=0x{arbitration_id:03X}: {exc}"
                )
            return False

    def _send_can_critical(self, arbitration_id: int, data: bytes = b"") -> None:
        """SDO/NMT처럼 반드시 전송되어야 하는 프레임을 재시도해 보낸다."""
        for attempt in range(1, self.sdo_retries + 1):
            try:
                self.bus.send(
                    can.Message(
                        arbitration_id=arbitration_id,
                        data=data,
                        is_extended_id=False,
                    ),
                    timeout=self.sdo_timeout_sec,
                )
                return
            except can.CanError as exc:
                if attempt == self.sdo_retries:
                    raise CanopenError(
                        f"CAN TX failed after {attempt} attempts, "
                        f"id=0x{arbitration_id:03X}: {exc}"
                    ) from exc
                time.sleep(0.02 * attempt)

    def _sdo_exchange(self, node_id: int, index: int, subindex: int, request: bytes) -> bytes:
        """SDO 요청 하나를 보내고 같은 index/subindex의 응답을 기다린다.

        이 노드가 사용하는 읽기와 같은 값 재쓰기 요청은 전체 요청을 재시도해도
        안전하다. 다만 CANopen Abort 응답은 원인을 확인해야 하므로 반복하지 않는다.
        """
        key = (node_id, index, subindex)
        with self._sdo_lock:
            for attempt in range(1, self.sdo_retries + 1):
                with self._sdo_cv:
                    self._sdo_replies.pop(key, None)

                self._send_can_critical(0x600 + node_id, request)
                deadline = time.monotonic() + self.sdo_timeout_sec

                with self._sdo_cv:
                    while True:
                        response = self._sdo_replies.pop(key, None)
                        if response is not None:
                            if response[0] == 0x80:
                                abort_code = int.from_bytes(response[4:8], "little")
                                description = SDO_ABORT_CODES.get(abort_code, "Unknown abort code")
                                raise CanopenError(
                                    f"Node {node_id} SDO abort 0x{abort_code:08X} "
                                    f"at 0x{index:04X}:{subindex:02X}: {description}"
                                )
                            return response

                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            break
                        self._sdo_cv.wait(remaining)

                if attempt == self.sdo_retries:
                    raise CanopenError(
                        f"Node {node_id} SDO timeout at 0x{index:04X}:{subindex:02X} "
                        f"after {attempt} attempts"
                    )
                time.sleep(0.02 * attempt)

        raise AssertionError("unreachable")

    @staticmethod
    def _sdo_download_request(index: int, subindex: int, payload: bytes) -> bytes:
        """1/2/4바이트 expedited SDO 쓰기 요청 프레임을 만든다."""
        command = {1: 0x2F, 2: 0x2B, 4: 0x23}.get(len(payload))
        if command is None:
            raise ValueError("Only 1, 2, or 4-byte expedited SDO writes are supported")
        return bytes((command, index & 0xFF, index >> 8, subindex)) + payload.ljust(4, b"\x00")

    def sdo_write(self, node_id: int, index: int, subindex: int, payload: bytes) -> None:
        """SDO 쓰기 요청을 보내고 드라이버의 쓰기 완료 응답을 확인한다."""
        response = self._sdo_exchange(
            node_id,
            index,
            subindex,
            self._sdo_download_request(index, subindex, payload),
        )
        if response[0] != 0x60:
            raise CanopenError(
                f"Node {node_id} unexpected SDO write response 0x{response[0]:02X} "
                f"at 0x{index:04X}:{subindex:02X}"
            )

    def sdo_read(self, node_id: int, index: int, subindex: int) -> bytes:
        """SDO 읽기 요청을 보내고 반환된 실제 데이터 바이트를 꺼낸다."""
        request = bytes((0x40, index & 0xFF, index >> 8, subindex, 0, 0, 0, 0))
        response = self._sdo_exchange(node_id, index, subindex, request)
        size = {0x4F: 1, 0x4B: 2, 0x47: 3, 0x43: 4}.get(response[0])
        if size is None:
            raise CanopenError(
                f"Node {node_id} unsupported SDO upload response 0x{response[0]:02X} "
                f"at 0x{index:04X}:{subindex:02X}"
            )
        return response[4 : 4 + size]

    def write_u16(self, node_id: int, index: int, subindex: int, value: int) -> None:
        """부호 없는 16비트 값을 SDO로 쓴다."""
        self.sdo_write(node_id, index, subindex, struct.pack("<H", value))

    def write_i8(self, node_id: int, index: int, subindex: int, value: int) -> None:
        """부호 있는 8비트 값을 SDO로 쓴다."""
        self.sdo_write(node_id, index, subindex, struct.pack("<b", value))

    def write_i32(self, node_id: int, index: int, subindex: int, value: int) -> None:
        """부호 있는 32비트 값을 SDO로 쓴다."""
        self.sdo_write(node_id, index, subindex, struct.pack("<i", value))

    def read_u16(self, node_id: int, index: int, subindex: int) -> int:
        """SDO에서 부호 없는 16비트 값을 읽는다."""
        return struct.unpack("<H", self.sdo_read(node_id, index, subindex))[0]

    def read_i8(self, node_id: int, index: int, subindex: int) -> int:
        """SDO에서 부호 있는 8비트 값을 읽는다."""
        return struct.unpack("<b", self.sdo_read(node_id, index, subindex))[0]

    def read_i32(self, node_id: int, index: int, subindex: int) -> int:
        """SDO에서 부호 있는 32비트 값을 읽는다."""
        return struct.unpack("<i", self.sdo_read(node_id, index, subindex))[0]

    # ------------------------------------------------------------------
    # CANopen 초기 설정과 드라이버 상태 확인
    # ------------------------------------------------------------------

    def send_nmt(self, command: int, node_id: int) -> None:
        """지정한 노드에 NMT 상태 전환 명령을 보낸다."""
        self._send_can_critical(0x000, bytes((command, node_id)))

    def send_sync(self) -> bool:
        """한 주기의 RPDO 명령을 적용시키는 SYNC 프레임을 보낸다."""
        return self._send_can(0x080)

    def _send_rpdo3(self, node_id: int, controlword: int, target_velocity: int) -> bool:
        """RPDO3에 Controlword와 목표 속도 0x60FF를 묶어 전송한다."""
        packet = struct.pack("<Hi", controlword, int(target_velocity))
        return self._send_can(self.rpdo3_ids[node_id], packet)

    def _send_velocity_cycle(self, controlword: int, targets: Dict[int, int]) -> bool:
        """네 모터 RPDO3를 보낸 뒤 SYNC를 한 번 보내는 제어 주기다."""
        ok = True
        for node_id in self.node_ids:
            ok = self._send_rpdo3(node_id, controlword, targets[node_id]) and ok
        # 모든 RPDO를 보낸 뒤에만 SYNC를 한 번 보내야 같은 주기에 적용된다.
        return self.send_sync() and ok

    def _read_and_verify_drive_state(self, node_id: int) -> None:
        """드라이버가 CSV 모드와 OPERATION_ENABLED 상태인지 확인한다."""
        statusword = self.read_u16(node_id, 0x6041, 0x00)
        mode_display = self.read_i8(node_id, 0x6061, 0x00)
        state = cia402_state(statusword)
        with self._lock:
            status = self.drive_status[node_id]
            status.statusword = statusword
            status.cia402 = state
            status.mode_display = mode_display

        if state != "OPERATION_ENABLED":
            raise CanopenError(
                f"Node {node_id} did not reach OPERATION_ENABLED: "
                f"statusword=0x{statusword:04X} ({state})"
            )
        if mode_display != 9:
            raise CanopenError(
                f"Node {node_id} CSV mode mismatch: mode display is {mode_display}, expected 9"
            )

    def initialize_drives(self, reason: str) -> bool:
        """모든 노드를 CSV 모드로 설정하고 Enable 상태까지 확인한다.

        이 함수는 PDO 매핑을 바꾸지 않는다. 현재 RPDO3 프레임 형식은 실제
        EDS/오브젝트 딕셔너리에서 매핑을 확인한 뒤에만 유효하다고 판단해야 한다.
        """
        self.get_logger().info(f"CANopen initialization started ({reason})")
        try:
            with self._lock:
                self.accept_commands = False
                self.desired_linear_mps = 0.0
                self.desired_angular_radps = 0.0
                self.latest_targets = {node_id: 0 for node_id in self.node_ids}
                self.hard_stop_active = False
                self.hard_stop_complete = False
                self.hard_stop_started_monotonic = None

            # 같은 버스의 다른 장비를 건드리지 않도록 설정된 모터 노드에만 보낸다.
            for node_id in self.node_ids:
                self.send_nmt(0x80, node_id)  # CANopen 준비 상태
            time.sleep(0.10)

            for node_id in self.node_ids:
                self.write_u16(node_id, 0x6040, 0x00, 0x0080)  # fault 초기화
                self.write_i8(node_id, 0x6060, 0x00, 9)  # CSV 동기 속도 모드
                self.write_u8_rpdo3_type(node_id)
                self.write_i32(node_id, 0x60FF, 0x00, 0)

            for node_id in self.node_ids:
                self.send_nmt(0x01, node_id)  # CANopen 운전 상태
            time.sleep(0.10)

            for controlword in (0x0006, 0x0007, 0x000F):
                for node_id in self.node_ids:
                    self.write_u16(node_id, 0x6040, 0x00, controlword)
                time.sleep(0.05)

            zero_targets = {node_id: 0 for node_id in self.node_ids}
            for _ in range(5):
                if not self._send_velocity_cycle(0x000F, zero_targets):
                    raise CanopenError("CAN transmit failure while applying zero velocity")
                time.sleep(self.can_period_sec)

            for node_id in self.node_ids:
                self._read_and_verify_drive_state(node_id)

            with self._lock:
                self.fault_latched = False
                self.fault_reason = ""
                self.accept_commands = True
                self.last_cmd_monotonic = time.monotonic()
            self.get_logger().info("CANopen initialization verified for all configured nodes")
            return True

        except (CanopenError, ValueError, struct.error) as exc:
            self.get_logger().error(f"CANopen initialization failed: {exc}")
            self._latch_fault(str(exc))
            return False

    def write_u8_rpdo3_type(self, node_id: int) -> None:
        """RPDO3 전송 방식을 SYNC 방식으로 쓴다. PDO 매핑은 변경하지 않는다."""
        # 0x1402:02는 RPDO3 전송 방식이다. 실제 매핑 0x1602는 확인 전까지 수정하지 않는다.
        self.sdo_write(node_id, 0x1402, 0x02, struct.pack("<B", 1))

    # ------------------------------------------------------------------
    # ROS 속도 명령 처리와 바퀴 목표값 계산
    # ------------------------------------------------------------------

    def velocity_callback(self, msg: Twist) -> None:
        """브레인에서 받은 최신 /cmd_vel을 저장한다.

        속도 제한이나 가감속 보정은 하지 않는다. 다만 hard stop 중에는 이전
        명령이나 새 명령이 잠금을 해제하지 못하도록 모두 무시한다.
        """
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)

        if not math.isfinite(linear) or not math.isfinite(angular):
            self._latch_fault("Rejected non-finite /cmd_vel command")
            return

        with self._lock:
            if self.hard_stop_active:
                # 과거/새 속도 명령으로 안전 잠금이 풀리면 안 된다.
                # 잠금 해제는 release_hard_stop 서비스 호출만 할 수 있다.
                return
            if not self.accept_commands or self.fault_latched or self.state == "SHUTTING_DOWN":
                self._queue_event_locked("WARN", "Ignored /cmd_vel while motor node is not command-ready")
                return

            self.desired_linear_mps = linear
            self.desired_angular_radps = angular
            self.last_cmd_monotonic = time.monotonic()

    def _update_direct_targets(self) -> Dict[int, int]:
        """최신 Twist를 좌우 바퀴의 CANopen 목표 속도로 바로 변환한다."""
        now = time.monotonic()
        with self._lock:
            timed_out = now - self.last_cmd_monotonic > self.command_timeout_sec
            hard_stop_active = self.hard_stop_active
            if self.fault_latched or hard_stop_active or not self.accept_commands or timed_out:
                desired_linear = 0.0
                desired_angular = 0.0
            else:
                desired_linear = self.desired_linear_mps
                desired_angular = self.desired_angular_radps

            # 속도 제한, 바퀴 속도 비율 축소, 가감속 램프를 적용하지 않는다.
            left_mps = desired_linear - desired_angular * self.wheel_base_m * 0.5
            right_mps = desired_linear + desired_angular * self.wheel_base_m * 0.5

            left_target = int(round(left_mps * self.velocity_scale))
            right_target = int(round(right_mps * self.velocity_scale))
            if not all(
                -(2**31) <= target < 2**31
                for target in (left_target, right_target)
            ):
                # 이는 설정 가능한 속도 제한이 아니라 CANopen 0x60FF의 부호 있는
                # 32비트 표현 범위다. 표현 불가능한 잘못된 프레임을 막는다.
                self._latch_fault_locked("/cmd_vel cannot be represented by 0x60FF")
                return {node_id: 0 for node_id in self.node_ids}

            self.latest_targets = {
                node_id: -left_target for node_id in self.left_node_ids
            }
            self.latest_targets.update(
                {node_id: right_target for node_id in self.right_node_ids}
            )

            moving = left_target != 0 or right_target != 0
            if not self.fault_latched and self.accept_commands and not hard_stop_active:
                self.state = "RUNNING" if moving else "READY"
            return dict(self.latest_targets)

    def can_send_loop(self) -> None:
        """주기마다 바퀴 목표를 RPDO3와 SYNC로 모터에 보낸다."""
        targets = self._update_direct_targets()
        with self._lock:
            controlword = (
                self.hard_stop_controlword if self.hard_stop_active else 0x000F
            )
        if not self._send_velocity_cycle(controlword, targets):
            with self._lock:
                self.consecutive_can_tx_failures += 1
                failures = self.consecutive_can_tx_failures
            if failures >= self.max_consecutive_can_tx_failures:
                self._latch_fault(
                    f"CAN transmit failure repeated {failures} consecutive cycles"
                )
            else:
                self.get_logger().warning(
                    "Temporary CAN transmit failure "
                    f"({failures}/{self.max_consecutive_can_tx_failures})"
                )
        else:
            with self._lock:
                self.consecutive_can_tx_failures = 0

    # ------------------------------------------------------------------
    # 운전 중 상태 감시, 강제 정지, 수동 fault 복구
    # ------------------------------------------------------------------

    def hard_stop_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """드라이버 Halt 비트를 잠그고 이후 속도 명령을 모두 무시한다."""
        del request
        with self._lock:
            if self.fault_latched:
                response.success = False
                response.message = "Fault is latched; use reset_fault after diagnosing it."
                return response

            if not self.hard_stop_active:
                self.hard_stop_active = True
                self.hard_stop_complete = False
                self.hard_stop_started_monotonic = time.monotonic()
                self.desired_linear_mps = 0.0
                self.desired_angular_radps = 0.0
                self.latest_targets = {node_id: 0 for node_id in self.node_ids}
                self.state = "HARD_STOPPING"
                self._queue_event_locked(
                    "WARN",
                    "Hard stop latched: /cmd_vel will be ignored until release_hard_stop",
                )

            response.success = True
            response.message = (
                "Hard stop is already complete."
                if self.hard_stop_complete
                else "Hard stop latched; waiting for all actual velocities to reach zero."
            )
            return response

    def release_hard_stop_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """네 모터가 정지했다고 확인된 뒤에만 Halt 잠금을 해제한다."""
        del request
        with self._lock:
            if self.fault_latched:
                response.success = False
                response.message = "Fault is latched; reset_fault is required first."
                return response
            if not self.hard_stop_active:
                response.success = False
                response.message = "Hard stop is not active."
                return response
            if not self.hard_stop_complete:
                response.success = False
                response.message = "Motors are not yet confirmed stopped."
                return response

            self.hard_stop_active = False
            self.hard_stop_complete = False
            self.hard_stop_started_monotonic = None
            self.desired_linear_mps = 0.0
            self.desired_angular_radps = 0.0
            self.last_cmd_monotonic = time.monotonic()
            self.state = "READY"
            self._queue_event_locked(
                "INFO",
                "Hard stop released: the next new /cmd_vel may be executed",
            )
            response.success = True
            response.message = "Hard stop released; publish the next /cmd_vel now."
            return response

    def _monitor_hard_stop(self) -> bool:
        """hard stop 중이면 실제 속도와 상태word로 완전 정지를 확인한다."""
        with self._lock:
            if not self.hard_stop_active:
                return False
            started = self.hard_stop_started_monotonic

        try:
            measured = {
                node_id: (
                    self.read_u16(node_id, 0x6041, 0x00),
                    self.read_i32(node_id, 0x606C, 0x00),
                )
                for node_id in self.node_ids
            }
        except CanopenError as exc:
            self._latch_fault(f"Hard stop could not verify actual velocity: {exc}")
            return True

        now = time.monotonic()
        with self._lock:
            for node_id, (statusword, velocity) in measured.items():
                status = self.drive_status[node_id]
                status.statusword = statusword
                status.cia402 = cia402_state(statusword)
                status.actual_velocity = velocity
                status.actual_velocity_time = now
                status.poll_failures = 0

                if status.cia402 in ("FAULT", "FAULT_REACTION_ACTIVE"):
                    self._latch_fault_locked(
                        f"Node {node_id} CiA-402 {status.cia402} during hard stop, "
                        f"statusword=0x{statusword:04X}"
                    )
                    return True

            if all(
                abs(velocity) <= self.hard_stop_velocity_threshold_raw
                for _, velocity in measured.values()
            ):
                if not self.hard_stop_complete:
                    self.hard_stop_complete = True
                    self.state = "HARD_STOPPED"
                    self._queue_event_locked(
                        "INFO",
                        "Hard stop complete: all actual velocities are within threshold",
                    )
            elif (
                started is not None
                and now - started > self.hard_stop_timeout_sec
            ):
                self._latch_fault_locked(
                    "Hard stop timeout: actual velocity did not reach the configured threshold"
                )
        return True

    def _status_poll_loop(self) -> None:
        """백그라운드에서 모터 상태, 모드, 실제 속도를 순환 조회한다."""
        node_index = 0
        while not self._status_stop.is_set():
            if self._monitor_hard_stop():
                self._status_stop.wait(self.hard_stop_monitor_period_sec)
                continue

            node_id = self.node_ids[node_index]
            node_index = (node_index + 1) % len(self.node_ids)

            try:
                with self._lock:
                    should_poll = self.state not in ("INIT", "SHUTTING_DOWN")
                if should_poll:
                    statusword = self.read_u16(node_id, 0x6041, 0x00)
                    mode = self.read_i8(node_id, 0x6061, 0x00)
                    actual_velocity = self.read_i32(node_id, 0x606C, 0x00)
                    state = cia402_state(statusword)
                    with self._lock:
                        status = self.drive_status[node_id]
                        status.statusword = statusword
                        status.cia402 = state
                        status.mode_display = mode
                        status.actual_velocity = actual_velocity
                        status.actual_velocity_time = time.monotonic()
                        status.poll_failures = 0
                        if state in ("FAULT", "FAULT_REACTION_ACTIVE"):
                            self._latch_fault_locked(
                                f"Node {node_id} CiA-402 {state}, statusword=0x{statusword:04X}"
                            )
                        elif mode != 9 and not self.fault_latched:
                            self._latch_fault_locked(
                                f"Node {node_id} mode display is {mode}, expected CSV mode 9"
                            )

                self._check_heartbeat_timeout()
            except CanopenError as exc:
                with self._lock:
                    status = self.drive_status[node_id]
                    status.poll_failures += 1
                    self._queue_event_locked("WARN", f"Node {node_id} status poll failed: {exc}")
                    if status.poll_failures >= 3:
                        self._latch_fault_locked(
                            f"Node {node_id} failed three consecutive status polls"
                        )

            self._status_stop.wait(self.status_poll_period_sec)

    def _check_heartbeat_timeout(self) -> None:
        """Heartbeat 사용이 켜진 경우 각 노드의 수신 시간 초과를 검사한다."""
        if not self.require_heartbeat:
            return
        now = time.monotonic()
        with self._lock:
            for node_id, status in self.drive_status.items():
                if status.heartbeat_time is None or now - status.heartbeat_time > self.heartbeat_timeout_sec:
                    self._latch_fault_locked(f"Node {node_id} heartbeat timeout")

    def reset_fault_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """사용자가 요청했을 때만 fault reset과 초기화를 다시 수행한다."""
        del request
        with self._lock:
            was_faulted = self.fault_latched
            self.accept_commands = False
            self.desired_linear_mps = 0.0
            self.desired_angular_radps = 0.0

        if not was_faulted:
            response.success = False
            response.message = "No latched fault. The node is already command-ready."
            return response

        self.get_logger().warn("Manual fault reset requested")
        if self.initialize_drives("manual reset_fault service"):
            self._set_state("READY", "manual fault reset succeeded")
            response.success = True
            response.message = "All configured nodes re-enabled and verified."
        else:
            response.success = False
            with self._lock:
                response.message = f"Fault recovery failed: {self.fault_reason}"
        return response

    def publish_diagnostics(self) -> None:
        """현재 상태와 누적 이벤트를 JSON 진단 토픽으로 발행한다."""
        with self._lock:
            events = list(self._events)
            self._events.clear()
            payload = {
                "state": self.state,
                "fault_latched": self.fault_latched,
                "fault_reason": self.fault_reason,
                "hard_stop": {
                    "active": self.hard_stop_active,
                    "complete": self.hard_stop_complete,
                    "velocity_threshold_raw": self.hard_stop_velocity_threshold_raw,
                },
                "desired": {
                    "linear_mps": self.desired_linear_mps,
                    "angular_radps": self.desired_angular_radps,
                },
                "nodes": {
                    str(node_id): {
                        "cia402": status.cia402,
                        "statusword": status.statusword,
                        "mode_display": status.mode_display,
                        "actual_velocity": status.actual_velocity,
                        "actual_velocity_time": status.actual_velocity_time,
                        "emcy_code": status.emcy_code,
                        "poll_failures": status.poll_failures,
                    }
                    for node_id, status in self.drive_status.items()
                },
            }

        for level, message in events:
            if level == "ERROR":
                self.get_logger().error(message)
            elif level == "WARN":
                self.get_logger().warn(message)
            else:
                self.get_logger().info(message)

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.diag_pub.publish(msg)

    # ------------------------------------------------------------------
    # 정상 종료 시 토크 해제 확인
    # ------------------------------------------------------------------

    def shutdown_to_idle(self) -> bool:
        """0속도 전송 후 Disable 상태를 요청하고 토크 해제를 확인한다."""
        self.get_logger().info("Safe shutdown: stop, disable voltage, verify state")
        with self._lock:
            self.state = "SHUTTING_DOWN"
            self.accept_commands = False
            self.desired_linear_mps = 0.0
            self.desired_angular_radps = 0.0
            self.latest_targets = {node_id: 0 for node_id in self.node_ids}
            self.hard_stop_active = False
            self.hard_stop_complete = False

        if hasattr(self, "can_timer"):
            self.can_timer.cancel()
        self._status_stop.set()
        if hasattr(self, "_status_thread") and self._status_thread.is_alive():
            self._status_thread.join(timeout=self.sdo_timeout_sec * 2.0)

        try:
            zero_targets = {node_id: 0 for node_id in self.node_ids}
            for _ in range(5):
                if not self._send_velocity_cycle(0x000F, zero_targets):
                    raise CanopenError("CAN transmit failure while stopping")
                time.sleep(self.can_period_sec)

            for node_id in self.node_ids:
                self.write_u16(node_id, 0x6040, 0x00, 0x0006)
            time.sleep(0.05)

            for node_id in self.node_ids:
                self.write_u16(node_id, 0x6040, 0x00, 0x0000)
            time.sleep(0.05)

            for node_id in self.node_ids:
                statusword = self.read_u16(node_id, 0x6041, 0x00)
                state = cia402_state(statusword)
                if state != "SWITCH_ON_DISABLED":
                    raise CanopenError(
                        f"Node {node_id} torque release was not verified: "
                        f"statusword=0x{statusword:04X} ({state})"
                    )

            self.get_logger().info("Safe shutdown verified: all motors are SWITCH_ON_DISABLED")
            return True
        except (CanopenError, ValueError, struct.error) as exc:
            self.get_logger().error(f"Safe shutdown could not be verified: {exc}")
            return False

    def destroy_node(self) -> bool:
        """CAN 수신기와 CAN 버스를 닫은 뒤 ROS 노드를 파기한다."""
        if hasattr(self, "_notifier"):
            try:
                self._notifier.stop()
            except Exception as exc:
                self.get_logger().warn(f"CAN notifier shutdown error: {exc}")
        if hasattr(self, "bus"):
            try:
                self.bus.shutdown()
            except Exception as exc:
                self.get_logger().warn(f"CAN bus shutdown error: {exc}")
        return super().destroy_node()


def main(args=None) -> None:
    """ROS 2를 초기화하고 모터 노드를 실행·정상 종료한다."""
    rclpy.init(args=args)
    node: Optional[SafeCanopenMotorNode] = None
    try:
        node = SafeCanopenMotorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Ctrl+C received")
    finally:
        if node is not None:
            node.shutdown_to_idle()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
