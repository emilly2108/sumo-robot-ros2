"""SteadyWin CSV transport extracted from the validated motor tester.

Only CAN configuration, drive state transitions, cyclic PDO/SYNC and shutdown.
No speed cap, command expiration, ramps, diagnostic publishers or automatic reset.
"""
import math
import collections
import contextlib
import json
import struct
import subprocess
import threading
import time

import can


class DriveError(RuntimeError):
    pass


def drive_state(word):
    for mask, value, name in (
        (0x4F, 0x00, "NOT_READY"), (0x4F, 0x40, "SWITCH_ON_DISABLED"),
        (0x6F, 0x21, "READY_TO_SWITCH_ON"), (0x6F, 0x23, "SWITCHED_ON"),
        (0x6F, 0x27, "OPERATION_ENABLED"), (0x6F, 0x07, "QUICK_STOP_ACTIVE"),
        (0x4F, 0x0F, "FAULT_REACTION_ACTIVE"), (0x4F, 0x08, "FAULT"),
    ):
        if word & mask == value:
            return name
    return "UNKNOWN"


class CanopenDrive:
    STARTUP_HEARTBEAT_TIMEOUT = 3.0
    STARTUP_STATUS_ATTEMPTS = 3
    STARTUP_RETRY_DELAY = .1
    TRANSIENT_CAN_ERROR_LIMIT = 5
    TRANSIENT_CAN_ERROR_WINDOW = 1.0

    def __init__(self, channel="can0", nodes=(1, 2, 3, 4), period=.05,
                 timeout=.25, info=None, error=None, bus=None, warning=None):
        if not nodes or len(set(nodes)) != len(nodes) or any(not 1 <= n <= 127 for n in nodes):
            raise ValueError("모터 ID는 중복 없는 1~127 값이어야 합니다")
        if any(not math.isfinite(v) or v <= 0 for v in (period, timeout)):
            raise ValueError("CAN 주기와 SDO 응답 시간은 양수여야 합니다")
        self.nodes, self.period, self.timeout = tuple(nodes), period, timeout
        self.info, self.error = info or (lambda _: None), error or (lambda _: None)
        self.warning = warning or self.info
        self.channel, self.real_bus = channel, bus is None
        self.cv = threading.Condition(threading.RLock())
        self.tx_lock = threading.RLock()
        self.sdo_lock = threading.Lock()
        self.stop_lock = threading.Lock()
        self.rx_stop, self.cycle_stop = threading.Event(), threading.Event()
        self.pending = self.reply = self.echo_key = None
        self.echo_seen = False
        self.tx_echo_count = 0
        self.heartbeat, self.feedback, self.generation = {}, {}, {n: 0 for n in nodes}
        self.targets = {n: 0 for n in nodes}
        self.controlword = 0
        self.zero_cycle_start = 0.0
        self.active = self.ready = self.closed = self.touched = False
        self.fault = None
        self.worker = None
        self.disable_attempted = False
        self.stop_verified = False
        self.pdo_nodes = set()
        self.verified_cycles = 0
        self.sdo_recovered_cycles = 0
        self.last_cycle_recovered = False
        self.can_error_times = collections.deque()
        self.bus = bus if bus is not None else can.Bus(
            channel=channel, interface="socketcan", receive_own_messages=True)
        self.receiver = threading.Thread(target=self._receive, name="motor-can-rx", daemon=True)
        self.receiver.start()

    def _report(self, level, text):
        # Logging must never interrupt CAN reception or the torque-disable path.
        try:
            getattr(self, level)(text)
        except Exception as exc:
            try:
                print(f"[{level.upper()}] [motor_node] {text} (로그 출력 실패: {exc})", flush=True)
            except Exception:
                pass

    def _fail(self, reason):
        with self.cv:
            first = self.fault is None
            self.fault = self.fault or str(reason)
            self.ready = False
            self.targets = {n: 0 for n in self.nodes}
            self.cycle_stop.set()
            self.cv.notify_all()
        if first:
            self._report("error", "모터 통신 중단: " + str(reason))

    @staticmethod
    def _can_error_description(msg):
        """Decode a Linux SocketCAN error frame without hiding the raw bytes."""
        cid = msg.arbitration_id
        data = bytes(msg.data).ljust(8, b"\0")
        classes = [name for bit, name in (
            (0x001, "TX_TIMEOUT"), (0x002, "LOST_ARB"),
            (0x004, "CONTROLLER"), (0x008, "PROTOCOL"),
            (0x010, "TRANSCEIVER"), (0x020, "NO_ACK"),
            (0x040, "BUS_OFF"), (0x080, "BUS_ERROR"),
            (0x100, "RESTARTED"), (0x200, "COUNTERS"),
        ) if cid & bit]
        controller = [name for bit, name in (
            (0x01, "RX_OVERFLOW"), (0x02, "TX_OVERFLOW"),
            (0x04, "RX_WARNING"), (0x08, "TX_WARNING"),
            (0x10, "RX_PASSIVE"), (0x20, "TX_PASSIVE"),
            (0x40, "ACTIVE"),
        ) if data[1] & bit]
        protocol = [name for bit, name in (
            (0x01, "BIT"), (0x02, "FORM"), (0x04, "STUFF"),
            (0x08, "BIT0"), (0x10, "BIT1"), (0x20, "OVERLOAD"),
            (0x40, "ACTIVE_ERROR"), (0x80, "TX"),
        ) if data[2] & bit]
        parts = [
            "class=" + ("|".join(classes) or "UNSPEC"),
            "controller=" + ("|".join(controller) or "none"),
            "protocol=" + ("|".join(protocol) or "none"),
            f"txerr={data[6]}", f"rxerr={data[7]}",
            "raw=" + bytes(msg.data).hex(),
        ]
        return ", ".join(parts), cid, data

    def _handle_can_error(self, msg):
        """Stop on severe/repeated errors, but let CAN retry one bit error."""
        description, cid, data = self._can_error_description(msg)
        now = time.monotonic()
        while self.can_error_times and now-self.can_error_times[0] > self.TRANSIENT_CAN_ERROR_WINDOW:
            self.can_error_times.popleft()
        self.can_error_times.append(now)

        controller_severe = bool(data[1] & 0x3F)
        transceiver_severe = bool((cid & 0x010) and data[4])
        counter_severe = data[6] >= 96 or data[7] >= 96
        immediate = bool(cid & (0x001 | 0x040)) or controller_severe or transceiver_severe or counter_severe
        repeated = len(self.can_error_times) >= self.TRANSIENT_CAN_ERROR_LIMIT
        if immediate or repeated:
            suffix = "즉시 중단 조건" if immediate else (
                f"{self.TRANSIENT_CAN_ERROR_WINDOW:.1f}초 내 "
                f"{len(self.can_error_times)}회 반복")
            self._fail(f"CAN 오류 ({suffix}): {description}")
            return

        self._report(
            "warning",
            f"CAN 일시 오류 감시 {len(self.can_error_times)}/{self.TRANSIENT_CAN_ERROR_LIMIT}: "
            f"{description}; 다음 SYNC 피드백을 계속 검증합니다")

    def _receive(self):
        try:
            while not self.rx_stop.is_set():
                msg = self.bus.recv(timeout=.05)
                if msg is None:
                    continue
                if msg.is_error_frame:
                    self._handle_can_error(msg)
                    continue
                if msg.is_remote_frame or msg.is_extended_id:
                    continue
                cid, data, now = msg.arbitration_id, bytes(msg.data), time.monotonic()
                with self.cv:
                    if not msg.is_rx:
                        self.tx_echo_count += 1
                        if self.echo_key == (cid, data):
                            self.echo_seen = True
                    elif 0x700 < cid <= 0x77F and len(data) == 1:
                        n = cid-0x700
                        self.heartbeat[n] = (now, data[0])
                        if n in self.nodes and self.active and data[0] != 5:
                            self._fail(f"모터 {n}: NMT 운전 상태 이탈 (상태={data[0]:02X})")
                    elif cid-0x380 in self.nodes and len(data) == 6:
                        n = cid-0x380
                        word, velocity = struct.unpack("<Hi", data)
                        self.feedback[n] = (now, word, velocity)
                        self.generation[n] += 1
                        if self.active and drive_state(word) in ("FAULT", "FAULT_REACTION_ACTIVE"):
                            self._fail(f"모터 {n}: 드라이버 고장 (상태=0x{word:04X})")
                    elif self.pending and cid == 0x580+self.pending[0] and len(data) == 8:
                        if data[1:4] == self.pending[1]:
                            self.reply = data
                    elif 0x080 < cid <= 0x0FF and cid-0x080 in self.nodes and len(data) >= 3:
                        code = int.from_bytes(data[:2], "little")
                        if code:
                            self._fail(f"모터 {cid-0x080}: EMCY 0x{code:04X}")
                    self.cv.notify_all()
        except Exception as exc:
            if not self.rx_stop.is_set():
                self._fail("CAN 수신 실패: " + str(exc))

    def _send(self, cid, data=b"", echo=False, allow_fault=False):
        with self.tx_lock:
            if self.fault and not allow_fault:
                raise DriveError(self.fault)
            with self.cv:
                if echo:
                    self.echo_key, self.echo_seen = (cid, bytes(data)), False
            try:
                self.bus.send(can.Message(arbitration_id=cid, data=data, is_extended_id=False), timeout=.02)
                if echo:
                    deadline = time.monotonic()+self.timeout
                    with self.cv:
                        while not self.echo_seen:
                            if self.fault and not allow_fault:
                                raise DriveError(self.fault)
                            left = deadline-time.monotonic()
                            if left <= 0:
                                raise DriveError(f"송신 확인 시간 초과: CAN ID 0x{cid:03X}")
                            self.cv.wait(left)
            finally:
                if echo:
                    with self.cv:
                        self.echo_key = None

    @contextlib.contextmanager
    def _sdo_window(self, deadline=None):
        if deadline is None:
            with self.sdo_lock, self.tx_lock:
                yield
            return
        acquired = []
        try:
            for lock in (self.sdo_lock, self.tx_lock):
                remaining = deadline-time.monotonic()
                if remaining <= 0 or not lock.acquire(timeout=remaining):
                    raise DriveError("SDO 확인 제한 시간 초과: 송신 잠금 대기")
                acquired.append(lock)
            yield
        finally:
            for lock in reversed(acquired):
                lock.release()

    def _exchange(self, n, index, sub=0, payload=None, allow_fault=False, deadline=None):
        address = bytes((index & 255, index >> 8, sub))
        request = (b"\x40"+address+bytes(4) if payload is None else
                   bytes(({1: 0x2F, 2: 0x2B, 4: 0x23}[len(payload)],))+address+payload.ljust(4, b"\0"))
        # Keep the whole request/reply transaction outside the PDO/SYNC window.
        with self._sdo_window(deadline):
            with self.cv:
                self.pending, self.reply = (n, address), None
            try:
                if deadline is not None and time.monotonic() >= deadline:
                    raise DriveError(f"모터 {n}: SDO 확인 제한 시간 초과, 0x{index:04X}:{sub:02X}")
                # Serialize SDO requests at the actual gs_usb TX-completion
                # boundary so a stalled adapter cannot accumulate requests.
                self._send(0x600+n, request, echo=True, allow_fault=allow_fault)
                response_deadline = time.monotonic()+self.timeout
                if deadline is not None:
                    response_deadline = min(response_deadline, deadline)
                with self.cv:
                    while self.reply is None:
                        if self.fault and not allow_fault:
                            raise DriveError(self.fault)
                        left = response_deadline-time.monotonic()
                        if left <= 0:
                            raise DriveError(f"모터 {n}: SDO 응답 시간 초과, 0x{index:04X}:{sub:02X} "
                                             f"(응답 CAN ID 0x{0x580+n:03X})")
                        self.cv.wait(left)
                    reply = self.reply
                if reply[0] == 0x80:
                    raise DriveError(f"모터 {n}: SDO 거절 0x{index:04X}:{sub:02X}, "
                                     f"코드 0x{int.from_bytes(reply[4:], 'little'):08X}")
                if payload is not None:
                    if reply[0] != 0x60:
                        raise DriveError(f"모터 {n}: 비정상 SDO 쓰기 응답 {reply.hex()}")
                    return b""
                size = {0x4F: 1, 0x4B: 2, 0x47: 3, 0x43: 4}.get(reply[0])
                if size is None:
                    raise DriveError(f"모터 {n}: 비정상 SDO 읽기 응답 {reply.hex()}")
                return reply[4:4+size]
            finally:
                with self.cv:
                    self.pending = self.reply = None

    def read(self, n, index, sub=0, signed=False, allow_fault=False, deadline=None):
        return int.from_bytes(self._exchange(n, index, sub, allow_fault=allow_fault, deadline=deadline),
                              "little", signed=signed)

    def _tx_backlog(self):
        if not self.real_bus:
            return None
        try:
            data = subprocess.check_output(['tc', '-j', '-s', 'qdisc', 'show', 'dev', self.channel],
                                           timeout=.5, stderr=subprocess.DEVNULL)
            return sum(int(item.get('qlen', 0)) for item in json.loads(data))
        except (OSError, ValueError, subprocess.SubprocessError):
            return None

    def _heartbeat_ages(self, now=None):
        now = time.monotonic() if now is None else now
        with self.cv:
            return {
                n: (round((now-self.heartbeat[n][0])*1000, 1)
                    if n in self.heartbeat else None)
                for n in self.nodes
            }

    def _wait_startup_heartbeats(self):
        started = time.monotonic()
        deadline = started+self.STARTUP_HEARTBEAT_TIMEOUT
        with self.cv:
            while True:
                missing = [n for n in self.nodes
                           if n not in self.heartbeat or self.heartbeat[n][0] < started]
                if not missing:
                    return
                left = deadline-time.monotonic()
                if left <= 0:
                    break
                self.cv.wait(left)
        backlog = self._tx_backlog()
        raise DriveError(
            f"시작 전 CAN 하트비트 시간 초과: 모터 {missing}; "
            f"제한={self.STARTUP_HEARTBEAT_TIMEOUT:.1f}s, "
            f"마지막 하트비트 경과(ms)={self._heartbeat_ages()}, "
            f"송신 대기열={backlog if backlog is not None else '확인 불가'}. "
            "토크 인가는 수행하지 않습니다")

    def _read_startup_states(self):
        states, recovered = {}, []
        for n in self.nodes:
            for attempt in range(1, self.STARTUP_STATUS_ATTEMPTS+1):
                try:
                    first = self.read(n, 0x6041)
                    second = self.read(n, 0x6041)
                    states[n] = drive_state(second)
                    if attempt > 1:
                        recovered.append((n, attempt))
                    break
                except DriveError as exc:
                    if "SDO 응답 시간 초과" not in str(exc):
                        raise
                    backlog = self._tx_backlog()
                    if backlog:
                        raise DriveError(
                            f"모터 {n}: 시작 전 SDO 확인 중 송신 대기열 {backlog}개 발생, "
                            "0x6041:00; 토크 인가는 수행하지 않습니다") from exc
                    if attempt == self.STARTUP_STATUS_ATTEMPTS:
                        raise DriveError(
                            f"모터 {n}: 시작 전 SDO 응답 시간 초과, 0x6041:00; "
                            f"시도={attempt}/{self.STARTUP_STATUS_ATTEMPTS}, "
                            f"마지막 하트비트 경과(ms)={self._heartbeat_ages()}, "
                            f"송신 대기열={backlog if backlog is not None else '확인 불가'}. "
                            "토크 인가는 수행하지 않습니다") from exc
                    time.sleep(self.STARTUP_RETRY_DELAY)
        if recovered:
            detail = ", ".join(f"모터 {n} {attempt}/{self.STARTUP_STATUS_ATTEMPTS}회" for n, attempt in recovered)
            self._report("warning", "시작 전 SDO 일시 누락 복구: "+detail)
        return states

    def _write(self, n, index, sub, value, size, signed=False, allow_fault=False):
        self.touched = True
        self._exchange(n, index, sub, value.to_bytes(size, "little", signed=signed), allow_fault)
        actual = self.read(n, index, sub, signed, allow_fault)
        if actual != value:
            raise DriveError(f"모터 {n}: 설정 확인 실패 0x{index:04X}:{sub:02X}, 요청={value}, 읽은값={actual}")

    def _nmt(self, n, command, state):
        stamp = time.monotonic()
        self._send(0, bytes((command, n)), echo=True)
        deadline = stamp+1.5
        with self.cv:
            while True:
                received, actual = self.heartbeat.get(n, (0, None))
                if received > stamp and actual == state:
                    return
                if self.fault:
                    raise DriveError(self.fault)
                left = deadline-time.monotonic()
                if left <= 0:
                    raise DriveError(f"모터 {n}: NMT 상태 전환 확인 실패 (목표={state:02X})")
                self.cv.wait(left)

    def _wait_state(self, n, expected):
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            word = self.read(n, 0x6041)
            state = drive_state(word)
            if state == expected:
                return
            if state in ("FAULT", "FAULT_REACTION_ACTIVE"):
                raise DriveError(f"모터 {n}: 드라이버 고장 (상태=0x{word:04X})")
            time.sleep(.02)
        raise DriveError(f"모터 {n}: 상태 전환 실패, 목표={expected}, 현재={state} (0x{word:04X})")

    def _mapping(self, n, index, expected):
        count = self.read(n, index)
        if count > 8:
            raise DriveError(f"모터 {n}: 비정상 PDO 매핑 개수 {count}")
        mapping = [self.read(n, index, i) for i in range(1, count+1)]
        while mapping and mapping[-1] == 0:
            mapping.pop()
        if mapping != expected:
            raise DriveError(f"모터 {n}: PDO 매핑 불일치 0x{index:04X}, "
                             f"현재={[hex(v) for v in mapping]}")

    def _pdo(self, n, index, expected_cob, enabled=True):
        cob, kind = self.read(n, index, 1), self.read(n, index, 2)
        if cob & 0x7FFFFFFF != expected_cob:
            raise DriveError(f"모터 {n}: PDO CAN ID 불일치 0x{index:04X}, 현재=0x{cob:08X}")
        desired = expected_cob if enabled else expected_cob | 0x80000000
        if enabled and kind != 1:
            if not cob & 0x80000000:
                self._write(n, index, 1, cob | 0x80000000, 4)
            self._write(n, index, 2, 1, 1)
            self._write(n, index, 1, desired, 4)
        elif cob != desired:
            self._write(n, index, 1, desired, 4)

    def start(self):
        try:
            self._report("info", f"[1/3] 모터 {len(self.nodes)}축 CANopen 설정 확인")
            backlog = self._tx_backlog()
            if backlog:
                raise DriveError(f"{self.channel} 송신 대기열 {backlog}개가 남아 있습니다. "
                                 "CAN 복구 후 다시 실행하세요. 토크 인가는 수행하지 않습니다")
            self._wait_startup_heartbeats()
            states = self._read_startup_states()
            for n in self.nodes:
                self._mapping(n, 0x1602, [0x60400010, 0x60FF0020])
                self._mapping(n, 0x1A02, [0x60410010, 0x606C0020])
                if self.read(n, 0x1005) != 0x80:
                    raise DriveError(f"모터 {n}: SYNC CAN ID/Producer 설정 불일치")
                cob = self.read(n, 0x1402, 1)
                if cob & 0x7FFFFFFF != 0x400+n:
                    raise DriveError(f"모터 {n}: RPDO3 CAN ID 불일치")
                if not cob & 0x80000000:
                    self.pdo_nodes.add(n)
            # A previous synchronous RPDO can remain active after SDO writes.
            # Clear it with ZERO/DISABLE PDO + SYNC before configuring or enabling.
            if any(state != "SWITCH_ON_DISABLED" for state in states.values()):
                if not self._disable():
                    raise DriveError("시작 전 4축 정지·토크 해제를 확인하지 못했습니다")
            for n, state in states.items():
                if state in ("FAULT", "FAULT_REACTION_ACTIVE"):
                    raise DriveError(f"모터 {n}: 고장 상태입니다. 원인 해소 후 다시 시작하세요")
            self.stop_verified = False
            for n in self.nodes:
                self._nmt(n, 0x80, 127)
                period = self.read(n, 0x1006)
                if period not in (0, round(self.period*1e6)):
                    self._write(n, 0x1006, 0, round(self.period*1e6), 4)
                if self.read(n, 0x6060, signed=True) != 9:
                    self._write(n, 0x6060, 0, 9, 1, signed=True)
                self._pdo(n, 0x1402, 0x400+n)
                self.pdo_nodes.add(n)
                for i in range(4):
                    self._pdo(n, 0x1800+i, 0x180+i*0x100+n, enabled=i == 2)
                self._write(n, 0x60FF, 0, 0, 4, signed=True)
            self._report("info", "[2/3] 목표 속도 0으로 동기 제어 및 토크 인가")
            for n in self.nodes:
                self._nmt(n, 1, 5)
            self.active, self.controlword = True, 6
            self.worker = threading.Thread(target=self._cycle, name="motor-csv", daemon=True)
            self.worker.start()
            for cw, state in ((6, "READY_TO_SWITCH_ON"), (7, "SWITCHED_ON"), (15, "OPERATION_ENABLED")):
                with self.cv:
                    self.controlword = cw
                for n in self.nodes:
                    self._wait_state(n, state)
            for n in self.nodes:
                if self.read(n, 0x6061, signed=True) != 9:
                    raise DriveError(f"모터 {n}: CSV 모드 적용 확인 실패")
            if self.fault:
                raise DriveError(self.fault)
            self.ready = True
            self._report("info", f"[3/3] 모터 {len(self.nodes)}축 준비 완료: cmd_vel 수신 대기")
        except (Exception, KeyboardInterrupt) as exc:
            self._fail(str(exc) or "모터 초기화 중단")
            self.close()
            raise

    def set_targets(self, targets):
        if set(targets) != set(self.nodes):
            raise ValueError("속도 명령의 모터 ID가 설정과 다릅니다")
        if any(not math.isfinite(v) or v < -(2**31) or v >= 2**31 for v in targets.values()):
            raise ValueError("속도값은 유한한 CAN signed32bit 범위여야 합니다")
        with self.cv:
            stop_again = bool(self.fault) and all(v == 0 for v in targets.values())
            if (not self.ready or self.fault) and not stop_again:
                raise DriveError(self.fault or "모터가 아직 준비되지 않았습니다")
            self.targets = {n: int(v) for n, v in targets.items()}
        if stop_again:
            # An explicit ZERO must remain usable after a communication fault.
            # It retries stop only; it never clears FAULT or enables torque.
            if self.worker and self.worker is not threading.current_thread():
                self.worker.join(timeout=self.timeout*len(self.nodes)*12+3)
                if self.worker.is_alive():
                    raise DriveError("정지 작업이 진행 중입니다. 독립 전원 차단을 사용하세요")
            if not self._disable(force=True):
                raise DriveError("정지·토크 해제 미확인: 독립 전원 차단이 필요합니다")

    def _recover_feedback(self, missing, targets, cw, reason):
        if not self.ready or cw != 15 or self.last_cycle_recovered:
            suffix = "연속 주기 누락" if self.last_cycle_recovered else "운전 준비 중 누락"
            raise DriveError(reason+"; "+suffix)
        # Re-check actual drive data, never just ignore a missing TPDO.
        # The WHOLE recovery, including lock waits, is limited to one extra period.
        deadline = time.monotonic()+min(self.period, self.timeout)
        try:
            for n in missing:
                word = self.read(n, 0x6041, deadline=deadline)
                mode = self.read(n, 0x6061, signed=True, deadline=deadline)
                target = self.read(n, 0x60FF, signed=True, deadline=deadline)
                self.read(n, 0x606C, signed=True, deadline=deadline)
                if drive_state(word) != 'OPERATION_ENABLED' or mode != 9 or target != targets[n]:
                    raise DriveError(f"모터 {n}: SDO 재확인 불일치, 상태=0x{word:04X}, "
                                     f"모드={mode}, 목표={target}/요청={targets[n]}")
            if time.monotonic() > deadline:
                raise DriveError("SDO 재확인 전체 제한 시간 초과")
            if self.fault:
                raise DriveError(self.fault)
        except Exception as exc:
            raise DriveError(reason+"; SDO 재확인 실패: "+str(exc)) from exc
        self.sdo_recovered_cycles += 1
        if self.sdo_recovered_cycles == 1:
            self._report("warning", f"모터 {missing}: 동기 피드백 단발 누락, "
                         "SDO로 운전 상태·모드·목표·실제 속도 확인 후 제어 유지 "
                         "(동일 회복 알림은 생략, 연속 누락은 정지)")

    def _cycle(self):
        deadline = time.monotonic()
        try:
            while not self.cycle_stop.is_set():
                now = time.monotonic()
                if now-deadline > self.period*2:
                    raise DriveError("동기 제어 주기 지연: 밀린 프레임은 몰아서 전송하지 않습니다")
                with self.cv:
                    targets, cw = dict(self.targets), self.controlword
                missing, reason = [], None
                with self.tx_lock:
                    tx_started = time.monotonic()
                    for n in self.nodes:
                        self._send(0x400+n, struct.pack("<Hi", cw, targets[n]), echo=True)
                    rpdo_done = time.monotonic()
                    with self.cv:
                        before = dict(self.generation)
                    self._send(0x80, echo=True)
                    sync_echo = time.monotonic()
                    with self.cv:
                        while any(self.generation[n] <= before[n] for n in self.nodes):
                            if self.fault:
                                raise DriveError(self.fault)
                            left = now+self.period-time.monotonic()
                            if left <= 0:
                                missing = [n for n in self.nodes if self.generation[n] <= before[n]]
                                failed_at = time.monotonic()
                                ages = {n: round((failed_at-self.feedback[n][0])*1000, 1)
                                        if n in self.feedback else None for n in missing}
                                reason = (
                                    f"동기 피드백 누락: 모터 {missing}; "
                                    f"주기 #{self.verified_cycles+1}, "
                                    f"제한={self.period*1000:.1f}ms/경과={(failed_at-now)*1000:.1f}ms, "
                                    f"송신잠금={(tx_started-now)*1000:.1f}ms, "
                                    f"RPDO에코={(rpdo_done-tx_started)*1000:.1f}ms, "
                                    f"SYNC에코={(sync_echo-rpdo_done)*1000:.1f}ms, "
                                    f"응답대기={(failed_at-sync_echo)*1000:.1f}ms, "
                                    f"직전피드백경과(ms)={ages}")
                                break
                            self.cv.wait(left)
                # Release the PDO lock before acquiring the SDO transaction lock.
                # This preserves lock ordering against other SDO callers.
                if missing:
                    self._recover_feedback(missing, targets, cw, reason)
                with self.cv:
                    if self.fault:
                        raise DriveError(self.fault)
                    self.last_cycle_recovered = bool(missing)
                    self.verified_cycles += 1
                    if all(v == 0 for v in targets.values()):
                        self.zero_cycle_start = now
                    self.cv.notify_all()
                deadline += self.period
                if missing:
                    # Never burst out the cycle that was consumed by verification.
                    deadline = max(deadline, time.monotonic())
                self.cycle_stop.wait(max(0, deadline-time.monotonic()))
        except Exception as exc:
            self._fail(str(exc))
        finally:
            if self.fault:
                self._disable()

    def _stop_pdos(self):
        # Do not wait for the broken TX echo before attempting the other axes.
        # Actual SDO state/target/position, rather than an echo, proves stopping.
        for n in self.nodes:
            if n in self.pdo_nodes:
                try:
                    self._send(0x400+n, struct.pack("<Hi", 0, 0), allow_fault=True)
                except Exception:
                    pass
        if self.pdo_nodes:
            try:
                self._send(0x80, allow_fault=True)
            except Exception:
                pass

    def _stationary(self, timeout=1.0):
        """Verify encoder position stays within 8 counts over at least 250ms."""
        samples = collections.deque()
        deadline = time.monotonic()+timeout
        ranges = {}
        while time.monotonic() < deadline:
            positions = {n: self.read(n, 0x6064, signed=True, allow_fault=True) for n in self.nodes}
            stamp = time.monotonic()
            samples.append((stamp, positions))
            while len(samples) > 1 and stamp-samples[1][0] >= .25:
                samples.popleft()
            if stamp-samples[0][0] >= .25:
                for n in self.nodes:
                    origin = samples[0][1][n]
                    deltas = [((p[n]-origin+2**31) % 2**32)-2**31 for _, p in samples]
                    ranges[n] = max(deltas)-min(deltas)
                if all(v <= 8 for v in ranges.values()):
                    return
            time.sleep(.05)
        raise DriveError(f"엔코더 위치 정지 미확인: 위치 변화(count)={ranges}")

    def _disable(self, force=False):
        with self.stop_lock:
            if self.stop_verified and not force:
                return True
            self.disable_attempted = True
            self.stop_verified = False
            with self.cv:
                self.ready = False
                self.targets = {n: 0 for n in self.nodes}
                self.controlword = 0
            failures = []
            # A bounded retry can repair one missed frame; it never re-enables a drive.
            for _ in range(3):
                before_echo = self.tx_echo_count
                self._stop_pdos()
                failures = []
                for n in self.nodes:
                    try:
                        word = self.read(n, 0x6041, allow_fault=True)
                        target = self.read(n, 0x60FF, signed=True, allow_fault=True)
                        if drive_state(word) != "SWITCH_ON_DISABLED" or target != 0:
                            self._write(n, 0x60FF, 0, 0, 4, signed=True, allow_fault=True)
                            self._write(n, 0x6040, 0, 0, 2, allow_fault=True)
                            failures.append(f"모터 {n}: 상태=0x{word:04X}, 목표={target}")
                    except Exception as exc:
                        failures.append(f"모터 {n}: {exc}")
                if not failures:
                    try:
                        self._stationary()
                        # Re-read after the observation window; an SDO ACK is insufficient.
                        for n in self.nodes:
                            word = self.read(n, 0x6041, allow_fault=True)
                            target = self.read(n, 0x60FF, signed=True, allow_fault=True)
                            if drive_state(word) != "SWITCH_ON_DISABLED" or target != 0:
                                raise DriveError(f"모터 {n}: 정지 상태 유지 실패 (0x{word:04X}, 목표={target})")
                    except Exception as exc:
                        failures.append(str(exc))
                if not failures:
                    self.active = False
                    self.stop_verified = True
                    # Isolate the already disabled drives from later stray SYNC/PDOs.
                    for n in self.nodes:
                        try:
                            self._send(0, bytes((0x80, n)), allow_fault=True)
                        except Exception:
                            pass
                    self._report("info", f"모터 {len(self.nodes)}축 목표 0·위치 정지·토크 해제 확인 완료")
                    return True
                backlog = self._tx_backlog()
                if backlog or self.tx_echo_count == before_echo:
                    failures.append(f"송신 진행 정체: 대기열={backlog}, "
                                    "추가 정지 프레임 적재를 중단합니다")
                    break
                time.sleep(.05)
            self._report("error", "정지·토크 해제 미확인: " + "; ".join(failures)
                         + ". 독립 전원 차단이 필요합니다")
            return False

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.active and not self.fault:
                with self.cv:
                    self.ready = False
                    self.targets = {n: 0 for n in self.nodes}
                    stamp = time.monotonic()
                    deadline = stamp+max(.5, self.period*4)
                    while self.zero_cycle_start < stamp and not self.fault:
                        left = deadline-time.monotonic()
                        if left <= 0:
                            self._fail("종료 시 목표 속도 0의 동기 전송 확인 실패")
                            break
                        self.cv.wait(left)
                if not self.fault:
                    try:
                        # Keep zero-speed control active until the rotating wheels settle.
                        self._stationary(timeout=1.5)
                    except Exception as exc:
                        self._report("error", "종료 시 감속 정지 미확인: " + str(exc))
            self.cycle_stop.set()
            if self.worker and self.worker is not threading.current_thread():
                self.worker.join(timeout=self.timeout*len(self.nodes)*12+3)
                if self.worker.is_alive():
                    self._report("error", "송신 스레드 종료 미확인: 토크 해제 상태를 확인하세요")
                    return
            if self.active or (self.touched and not self.worker) or self.disable_attempted:
                self._disable()
        finally:
            self.rx_stop.set()
            self.receiver.join(timeout=1)
            self.bus.shutdown()
