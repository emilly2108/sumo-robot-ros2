"""D435i IMU용 충격 내성 기울기 판정기.

이 모듈은 ROS나 RealSense SDK에 의존하지 않는다.  따라서 기록한 IMU 로그나
``test_tilt_detector.py``로 먼저 검증할 수 있다. 입력 가속도는 m/s², 자이로는
rad/s, 타임스탬프는 초 단위여야 한다.

기준 자세는 시작 직후 안정적인 가속도 샘플로 잡는다. 충격으로 가속도 크기가
1 g에서 크게 벗어나면 가속도 보정과 15도 판정을 잠시 중지한다. 그 사이에는
자이로로만 중력 방향을 예측하므로, 1 cm 턱이나 충돌의 짧은 가속도를 기울기로
잘못 판정하지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple


Vector3 = Tuple[float, float, float]
STANDARD_GRAVITY = 9.80665


@dataclass(frozen=True)
class TiltStatus:
    """현재 기울기 상태.

    ``angle_deg``는 기준 자세에서 벗어난 전체 기울기(pitch/roll 결합)다.
    yaw 회전은 중력 방향을 바꾸지 않으므로 이 값에 영향을 주지 않는다.
    """

    ready: bool
    angle_deg: float
    tilted: bool
    impact_active: bool
    calibration_progress: float
    just_calibrated: bool = False


class TiltDetector:
    """자이로 예측 + 가속도 보정 방식의 pitch/roll 임계값 판정기."""

    def __init__(
        self,
        threshold_deg: float = 15.0,
        release_deg: float = 12.0,
        confirm_time_sec: float = 0.25,
        release_time_sec: float = 0.25,
        impact_hold_sec: float = 0.30,
        accel_norm_tolerance_g: float = 0.20,
        accel_correction_time_sec: float = 0.20,
        calibration_samples: int = 50,
    ) -> None:
        if threshold_deg <= 0.0:
            raise ValueError("threshold_deg must be positive")
        if not 0.0 <= release_deg < threshold_deg:
            raise ValueError("release_deg must be non-negative and below threshold_deg")
        if calibration_samples < 1:
            raise ValueError("calibration_samples must be at least one")

        self.threshold_deg = threshold_deg
        self.release_deg = release_deg
        self.confirm_time_sec = confirm_time_sec
        self.release_time_sec = release_time_sec
        self.impact_hold_sec = impact_hold_sec
        self.accel_norm_tolerance = accel_norm_tolerance_g * STANDARD_GRAVITY
        self.accel_correction_time_sec = accel_correction_time_sec
        self.calibration_samples = calibration_samples

        self._reference_gravity: Optional[Vector3] = None
        self._gravity_estimate: Optional[Vector3] = None
        self._calibration_sum: Vector3 = (0.0, 0.0, 0.0)
        self._calibration_count = 0
        self._last_accel_timestamp: Optional[float] = None
        self._last_gyro_timestamp: Optional[float] = None
        self._last_timestamp: Optional[float] = None
        self._impact_until = float("-inf")
        self._above_since: Optional[float] = None
        self._below_since: Optional[float] = None
        self._tilted = False
        self._angle_deg = 0.0
        self._just_calibrated = False

    @property
    def ready(self) -> bool:
        return self._reference_gravity is not None

    @staticmethod
    def _add(left: Vector3, right: Vector3) -> Vector3:
        return (left[0] + right[0], left[1] + right[1], left[2] + right[2])

    @staticmethod
    def _scale(vector: Vector3, value: float) -> Vector3:
        return (vector[0] * value, vector[1] * value, vector[2] * value)

    @staticmethod
    def _dot(left: Vector3, right: Vector3) -> float:
        return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]

    @staticmethod
    def _cross(left: Vector3, right: Vector3) -> Vector3:
        return (
            left[1] * right[2] - left[2] * right[1],
            left[2] * right[0] - left[0] * right[2],
            left[0] * right[1] - left[1] * right[0],
        )

    @classmethod
    def _normalize(cls, vector: Vector3) -> Optional[Vector3]:
        length_sq = cls._dot(vector, vector)
        if length_sq <= 1e-12:
            return None
        return cls._scale(vector, 1.0 / math.sqrt(length_sq))

    def update_gyro(self, gyro_rad_s: Vector3, timestamp_sec: float) -> TiltStatus:
        """새 자이로 샘플을 반영한다.

        월드에 고정된 중력 벡터를 현재 센서 좌표계로 적분한다. 샘플 간격이 너무
        길면 충격 뒤의 큰 오차를 막기 위해 적분하지 않고 다음 가속도 보정을 기다린다.
        """
        self._just_calibrated = False
        self._last_timestamp = timestamp_sec

        if self._last_gyro_timestamp is not None and timestamp_sec <= self._last_gyro_timestamp:
            return self.status(timestamp_sec)

        if self._gravity_estimate is not None and self._last_gyro_timestamp is not None:
            dt = timestamp_sec - self._last_gyro_timestamp
            if 0.0 < dt <= 0.10:
                # d(g_body)/dt = -omega_body x g_body
                derivative = self._scale(self._cross(gyro_rad_s, self._gravity_estimate), -1.0)
                predicted = self._add(self._gravity_estimate, self._scale(derivative, dt))
                normalized = self._normalize(predicted)
                if normalized is not None:
                    self._gravity_estimate = normalized
                    self._update_angle_and_latch(timestamp_sec)

        self._last_gyro_timestamp = timestamp_sec
        return self.status(timestamp_sec)

    def update_accel(self, accel_m_s2: Vector3, timestamp_sec: float) -> TiltStatus:
        """새 가속도계 샘플을 반영한다."""
        self._just_calibrated = False
        self._last_timestamp = timestamp_sec

        if self._last_accel_timestamp is not None and timestamp_sec <= self._last_accel_timestamp:
            return self.status(timestamp_sec)

        norm = math.sqrt(self._dot(accel_m_s2, accel_m_s2))
        if abs(norm - STANDARD_GRAVITY) > self.accel_norm_tolerance:
            # 충격 또는 큰 선형 가속도: 중력 방향 보정·새 기울기 판정 금지.
            self._impact_until = max(self._impact_until, timestamp_sec + self.impact_hold_sec)
            return self.status(timestamp_sec)

        measured_gravity = self._normalize(accel_m_s2)
        if measured_gravity is None:
            self._impact_until = max(self._impact_until, timestamp_sec + self.impact_hold_sec)
            return self.status(timestamp_sec)

        if not self.ready:
            self._calibration_sum = self._add(self._calibration_sum, measured_gravity)
            self._calibration_count += 1
            self._last_accel_timestamp = timestamp_sec
            if self._calibration_count >= self.calibration_samples:
                reference = self._normalize(self._calibration_sum)
                if reference is not None:
                    self._reference_gravity = reference
                    self._gravity_estimate = reference
                    self._angle_deg = 0.0
                    self._just_calibrated = True
            return self.status(timestamp_sec)

        if self._last_accel_timestamp is None:
            alpha = 1.0
        else:
            dt = max(0.0, timestamp_sec - self._last_accel_timestamp)
            alpha = 1.0 - math.exp(-dt / self.accel_correction_time_sec)
            alpha = min(max(alpha, 0.0), 1.0)

        estimate = self._gravity_estimate or measured_gravity
        corrected = self._add(
            self._scale(estimate, 1.0 - alpha),
            self._scale(measured_gravity, alpha),
        )
        normalized = self._normalize(corrected)
        if normalized is not None:
            self._gravity_estimate = normalized
        self._last_accel_timestamp = timestamp_sec
        self._update_angle_and_latch(timestamp_sec)
        return self.status(timestamp_sec)

    def _update_angle_and_latch(self, timestamp_sec: float) -> None:
        if not self.ready or self._gravity_estimate is None or self._reference_gravity is None:
            return

        cosine = min(1.0, max(-1.0, self._dot(self._gravity_estimate, self._reference_gravity)))
        self._angle_deg = math.degrees(math.acos(cosine))

        # 충격 유지 시간에는 상태 전환을 하지 않는다. 이미 검출된 기울기는 유지한다.
        if timestamp_sec < self._impact_until:
            self._above_since = None
            self._below_since = None
            return

        if not self._tilted:
            self._below_since = None
            if self._angle_deg >= self.threshold_deg:
                if self._above_since is None:
                    self._above_since = timestamp_sec
                elif timestamp_sec - self._above_since >= self.confirm_time_sec:
                    self._tilted = True
                    self._above_since = None
            else:
                self._above_since = None
            return

        # 이미 검출된 상태는 release_deg 아래가 일정 시간 유지될 때만 해제한다.
        self._above_since = None
        if self._angle_deg <= self.release_deg:
            if self._below_since is None:
                self._below_since = timestamp_sec
            elif timestamp_sec - self._below_since >= self.release_time_sec:
                self._tilted = False
                self._below_since = None
        else:
            self._below_since = None

    def status(self, timestamp_sec: Optional[float] = None) -> TiltStatus:
        """현재 상태를 반환한다. 기준 자세 전에는 ``ready``가 False다."""
        if timestamp_sec is None:
            timestamp_sec = self._last_timestamp
        if timestamp_sec is None:
            impact_active = False
        else:
            impact_active = timestamp_sec < self._impact_until
        progress = min(1.0, self._calibration_count / self.calibration_samples)
        return TiltStatus(
            ready=self.ready,
            angle_deg=self._angle_deg if self.ready else 0.0,
            tilted=self._tilted if self.ready else False,
            impact_active=impact_active,
            calibration_progress=progress,
            just_calibrated=self._just_calibrated,
        )

