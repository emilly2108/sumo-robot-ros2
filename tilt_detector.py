from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple


Vector3 = Tuple[float, float, float]
STANDARD_GRAVITY = 9.80665


@dataclass(frozen=True)
class TiltStatus:
    ready: bool
    angle_deg: float
    tilted: bool
    impact_active: bool
    calibration_progress: float
    just_calibrated: bool = False


class TiltDetector:

    def __init__(
        self,
        threshold_deg: float = 15.0,  # 기울어짐을 시작으로 판정할 각도
        release_deg: float = 12.0,  # 기울어짐 판정을 해제할 더 낮은 각도
        confirm_time_sec: float = 0.25,  # 임계각 이상이 계속돼야 하는 시간
        release_time_sec: float = 0.25,  # 해제각 이하가 계속돼야 하는 시간이
        calibration_samples: int = 50,  # 기준 자세 평균에 사용할 정상 가속도 샘플 수
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
        self.calibration_samples = calibration_samples

        self._reference_gravity: Optional[Vector3] = None
        # 자이로 예측과 가속도 보정을 합친 현재 단위 중력 방향 추정값
        self._gravity_estimate: Optional[Vector3] = None
        # 초기 보정 동안 받은 단위 중력 방향들의 누적합
        self._calibration_sum: Vector3 = (0.0, 0.0, 0.0)
        # 초기 보정에 실제로 더한 정상 샘플 수
        self._calibration_count = 0
        # 가속도계 샘플의 시간 역행과 보정 간격 계산을 위한 마지막 시각
        self._last_accel_timestamp: Optional[float] = None
        # 자이로 샘플의 시간 역행과 적분 간격 계산을 위한 마지막 시각
        self._last_gyro_timestamp: Optional[float] = None
        # status()가 시각을 받지 않았을 때 사용할 가장 최근 IMU 샘플 시각
        self._last_timestamp: Optional[float] = None
        # 이 시각 전까지는 충격 중이므로 기울기 상태를 새로 전환하지 않음
        self._impact_until = float("-inf")
        # 임계각 이상이 처음 이어지기 시작한 시각
        self._above_since: Optional[float] = None
        # 해제각 이하가 처음 이어지기 시작한 시각
        self._below_since: Optional[float] = None
        # 시간 조건까지 충족해 확정된 현재 기울어짐 상태
        self._tilted = False
        # 기준 중력 방향과 추정 중력 방향의 현재 각도
        self._angle_deg = 0.0
        # status() 호출자에게 초기 보정 완료 직후 한 번만 알릴 표시
        self._just_calibrated = False

    @property
    def ready(self) -> bool:
        return self._reference_gravity is not None

    @staticmethod
    def _add(left: Vector3, right: Vector3) -> Vector3:
        # 두 3차원 벡터의 같은 축 성분을 더해 새 벡터를 만든다.
        return (left[0] + right[0], left[1] + right[1], left[2] + right[2])

    @staticmethod
    def _scale(vector: Vector3, value: float) -> Vector3:
        return (vector[0] * value, vector[1] * value, vector[2] * value)

    @staticmethod
    def _dot(left: Vector3, right: Vector3) -> float:
        # 내적구하기
        return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]

    @staticmethod
    def _cross(left: Vector3, right: Vector3) -> Vector3:
        # 외적구하기
        return (
            left[1] * right[2] - left[2] * right[1],
            left[2] * right[0] - left[0] * right[2],
            left[0] * right[1] - left[1] * right[0],
        )

    @classmethod
    #자이로 측정 함수
    def _normalize(cls, vector: Vector3) -> Optional[Vector3]:
        length_sq = cls._dot(vector, vector)
        if length_sq <= 1e-12:
            return None
        return cls._scale(vector, 1.0 / math.sqrt(length_sq))

    def update_gyro(self, gyro_rad_s: Vector3, timestamp_sec: float) -> TiltStatus:
        self._just_calibrated = False
        self._last_timestamp = timestamp_sec

        if self._last_gyro_timestamp is not None and timestamp_sec <= self._last_gyro_timestamp:
            return self.status(timestamp_sec)

        if self._gravity_estimate is not None and self._last_gyro_timestamp is not None:
            dt = timestamp_sec - self._last_gyro_timestamp
            if 0.0 < dt <= 0.10:
                derivative = self._scale(self._cross(gyro_rad_s, self._gravity_estimate), -1.0)
                predicted = self._add(self._gravity_estimate, self._scale(derivative, dt))
                normalized = self._normalize(predicted)
                if normalized is not None:
                    self._gravity_estimate = normalized
                    self._update_angle_and_latch(timestamp_sec)

        self._last_gyro_timestamp = timestamp_sec
        return self.status(timestamp_sec)

    def update_accel(self, accel_m_s2: Vector3, timestamp_sec: float) -> TiltStatus:
        self._just_calibrated = False
        self._last_timestamp = timestamp_sec

        if self._last_accel_timestamp is not None and timestamp_sec <= self._last_accel_timestamp:
            return self.status(timestamp_sec)

        norm = math.sqrt(self._dot(accel_m_s2, accel_m_s2))
        if abs(norm - STANDARD_GRAVITY) > 0.20 * STANDARD_GRAVITY:
            self._impact_until = max(self._impact_until, timestamp_sec + 0.30)
            return self.status(timestamp_sec)

        measured_gravity = self._normalize(accel_m_s2)
        if measured_gravity is None:
            self._impact_until = max(self._impact_until, timestamp_sec + 0.30)
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
            alpha = 1.0 - math.exp(-dt / 0.20)
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
