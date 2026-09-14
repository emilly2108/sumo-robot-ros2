"""ROS2 없이 D435i의 기울기를 감시하는 순수 Python 프로그램.

실행:
    python3 d435i_tilt_monitor.py

시작 뒤 로봇을 기준 자세로 약 2초간 정지시킨다. 이후 15도 이상이 0.25초
지속될 때만 ``TILT DETECTED``와 각도를 출력한다. 15도 미만에서는 각도를
출력하지 않는다. Ctrl+C로 종료한다.
"""

from __future__ import annotations

import argparse
import json
import time
from typing import List, Tuple

import pyrealsense2 as rs

from tilt_detector import TiltDetector, TiltStatus


ImuSample = Tuple[float, str, Tuple[float, float, float]]


def collect_imu_samples(frames) -> List[ImuSample]:
    """한 frameset 안의 gyro/accel 샘플을 시간 순서대로 반환한다."""
    samples: List[ImuSample] = []

    gyro_frame = frames.first_or_default(rs.stream.gyro)
    if gyro_frame:
        gyro = gyro_frame.as_motion_frame().get_motion_data()
        samples.append(
            (
                gyro_frame.get_timestamp() / 1000.0,
                'gyro',
                (gyro.x, gyro.y, gyro.z),
            )
        )

    accel_frame = frames.first_or_default(rs.stream.accel)
    if accel_frame:
        accel = accel_frame.as_motion_frame().get_motion_data()
        samples.append(
            (
                accel_frame.get_timestamp() / 1000.0,
                'accel',
                (accel.x, accel.y, accel.z),
            )
        )

    return sorted(samples)


def process_samples(detector: TiltDetector, samples: List[ImuSample]) -> TiltStatus:
    """D435i 타임스탬프 순서로 자세 필터를 갱신한다."""
    status = detector.status()
    for timestamp, stream_name, values in samples:
        if stream_name == 'gyro':
            status = detector.update_gyro(values, timestamp)
        else:
            status = detector.update_accel(values, timestamp)
    return status


def status_payload(status: TiltStatus) -> dict:
    """15도 미만의 각도는 의도적으로 출력하지 않는다."""
    return {
        'ready': status.ready,
        'tilt_detected': status.tilted,
        'tilt_angle_deg': round(status.angle_deg, 2) if status.tilted else None,
        'impact_active': status.impact_active,
    }


def print_status(status: TiltStatus, json_output: bool) -> None:
    if json_output:
        print(json.dumps(status_payload(status), ensure_ascii=False), flush=True)
        return

    if not status.ready:
        print(
            f'Calibrating reference pose: {status.calibration_progress * 100:.0f}%',
            flush=True,
        )
    elif status.impact_active:
        print('IMPACT: acceleration correction paused', flush=True)
    elif status.tilted:
        print(f'TILT DETECTED: {status.angle_deg:.1f} deg', flush=True)
    else:
        print('READY: below 15 deg', flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='D435i IMU 15-degree tilt monitor')
    parser.add_argument('--threshold', type=float, default=15.0, help='detect threshold in degrees')
    parser.add_argument('--release', type=float, default=12.0, help='release threshold in degrees')
    parser.add_argument('--confirm-sec', type=float, default=0.25, help='minimum detect duration')
    parser.add_argument('--json', action='store_true', help='write one JSON status per state change')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    detector = TiltDetector(
        threshold_deg=args.threshold,
        release_deg=args.release,
        confirm_time_sec=args.confirm_sec,
        release_time_sec=0.25,
        impact_hold_sec=0.30,
        accel_norm_tolerance_g=0.20,
        # D435i 가속도계 250 Hz 기준 약 2초 동안 기준 중력 방향을 평균낸다.
        calibration_samples=500,
    )

    pipeline = rs.pipeline()
    config = rs.config()
    # FPS/포맷을 고정하지 않는다. 연결된 D435i와 현재 펌웨어가 제공하는
    # 가속도계·자이로 프로파일을 librealsense가 자동 선택하게 한다.
    config.enable_stream(rs.stream.accel)
    config.enable_stream(rs.stream.gyro)

    try:
        pipeline.start(config)
    except Exception as error:
        raise SystemExit(
            f'Could not start D435i IMU: {error}\n'
            'Run "python d435i_imu_diagnose.py" to verify the connected model and IMU profiles.\n'
            'Also close every other program that uses the RealSense camera.'
        ) from error

    print('D435i IMU started. Keep the robot still in its reference pose until calibration ends.')
    previous_state = None
    last_calibration_print = 0.0
    last_tilt_print = 0.0

    try:
        while True:
            frames = pipeline.wait_for_frames()
            status = process_samples(detector, collect_imu_samples(frames))
            state = (status.ready, status.tilted, status.impact_active)

            # 보정 중 진행률만 주기적으로 표시하고, 보정 뒤에는 상태가 바뀔 때만 출력한다.
            now = time.monotonic()
            if not status.ready:
                if now - last_calibration_print >= 0.25:
                    print_status(status, args.json)
                    last_calibration_print = now
                continue

            # JSON 모드에서는 상태 전환만 출력한다. 일반 모드에서는 기울어진 동안
            # 각도를 확인할 수 있도록 10 Hz 이하로만 추가 출력한다.
            state_changed = state != previous_state
            should_report_angle = (
                not args.json
                and status.tilted
                and now - last_tilt_print >= 0.10
            )
            if state_changed or should_report_angle:
                print_status(status, args.json)
                previous_state = state
                if status.tilted:
                    last_tilt_print = now
    except KeyboardInterrupt:
        print('\nStopped.')
    finally:
        pipeline.stop()


if __name__ == '__main__':
    main()

