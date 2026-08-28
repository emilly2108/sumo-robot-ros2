# -*- coding: utf-8 -*-

"""Create a camera-to-robot center-line calibration without ROS2.

Place a flat green target on the robot's physical forward centerline. Move it
to several different distances and press Space at each distance. The script
uses the detected pixel center and RealSense depth to save a fitted centerline
to center_calibration.json.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

from center_calibration import calibration_path


WIDTH = 424
HEIGHT = 240
FPS = 30
CENTER_X = WIDTH // 2
MIN_CONTOUR_AREA = 30
MIN_VALID_DEPTH_M = 0.15
MAX_VALID_DEPTH_M = 4.0
DEFAULT_RIGHT_BIAS_MM = 32.5

# Keep these values identical to green_test.py and green.py.
GREEN_MIN_EXCESS = 25
GREEN_CHANNEL_MARGIN = 10
GREEN_MIN_SATURATION = 35
GREEN_MIN_VALUE = 30


def make_green_mask(color_image):
    b, g, r = cv2.split(color_image.astype(np.int16))
    hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]

    excess_green = (2 * g) - r - b
    green_is_dominant = (
        (g >= r + GREEN_CHANNEL_MARGIN)
        & (g >= b + GREEN_CHANNEL_MARGIN)
    )
    green_pixels = (
        (excess_green >= GREEN_MIN_EXCESS)
        & green_is_dominant
        & (saturation >= GREEN_MIN_SATURATION)
        & (value >= GREEN_MIN_VALUE)
    )

    mask = np.zeros(color_image.shape[:2], dtype=np.uint8)
    mask[green_pixels] = 255
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)


def detect_target(green_mask, depth_image):
    contours, _ = cv2.findContours(
        green_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area <= MIN_CONTOUR_AREA:
        return None

    x, y, w, h = cv2.boundingRect(contour)
    cx = x + w // 2
    cy = y + h // 2
    region = green_mask[y:y + h, x:x + w] > 0
    depths = depth_image[y:y + h, x:x + w][region]
    depths = depths[(depths >= MIN_VALID_DEPTH_M) & (depths <= MAX_VALID_DEPTH_M)]
    if depths.size == 0:
        return None

    return {
        'area': area,
        'x': x,
        'y': y,
        'w': w,
        'h': h,
        'cx': cx,
        'cy': cy,
        'distance_m': float(np.median(depths)),
    }


def draw_status(image, target, sample_count, sample_total):
    cv2.line(image, (CENTER_X, 0), (CENTER_X, HEIGHT), (100, 100, 100), 1)
    if target is None:
        cv2.putText(
            image,
            'GREEN TARGET NOT FOUND',
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
        )
    else:
        x, y, w, h = target['x'], target['y'], target['w'], target['h']
        cx, cy = target['cx'], target['cy']
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)
        cv2.putText(
            image,
            f'x={cx} depth={target["distance_m"]:.3f}m',
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )

    cv2.putText(
        image,
        f'Samples: {sample_count}/{sample_total} | SPACE=save  Q=quit',
        (8, HEIGHT - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
    )


def fit_calibration(samples):
    distances = np.array([sample['distance_m'] for sample in samples], dtype=np.float64)
    pixels = np.array([sample['pixel_x'] for sample in samples], dtype=np.float64)
    design = np.column_stack((np.ones(distances.size), 1.0 / distances))
    a, b = np.linalg.lstsq(design, pixels, rcond=None)[0]
    predicted = design @ np.array([a, b])
    rmse_px = float(np.sqrt(np.mean((predicted - pixels) ** 2)))
    return float(a), float(b), rmse_px


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output',
        type=Path,
        default=calibration_path(__file__),
        help='Calibration JSON output path.',
    )
    parser.add_argument('--samples', type=int, default=5)
    parser.add_argument(
        '--right-bias-mm',
        type=float,
        default=DEFAULT_RIGHT_BIAS_MM,
        help='Additional reference-center shift to the right. Positive is right.',
    )
    args = parser.parse_args()

    if args.samples < 3:
        raise ValueError('--samples must be at least 3')

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    align = rs.align(rs.stream.color)
    samples = []

    print('초록색 표적을 로봇의 실제 정면 중심선 위에 놓으세요.')
    print('표적을 서로 다른 거리로 이동한 뒤, 화면에서 SPACE를 누르세요.')
    print('권장 거리: 약 0.5m, 0.8m, 1.0m, 1.5m, 2.0m')

    try:
        profile = pipeline.start(config)
        color_intrinsics = (
            profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        fx = color_intrinsics.fx
        while len(samples) < args.samples:
            frames = align.process(pipeline.wait_for_frames())
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data()).astype(np.float32)
            depth_image *= depth_frame.get_units()
            green_mask = make_green_mask(color_image)
            target = detect_target(green_mask, depth_image)

            view = color_image.copy()
            draw_status(view, target, len(samples), args.samples)
            cv2.imshow('Center calibration', view)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord('q'), 27):
                print('Calibration cancelled.')
                return

            if key == ord(' ') and target is not None:
                sample = {
                    'distance_m': target['distance_m'],
                    'pixel_x': target['cx'],
                    'pixel_y': target['cy'],
                    'area': target['area'],
                }
                samples.append(sample)
                print(
                    f'{len(samples)}/{args.samples}: '
                    f'x={sample["pixel_x"]}, distance={sample["distance_m"]:.3f}m'
                )

        distances = [sample['distance_m'] for sample in samples]
        if max(distances) - min(distances) < 0.10:
            raise RuntimeError('거리 변화가 너무 작습니다. 표적을 더 멀리 이동해 다시 측정하세요.')

        base_a, base_b, rmse_px = fit_calibration(samples)
        bias_m = args.right_bias_mm / 1000.0
        b = base_b + fx * bias_m
        result = {
            'version': 1,
            'a': base_a,
            'b': b,
            'base_a': base_a,
            'base_b': base_b,
            'right_bias_mm': float(args.right_bias_mm),
            'fx': float(fx),
            'rmse_px': rmse_px,
            'image_width': WIDTH,
            'image_height': HEIGHT,
            'samples': samples,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, ensure_ascii=False, indent=2),
            encoding='utf-8',
        )
        print(f'Calibration saved: {args.output.resolve()}')
        print(
            f'center_x(distance) = {base_a:.3f} + {b:.3f} / distance_m '
            f'(right bias={args.right_bias_mm:+.1f} mm)'
        )
        print(f'fit RMSE = {rmse_px:.2f} pixels')

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
