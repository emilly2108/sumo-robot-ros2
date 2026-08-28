# -*- coding: utf-8 -*-

"""Windows-only green detection preview.

This file does not use ROS2. It shows the RealSense color image, the detected
green area, and the binary mask. Press q or Esc to quit.
"""

import cv2
import numpy as np
import pyrealsense2 as rs

from center_calibration import calibrated_center_x, load_calibration


WIDTH = 424
HEIGHT = 240
FPS = 30
CENTER_X = WIDTH // 2
MIN_CONTOUR_AREA = 30
MIN_VALID_DEPTH_M = 0.15
MAX_VALID_DEPTH_M = 4.0
DEFAULT_RIGHT_BIAS_MM = 32.5

# Keep these values the same as the ROS2 green.py detector.
GREEN_MIN_EXCESS = 25
GREEN_CHANNEL_MARGIN = 10
GREEN_MIN_SATURATION = 35
GREEN_MIN_VALUE = 30


def make_green_mask(color_image):
    """Detect green using Excess Green and channel dominance."""
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
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def draw_detection(color_image, green_mask, depth_image, calibration, fx):
    """Draw the target and the calibrated robot centerline."""
    contours, _ = cv2.findContours(
        green_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    largest_area = 0.0
    centerline_x = CENTER_X
    distance_m = 0.0
    if contours:
        target = max(contours, key=cv2.contourArea)
        largest_area = cv2.contourArea(target)

        if largest_area > MIN_CONTOUR_AREA:
            x, y, w, h = cv2.boundingRect(target)
            cx = x + w // 2
            cy = y + h // 2
            region = green_mask[y:y + h, x:x + w] > 0
            depths = depth_image[y:y + h, x:x + w][region]
            depths = depths[(depths >= MIN_VALID_DEPTH_M) & (depths <= MAX_VALID_DEPTH_M)]
            if depths.size > 0:
                distance_m = float(np.median(depths))
                if calibration is not None:
                    centerline_x = calibrated_center_x(
                        calibration,
                        distance_m,
                        CENTER_X,
                        WIDTH,
                    )
                else:
                    pixel_bias = fx * (DEFAULT_RIGHT_BIAS_MM / 1000.0) / distance_m
                    centerline_x = max(
                        0,
                        min(WIDTH - 1, int(round(CENTER_X + pixel_bias))),
                    )

            cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(color_image, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(
                color_image,
                f'GREEN x={cx} y={cy} D={distance_m:.2f}m',
                (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )
    else:
        cv2.putText(
            color_image,
            'GREEN NOT FOUND',
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
        )

    cv2.line(
        color_image,
        (centerline_x, 0),
        (centerline_x, HEIGHT),
        (255, 255, 255) if calibration is not None and distance_m > 0 else (100, 100, 100),
        1,
    )
    cv2.putText(
        color_image,
        f'CENTER x={centerline_x} ' + (
            'CALIBRATED'
            if calibration is not None
            else f'DEFAULT +{DEFAULT_RIGHT_BIAS_MM:.1f}mm'
        ),
        (8, 45),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
    )

    return largest_area, centerline_x, distance_m


def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    align = rs.align(rs.stream.color)
    calibration = load_calibration(__file__)

    try:
        profile = pipeline.start(config)
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        color_intrinsics = (
            profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        fx = color_intrinsics.fx
        if calibration is None:
            print(
                f'Center calibration not found. '
                f'Using default right bias: +{DEFAULT_RIGHT_BIAS_MM:.1f} mm.'
            )
        else:
            print(f'Center calibration loaded: {calibration["path"]}')
        print('RealSense started. Press q or Esc to quit.')

        while True:
            frames = align.process(pipeline.wait_for_frames())
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
            green_mask = make_green_mask(color_image)
            area, centerline_x, distance_m = draw_detection(
                color_image,
                green_mask,
                depth_image,
                calibration,
                fx,
            )

            mask_view = cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR)
            green_view = cv2.bitwise_and(color_image, color_image, mask=green_mask)
            view = np.hstack((color_image, mask_view, green_view))

            cv2.putText(
                view,
                f'area={area:.0f} center={centerline_x} distance={distance_m:.2f}m',
                (WIDTH + 8, 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )
            cv2.imshow('Green test: original | mask | green only', view)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
