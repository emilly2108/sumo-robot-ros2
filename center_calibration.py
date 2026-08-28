# -*- coding: utf-8 -*-

"""Shared loading and projection helpers for the center-line calibration."""

import json
from pathlib import Path


CALIBRATION_FILENAME = 'center_calibration.json'


def calibration_path(script_file):
    return Path(script_file).resolve().with_name(CALIBRATION_FILENAME)


def load_calibration(script_file):
    """Load the calibration next to a script, or return None if unavailable."""
    path = calibration_path(script_file)
    if not path.exists():
        return None

    try:
        data = json.loads(path.read_text(encoding='utf-8'))
        a = float(data['a'])
        b = float(data['b'])
        # Files created before the explicit right-bias option must be
        # regenerated so an old centerline is not mistaken for the new one.
        right_bias_mm = float(data['right_bias_mm'])
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
        return None

    return {
        'a': a,
        'b': b,
        'right_bias_mm': right_bias_mm,
        'path': str(path),
        'samples': data.get('samples', []),
        'rmse_px': data.get('rmse_px'),
    }


def calibrated_center_x(calibration, distance_m, default_x, width):
    """Project the calibrated robot centerline into the color image."""
    if calibration is None or distance_m <= 0.0:
        return int(default_x)

    center_x = calibration['a'] + calibration['b'] / distance_m
    return max(0, min(width - 1, int(round(center_x))))
