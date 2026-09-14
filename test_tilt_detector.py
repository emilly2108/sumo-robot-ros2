"""하드웨어 없이 TiltDetector 핵심 동작을 확인하는 단위 테스트.

실행: python3 -m unittest test_tilt_detector.py
"""

import math
import unittest

from tilt_detector import STANDARD_GRAVITY, TiltDetector


class TiltDetectorTest(unittest.TestCase):
    STEP = 0.02

    def feed_accel(self, detector, vector, start, count):
        timestamp = start
        for _ in range(count):
            detector.update_gyro((0.0, 0.0, 0.0), timestamp)
            detector.update_accel(vector, timestamp)
            timestamp += self.STEP
        return timestamp

    def test_detects_stable_twenty_degree_tilt(self):
        detector = TiltDetector(calibration_samples=50)
        timestamp = self.feed_accel(detector, (0.0, STANDARD_GRAVITY, 0.0), 0.0, 50)
        self.assertTrue(detector.status(timestamp).ready)

        angle_rad = math.radians(20.0)
        tilted_gravity = (
            0.0,
            STANDARD_GRAVITY * math.cos(angle_rad),
            STANDARD_GRAVITY * math.sin(angle_rad),
        )
        timestamp = self.feed_accel(detector, tilted_gravity, timestamp, 100)

        status = detector.status(timestamp)
        self.assertTrue(status.tilted)
        self.assertGreaterEqual(status.angle_deg, 15.0)

    def test_rejects_short_high_g_collision(self):
        detector = TiltDetector(calibration_samples=50)
        timestamp = self.feed_accel(detector, (0.0, STANDARD_GRAVITY, 0.0), 0.0, 50)

        # 3 g 충격은 가속도 보정 및 기울기 판정에서 제외되어야 한다.
        detector.update_accel((0.0, STANDARD_GRAVITY * 3.0, 0.0), timestamp)
        status = detector.status(timestamp)
        self.assertTrue(status.impact_active)
        self.assertFalse(status.tilted)
        self.assertLess(status.angle_deg, 1.0)


if __name__ == '__main__':
    unittest.main()

