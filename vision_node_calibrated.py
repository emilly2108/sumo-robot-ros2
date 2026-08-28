# -*- coding: utf-8 -*-

"""ROS2 entry point for the calibrated green tracker.

Keep this file, green.py, center_calibration.py, and
center_calibration.json in the same directory on the robot.
"""

from green import GreenTrackerDepth, main


__all__ = ['GreenTrackerDepth', 'main']


if __name__ == '__main__':
    main()
