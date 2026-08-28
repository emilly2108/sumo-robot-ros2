# -*- coding: utf-8 -*-
"""Choose exactly one brain-function test and run this file."""

import rclpy

from brain_test_front_sensor import FrontSensorTestNode
from brain_test_green_distance import GreenDistanceTestNode
from brain_test_green_follow import GreenFollowTestNode
from brain_test_opening import OpeningTestNode
from brain_test_rear_sensor import RearSensorTestNode
from brain_test_wall_avoid import WallAvoidTestNode


# Change only this line before each test.
TEST_MODE = 'green_follow'

TEST_NODES = {
    'green_follow': GreenFollowTestNode,
    'green_distance': GreenDistanceTestNode,
    'opening': OpeningTestNode,
    'wall_avoid': WallAvoidTestNode,
    'front_sensor': FrontSensorTestNode,
    'rear_sensor': RearSensorTestNode,
}


def main(args=None):
    node_class = TEST_NODES.get(TEST_MODE)
    if node_class is None:
        allowed = ', '.join(TEST_NODES)
        raise ValueError(f'Unknown TEST_MODE: {TEST_MODE}. Use one of: {allowed}')

    rclpy.init(args=args)
    node = node_class()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
