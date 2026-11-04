"""Offline HFSM priority checks. No ROS or motor hardware is started."""

import importlib
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import MagicMock, patch


class Message:
    def __init__(self, data=None):
        self.data = data


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0)
        self.angular = types.SimpleNamespace(z=0.0)


class NodeStub:
    def __init__(self, name):
        self.seconds = 100.0
        self.logger = MagicMock()

    def get_clock(self):
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(nanoseconds=round(self.seconds * 1e9))
        )

    def get_logger(self):
        return self.logger

    def create_publisher(self, *args):
        return MagicMock()

    def create_subscription(self, *args):
        return MagicMock()

    def create_service(self, *args):
        return MagicMock()

    def create_timer(self, *args):
        return MagicMock()


package = types.ModuleType("priority_test_hfsm")
package.__path__ = [str(Path(__file__).resolve().parents[1] / "hfsm_brain")]
with patch.dict(sys.modules, {
    "priority_test_hfsm": package,
    "rclpy": types.ModuleType("rclpy"),
    "rclpy.node": types.SimpleNamespace(Node=NodeStub),
    "geometry_msgs.msg": types.SimpleNamespace(Twist=Twist),
    "std_msgs.msg": types.SimpleNamespace(Bool=Message, Float32=Message, String=Message),
    "std_srvs.srv": types.SimpleNamespace(
        SetBool=types.SimpleNamespace(Request=object, Response=object)
    ),
}):
    brain = importlib.import_module("priority_test_hfsm.node")


class PriorityTests(unittest.TestCase):
    def setUp(self):
        self.node = brain.HFSMBrainNode()
        self.node.switch_enabled = True
        self.node.active_action = brain.Cruise_Action(self.node.config)

    def colors(self, right, left, back=0):
        world = self.node.world
        world.sensor1 = brain.Sensor_Color(right)
        world.sensor2 = brain.Sensor_Color(left)
        world.sensor3 = brain.Sensor_Color(back)
        world.sensor_event = brain.SENSOR_EVENTS.get(
            (world.sensor1, world.sensor2, world.sensor3), brain.Sensor_Event.UNKNOWN
        )

    def wall(self):
        self.node.world.wall_left_distance = 0.2
        self.node.world.wall_right_distance = 0.2
        self.node.world.wall_last_received_at = self.node.seconds

    def wall_pair(self, left, right):
        self.node.wall_left_callback(Message(left))
        self.node.wall_right_callback(Message(right))

    def tick(self):
        self.node.control_loop()
        return self.node.pub_vel.publish.call_args.args[0]

    def start_wall_turn(self):
        action = brain.Wall_Avoid_Action(self.node.config, 1.0)
        action.enter(self.node.seconds, self.node.world)
        self.node.active_action = action
        self.wall()
        self.assertEqual(self.tick().angular.z, 0.0)
        self.node.seconds += self.node.config.pre_turn_stop_duration
        self.wall()
        self.assertEqual(self.tick().angular.z, self.node.config.turn_speed)
        return action

    def test_front_blue_finishes_reverse_pause_and_turn_before_wall(self):
        self.colors(2, 2)
        self.wall()
        self.assertEqual(self.tick().linear.x, self.node.config.reverse_speed)
        self.assertIsInstance(self.node.active_action, brain.Front_Color_Avoid_Action)

        self.node.seconds = 100.05
        self.colors(0, 0)
        self.wall()
        self.assertEqual(self.tick().angular.z, 0.0)
        self.assertEqual(self.node.last_command_state[1], "FRONT_BLUE_PRE_TURN_STOP")

        self.node.seconds = 100.05 + self.node.config.pre_turn_stop_duration
        self.wall()
        self.assertEqual(self.tick().angular.z, self.node.config.turn_speed)
        self.assertIsInstance(self.node.active_action, brain.Front_Color_Avoid_Action)

        self.node.seconds += self.node.config.turn_90_duration + 0.001
        self.wall()
        self.assertEqual(self.tick().angular.z, 0.0)
        self.assertIsInstance(self.node.active_action, brain.Wall_Avoid_Action)

    def test_all_blue_without_green_starts_front_recovery(self):
        self.colors(2, 2, 2)
        self.wall()
        self.assertEqual(self.tick().linear.x, self.node.config.reverse_speed)
        self.assertIsInstance(self.node.active_action, brain.Front_Color_Avoid_Action)

    def test_front_red_and_all_red_precede_wall(self):
        for back in (0, 1):
            with self.subTest(back=back):
                self.setUp()
                self.colors(1, 1, back)
                self.wall()
                self.assertEqual(self.tick().linear.x, self.node.config.reverse_speed)
                self.assertIsInstance(self.node.active_action, brain.Front_Color_Avoid_Action)

    def test_blue_preempts_active_wall(self):
        action = brain.Wall_Avoid_Action(self.node.config, 1.0)
        action.enter(self.node.seconds, self.node.world)
        self.node.active_action = action
        self.colors(2, 2)
        self.wall()
        self.assertEqual(self.tick().linear.x, self.node.config.reverse_speed)
        self.assertIsInstance(self.node.active_action, brain.Front_Color_Avoid_Action)

    def test_rear_color_precedes_wall_until_it_clears(self):
        self.colors(0, 0, 2)
        self.wall()
        self.assertEqual(self.tick().linear.x, self.node.config.max_speed)
        self.assertIsInstance(self.node.active_action, brain.Back_Single_Boost_Action)
        self.colors(0, 0)
        self.wall()
        self.assertEqual(self.tick().angular.z, 0.0)
        self.assertIsInstance(self.node.active_action, brain.Wall_Avoid_Action)

    def test_green_color_rule_precedes_wall(self):
        self.colors(2, 2)
        self.node.world.target_found = True
        self.node.world.target_distance = 0.2
        self.wall()
        self.assertEqual(self.tick().linear.x, self.node.config.reverse_speed)
        self.assertIsInstance(self.node.active_action, brain.Green_Close_Front_Color_Action)

    def test_far_green_turn_starts_at_one_point_five_meters(self):
        self.node.world.target_found = True
        for distance, expected in (
            (1.49, brain.Green_Follow_Action),
            (1.50, brain.Far_Green_Turn_Action),
        ):
            with self.subTest(distance=distance):
                self.node.world.target_distance = distance
                self.assertIsInstance(self.node.choose_green_action(), expected)

    def test_green_preempts_no_green_front_recovery(self):
        self.colors(2, 2)
        self.wall()
        self.tick()
        self.assertIsInstance(self.node.active_action, brain.Front_Color_Avoid_Action)
        self.node.world.target_found = True
        self.node.world.target_distance = 0.2
        self.assertEqual(self.tick().linear.x, self.node.config.reverse_speed)
        self.assertIsInstance(self.node.active_action, brain.Green_Close_Front_Color_Action)

    def test_wall_turn_finishes_into_cruise_without_post_turn_wait(self):
        self.start_wall_turn()
        self.node.seconds += self.node.config.turn_90_duration + 0.001
        self.node.world.wall_left_distance = 9.9
        self.node.world.wall_right_distance = 9.9
        self.node.world.wall_last_received_at = self.node.seconds
        command = self.tick()
        self.assertIsInstance(self.node.active_action, brain.Cruise_Action)
        self.assertEqual(command.linear.x, self.node.config.base_speed)
        self.assertEqual(self.node.last_command_state[1], "CRUISE")

    def test_wall_turn_finishes_into_color_recovery_immediately(self):
        self.start_wall_turn()
        self.node.seconds += self.node.config.turn_90_duration + 0.001
        self.colors(2, 2)
        self.wall()
        command = self.tick()
        self.assertIsInstance(self.node.active_action, brain.Front_Color_Avoid_Action)
        self.assertEqual(command.linear.x, self.node.config.reverse_speed)

    def test_persistent_wall_restarts_only_after_three_post_turn_pairs(self):
        previous = self.start_wall_turn()
        self.wall_pair(0.1, 0.1)
        self.assertEqual(self.node.world.wall_left_distance, 0.2)
        self.node.seconds += self.node.config.turn_90_duration + 0.001
        command = self.tick()
        self.assertIsInstance(self.node.active_action, brain.Cruise_Action)
        self.assertEqual(command.linear.x, self.node.config.base_speed)
        self.assertEqual(self.node.world.wall_left_distance, self.node.config.no_wall_distance)

        for remaining in (2, 1):
            self.node.seconds += self.node.config.control_period
            self.wall_pair(0.2, 0.2)
            self.assertEqual(self.node.wall_recheck_pairs_remaining, remaining)
            self.assertEqual(self.tick().linear.x, self.node.config.base_speed)

        self.node.seconds += self.node.config.control_period
        self.node.wall_left_callback(Message(0.2))
        self.assertEqual(self.node.wall_recheck_pairs_remaining, 1)
        self.node.wall_right_callback(Message(0.2))
        command = self.tick()
        self.assertIsInstance(self.node.active_action, brain.Wall_Avoid_Action)
        self.assertIsNot(self.node.active_action, previous)
        self.assertEqual((command.linear.x, command.angular.z), (0.0, 0.0))
        self.assertEqual(self.node.last_command_state[1], "WALL_AVOID_PRE_TURN_STOP")

    def test_clear_post_turn_pairs_do_not_restart_wall_turn(self):
        self.start_wall_turn()
        self.node.seconds += self.node.config.turn_90_duration + 0.001
        self.assertEqual(self.tick().linear.x, self.node.config.base_speed)

        for remaining in (2, 1, 0):
            self.node.seconds += self.node.config.control_period
            self.wall_pair(9.9, 9.9)
            self.assertEqual(self.node.wall_recheck_pairs_remaining, remaining)
            self.assertEqual(self.tick().linear.x, self.node.config.base_speed)
            self.assertIsInstance(self.node.active_action, brain.Cruise_Action)

    def test_green_interrupts_wall_turn(self):
        self.start_wall_turn()
        self.node.world.target_found = True
        self.node.world.target_distance = 0.6
        self.wall()
        self.assertEqual(self.tick().linear.x, self.node.config.chase_speed)
        self.assertIsInstance(self.node.active_action, brain.Green_Follow_Action)

    def test_switch_off_remains_highest_priority(self):
        self.node.switch_enabled = False
        self.colors(2, 2)
        self.wall()
        command = self.tick()
        self.assertEqual((command.linear.x, command.angular.z), (0.0, 0.0))
        self.assertEqual(self.node.last_command_state[1], "SWITCH_OFF")


if __name__ == "__main__":
    unittest.main()
