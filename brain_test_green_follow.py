# -*- coding: utf-8 -*-
"""Test only green-target following from vision_node.py."""

from brain_test_common import BrainTestBase, run_node


class GreenFollowTestNode(BrainTestBase):
    def __init__(self):
        super().__init__('brain_test_green_follow')
        self.start_control_timer()
        self.get_logger().info('TEST MODE: green_follow')

    def extra_log_text(self):
        return ' | mode=GREEN_FOLLOW' if self.target_found else ' | mode=GREEN_SEARCH'

    def control_loop(self):
        if self.green_data_is_stale():
            self.reset_green_target()

        if self.target_found:
            target_v, target_w = self.compute_green_follow_command()
        else:
            target_v, target_w = 0.0, 0.8
        self.publish_smoothed_command(target_v, target_w)


if __name__ == '__main__':
    run_node(GreenFollowTestNode)
