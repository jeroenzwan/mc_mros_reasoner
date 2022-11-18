#!/usr/bin/env python
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from mros2_msgs.action import ControlQos
from diagnostic_msgs.msg import KeyValue


class MockNode(Node):

    def __init__(self):
        super().__init__('mock')

        self._action_client = ActionClient(self, ControlQos, 'mros_objective')

    def start(self):
        self.get_logger().info('Waiting for server')
        self._action_client.wait_for_server()

        goal_msg = ControlQos.Goal()

        goal_msg.qos_expected.objective_type = "f_mock"
        goal_msg.qos_expected.objective_id = "obj_mock_{:.0f}".format(
            self.get_clock().now().to_msg().sec / 10)
        goal_msg.qos_expected.selected_mode = ""
        nfr = KeyValue()
        nfr.key = "mockiness"
        nfr.value = str(0.7)
        goal_msg.qos_expected.qos.append(nfr)

        self.get_logger().info(
            'Sending goal  {0}'.format(
                goal_msg.qos_expected.objective_type))
        self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('Goal Sent!!!')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Best mode: {0}'.format(
                feedback.qos_status.selected_mode))
        self.get_logger().info(
            'Solving: {0} of type {1}'.format(
                feedback.qos_status.objective_id,
                feedback.qos_status.objective_type))
        self.get_logger().info(
            'obj status: {0}'.format(
                feedback.qos_status.objective_status))
        for qos in feedback.qos_status.qos:
            self.get_logger().info(
                'QoS Status: Key: {0} - Value {1}'.format(qos.key, qos.value))


if __name__ == '__main__':
    print("Starting detect pipeline node")

    rclpy.init(args=sys.argv)

    mock_node = MockNode()
    mock_node.start()
    rclpy.spin(mock_node)

    rclpy.shutdown()