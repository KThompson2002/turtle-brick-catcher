"""Test that the turtle_robot node publishes cmd_vel commands at 100 Hz."""
import time
import unittest

from geometry_msgs.msg import Twist

from launch import LaunchDescription

from launch_ros.actions import Node

from launch_testing.actions import ReadyToTest

from launch_testing_ros import WaitForTopics

import pytest


import rclpy


@pytest.mark.rostest
def generate_test_description():
    """Launch the node."""
    node = Node(
        package='turtle_brick',
        executable='turtle_robot',
        remappings=[
            ('/cmd_vel', '/turtle1/cmd_vel')
        ]
    )
    # Here is a dictionary of additional arguments to pass to test functions
    # You could, for example, store the Node Action
    # in a variable and pass it to the test
    # functions.
    return (
        LaunchDescription([
            node,
            ReadyToTest()
            ]),
        {'myaction': node}
            )
# The above returns the launch description. Now it's time for the test
# The goal is essentially to create a node that can then be used in all tests
# to call services and subscribe/publish messages
# unlike a regular node, it is often useful to not subclass node but rather
# just create it. It is also useful (and necessary) to spin_once() as needed


class TestTurtleRobot(unittest.TestCase):
    """Test the TurtleRobot."""

    @classmethod
    def setUpClass(cls):
        """Run one time, when the testcase is loaded."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Run one time, when testcase is unloaded."""
        rclpy.shutdown()

    def setUp(self):
        """Run before every test."""
        # so before every test, we create a new node
        self.node = rclpy.create_node('test_turtle_bot_node')
        self.times_recived = 0

        self.cmd_vel_sub = self.node.create_subscription(
            Twist,
            'turtle1/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        """Count the cmd_vel publishing."""
        self.times_recived += 1

    def tearDown(self):
        """Run after every test."""
        # so after every test we destroy the node
        # Is a new node necessary for each test, or could we
        # create the nodes when we setupClass?
        self.node.destroy_node()

    def test_pub_freq(self):
        """Check if the the frequency is the right one (100 Hz)."""
        wait_cmd_vel = WaitForTopics(
            [('turtle1/cmd_vel', Twist)], timeout=10.0
        )
        assert wait_cmd_vel.wait()

        start_time = time.time()
        time_duration = 4.0

        while (time.time() < start_time + time_duration):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        expected_frequency = 100.0  # Hz
        received_frequency = self.times_recived / (time_duration)
        print(received_frequency)
        self.assertAlmostEqual(
            received_frequency, expected_frequency, delta=10.0
        )
