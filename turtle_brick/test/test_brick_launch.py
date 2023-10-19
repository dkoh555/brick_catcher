import unittest
import pytest
import rclpy
from launch import LaunchDescription
from launch_testing.actions import ReadyToTest
from launch_ros.actions import Node
from geometry_msgs.msg import Twist
import time

# Mark the launch description generation as a rostest
# Also, it's called generate_test_description() for a test
# But it still returns a LaunchDescription


@pytest.mark.rostest
def generate_test_description():
    # Create a node, as usual. But it's useful to keep it around
    node = Node(package="turtle_brick", executable="turtle_robot", name="turtle_robot")
    # return a tuple (LaunchDescription, extra_dict)
    # extra_dict is a dictionary of parameter names -> values that get passed to
    # test cases. Experiment a little, I'm not sure exactly how it works...
    return (
        LaunchDescription(
            [
                node,
                ReadyToTest(),  # this is the last action. Can be used elsewhere somehow
            ]
        ),
        {
            "turtle_robot": node
        },  # this is a way to pass the node action to the test case
    )


# The above returns the launch description. Now it's time for the test
# The goal is essentially to create a node that can then be used in all tests to
# call services and subscribe/publish messages
# unlike a regular node, it is often useful to not subclass node but rather
# just create it. It is also useful (and necessary) to spin_once() as needed


class TestTurtleBrick(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Runs one time, when the testcase is loaded"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Runs one time, when testcase is unloaded"""
        rclpy.shutdown()

    def setUp(self):
        """Runs before every test"""
        # so before every test, we create a new node
        self.node = rclpy.create_node("test_node")

    def tearDown(self):
        """Runs after every test"""
        # so after every test we destroy the node
        # Is a new node necessary for each test, or could we
        # create the nodes when we setupClass?
        self.node.destroy_node()

    def test_frequency(self, launch_service, turtle_robot, proc_output):
        """In UnitTest, any function starting with "test" is run as a test

        Args:
           launch_service - information about the launch
           myaction - this was passed in when we created the description
           proc_output - this object streams the output (if any) from the running process
        """
        # Create subscriber for cmd_vel messages
        self.sub_cmdvel = self.node.create_subscription(
            Twist, "turtle1/cmd_vel", self.sub_cmdvel_callback, 10
        )
        self.sub_cmdvel  # Used to prevent warnings
        self.start = time.time()
        self.end = time.time()
        self.count = 0
        while (self.end - self.start) < 10:  # Measuring a time period of ~10 seconds
            rclpy.spin_once(self.node)
        freq = self.count / (self.end - self.start)
        tol = 10
        assert 100 - round(freq) < tol

    def sub_cmdvel_callback(self, msg):
        self.end = time.time()
        self.count += 1
