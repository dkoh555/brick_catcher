import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros import TransformBroadcaster
import math
from .quaternion import angle_axis_to_quaternion

from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from turtle_brick_interfaces.msg import Tilt
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
from turtlesim.msg import Pose

from enum import Enum

class arena(Node):
    """ Node for simulating the environment in rviz
    """
    def __init__(self):
        pass


def arena_entry(args=None):
    rclpy.init(args=args)
    node = arena()
    rclpy.spin(node)
    rclpy.shutdown()