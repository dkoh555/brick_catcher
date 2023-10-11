import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from math import pi
from .quaternion import angle_axis_to_quaternion


class TurtleRobot(Node):
    """
    Moves some frames around.

    Static Broadcasts:
       world -> base
    Broadcasts:
       base -> left and base -> right
    """

    def __init__(self):
        super().__init__('turtle_robot')
        self.dx = 10  # used to control frame movement
        # create the broadcaster
        self.broadcaster = TransformBroadcaster(self)
        # Create a timer to do the rest of the transforms
        self.tmr = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        # Now create the transform, noted that it must have a parent frame and a timestamp
        # The header contains the timing information and frame id
        world_base_tf = TransformStamped()
        world_base_tf.header.stamp = self.get_clock().now().to_msg()
        world_base_tf.header.frame_id = "world"
        world_base_tf.child_frame_id = "base_link"

        # get a quaternion corresponding to a rotation by theta about an axis
        degrees = 36 * self.dx
        radians = degrees * pi / 180.0
        
        # The base frame will be raised in the z direction by 1 meter
        # and be aligned with world We are relying on the default values
        # of the transform message (which defaults to no rotation)
        world_base_tf.transform.translation.z = 0.4

        radians = degrees * pi / 180.0
        world_base_tf.transform.rotation = angle_axis_to_quaternion(radians, [0, 0, 1.0])

        self.broadcaster.sendTransform(world_base_tf)

        base_left = TransformStamped()
        base_left.header.frame_id = "base_link"
        base_left.child_frame_id = "left"
        base_left.transform.translation.x = -float(self.dx)
        
        base_left.transform.rotation = angle_axis_to_quaternion(radians, [0, 0, 1.0])

        base_right = TransformStamped()
        base_right.header.frame_id = "base_link"
        base_right.child_frame_id = "right"
        base_right.transform.translation.x = float(self.dx)
        base_right.transform.rotation = angle_axis_to_quaternion(radians, [0, 0, -1.0])

        # don't forget to put a timestamp
        time = self.get_clock().now().to_msg()
        base_right.header.stamp = time
        base_left.header.stamp = time

        self.broadcaster.sendTransform(base_left)
        self.broadcaster.sendTransform(base_right)

        # update the movement
        self.dx -= 0.5
        if self.dx == 0:
            self.dx = 10


def turtle_robot_entry(args=None):
    rclpy.init(args=args)
    node = TurtleRobot()
    rclpy.spin(node)
    rclpy.shutdown()
