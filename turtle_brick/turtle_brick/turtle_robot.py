import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rcl_interfaces.msg import ParameterDescriptor
from tf2_ros import TransformBroadcaster
from math import pi
from .quaternion import angle_axis_to_quaternion

from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from turtle_brick_interfaces.msg import Tilt
from geometry_msgs.msg import TransformStamped, PoseStamped
from turtlesim.msg import Pose

class Position:
    """ Class for storing the position of the turtle in turtlesim,
        containing x, y, and theta
    """
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

class TurtleRobot(Node):
    """ Central interface between turtlesim and other ROS programs
    """
    def __init__(self):
        # Initialize the node
        super().__init__('turtle_robot')
        # Initialize variables
        self.init_var()
        # This node will use Reentrant Callback Groups for nested services
        self.cbgroup = ReentrantCallbackGroup()

        ###
        ### Parameters
        ###
        # Declare and get the following parameters: platform_height, wheel_radius, max_velocity, gravity_accel
        self.declare_parameter("platform_height", 2,
                               ParameterDescriptor(description="The height between the turtle platform and the ground"))
        self.platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        self.declare_parameter("wheel_radius", 0.2,
                               ParameterDescriptor(description="The proximity in which the turtle needs to be to a waypoint to classify as arrived"))
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.declare_parameter("max_velocity", 50.0,
                               ParameterDescriptor(description="The velocity of the turtle"))
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        self.declare_parameter("gravity_accel", 9.81,
                               ParameterDescriptor(description="The acceleration caused by gravity"))
        self.gravity_accel = self.get_parameter("gravity_accel").get_parameter_value().double_value
        # Declare and get extra parameters
        self.declare_parameter("frequency", 100.0,
                               ParameterDescriptor(description="The frequency in which the msg is published"))
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value

        ###
        ### PUBLISHERS
        ###
        # Create publisher for joint_state
        self.pub_jointstate = self.create_publisher(JointState, 'joint_states', 10)
        # Create publisher for publishing odometry messages
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)

        ###
        ### SUBSCRIBERS
        ###
        # Create subscriber for goal_pose messages
        self.sub_goal = self.create_subscription(PoseStamped, 'goal_pose', self.sub_goal_callback, 10)
        self.sub_goal # Used to prevent warnings
        # Create subscriber for tilt messages
        self.sub_tilt = self.create_subscription(Tilt, 'tilt', self.sub_tilt_callback, 10)
        self.sub_tilt # Used to prevent warnings
        # Create subscriber for getting the turtle's position in turtlesim
        self.sub_pos = self.create_subscription(Pose, 'turtle1/pose', self.sub_pos_callback, 10)
        self.sub_pos # Used to prevent warnings

        ###
        ### BROADCASTER
        ###
        # create the broadcaster for transforms
        self.broadcaster = TransformBroadcaster(self)

        ###
        ### TIMER
        ###
        # Adjusted frequency for whatever the frequency param value is
        timer_period = 1.0/self.frequency  # seconds
        # create timer and timer callback
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.cbgroup)

    def init_var(self):
        """ Initialize all of the turtle_robot node's variables
        """
        # Position and TF
        self.init_odom = False # whether odom Position has been initialized
        self.odom = Position() # starting position/odom/spawn location
        self.old_pos = Position()
        self.new_pos = Position()
        self.transform = Position()

    def timer_callback(self):
        # Initialize the current time for all the tfs
        time = self.get_clock().now().to_msg()
        
        # Create the transform for world -> odom and odom -> base_link
        world_odom_tf = TransformStamped()
        world_odom_tf.header.stamp = time
        world_odom_tf.header.frame_id = "world"
        world_odom_tf.child_frame_id = "odom"
        temp_tf = self.new_transform(Position(), self.odom)
        world_odom_tf = self.update_tf(world_odom_tf, temp_tf)
        self.broadcaster.sendTransform(world_odom_tf)

        odom_base_tf = TransformStamped()
        odom_base_tf.header.stamp = time
        odom_base_tf.header.frame_id = "odom"
        odom_base_tf.child_frame_id = "base_link"
        temp_tf = self.new_transform(self.odom, self.new_pos)
        odom_base_tf = self.update_tf(odom_base_tf, temp_tf)
        odom_base_tf.transform.translation.z = self.wheel_radius * 2 # raise the base_link so that the turtle_robot is standing on the ground
        self.broadcaster.sendTransform(odom_base_tf)
    
    ###
    ### SUBSCRIBER CALLBACKS
    ###
    def sub_goal_callback(self, msg):
        pass

    def sub_tilt_callback(self, msg):
        pass

    def sub_pos_callback(self, msg):
        """ Callback function for the turtle1/pose topic.

            Receives the turtlesim's current position and updates the turtle robot's position.
            If the node has just been initialized (and the turtle has just spawned),
            take note of the starting position.
            
            Args:
                msg (turtlesim/Pose): A message that contains a Pose message, containing the
                    current x, y, theta, and linear and angular velocities of the turtle
        """
        if not self.init_odom:
            self.odom = Position(msg.x, msg.y, msg.theta)
            self.old_pos = Position(msg.x, msg.y, msg.theta)
            self.new_pos = Position(msg.x, msg.y, msg.theta)
            self.init_odom = True
        elif self.init_odom:
            self.old_pos = self.new_pos
            self.new_pos = Position(msg.x, msg.y, msg.theta)

    ###
    ### HELPER FUNCTIONS
    ###
    def new_transform(self, old, new):
        """ Returns a Position that reflects the transform between old and new positions
        """
        tf = Position()
        tf.x = old.x - new.x
        tf.y = old.y - new.y
        tf.theta = old.theta - new.theta
        return tf

    def update_tf(self, input_tf, change):
        """ Returns an updated TransformStamped that reflects the transform/change in position
        """
        tf = input_tf
        tf.transform.translation.x = change.x
        tf.transform.translation.y = change.y
        tf.transform.rotation = angle_axis_to_quaternion(change.theta, [0, 0, -1.0])
        return tf



def turtle_robot_entry(args=None):
    rclpy.init(args=args)
    node = TurtleRobot()
    rclpy.spin(node)
    rclpy.shutdown()
