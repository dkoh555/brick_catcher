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

class state(Enum):
    """ Current state of the system (turtle_robot node).
        Determines what movement commands are published to the turtle robot,
        whether it is MOVING or STOPPED
    """
    MOVING = 0
    STOPPED = 1

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
        ### PARAMETERS
        ###
        # Declare and get the following parameters: platform_height, wheel_radius, max_velocity, gravity_accel
        self.declare_parameter("platform_height", 2.0,
                               ParameterDescriptor(description="The height between the turtle platform and the ground"))
        self.platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        self.declare_parameter("wheel_radius", 0.2,
                               ParameterDescriptor(description="The proximity in which the turtle needs to be to a waypoint to classify as arrived"))
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.declare_parameter("max_velocity", 5.0,
                               ParameterDescriptor(description="The velocity of the turtle"))
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        self.declare_parameter("gravity_accel", 9.81,
                               ParameterDescriptor(description="The acceleration caused by gravity"))
        self.gravity_accel = self.get_parameter("gravity_accel").get_parameter_value().double_value
        # Declare and get extra parameters
        self.declare_parameter("frequency", 100.0,
                               ParameterDescriptor(description="The frequency in which the msg is published"))
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.declare_parameter("tolerance", 0.1,
                               ParameterDescriptor(description="The proximity in which the turtle needs to be to a waypoint to classify as arrived"))
        self.tolerance = self.get_parameter("tolerance").get_parameter_value().double_value

        ###
        ### PUBLISHERS
        ###
        # Create publisher for joint_state
        self.pub_jointstate = self.create_publisher(JointState, 'joint_states', 10)
        # Create publisher for publishing odometry messages
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        # Create publisher to move the turtle
        self.pub_cmdvel = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

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
        # State of the node
        self.state = state.STOPPED
        # Position and TF
        self.init_odom = False # whether odom Position has been initialized
        self.odom = Position() # starting position/odom/spawn location
        self.old_pos = Position()
        self.new_pos = Position()
        self.transform = Position()
        # Joint States
        self.platform_angle = 0.0
        self.wheel_angle = 0.0
        # Goal/Target Position
        self.target_pos = Position()

    def timer_callback(self):
        # Initialize the current time
        time = self.get_clock().now().to_msg()

        # Declaring all the import variables/transforms/joint states
        move_msg = Twist()
        world_odom_tf = TransformStamped()
        odom_base_tf = TransformStamped()
        joint_state = JointState()

        # If in MOVING state, move the turtle robot to its goal
        if self.state == state.MOVING:
            move_msg = self.nav_to_goal(move_msg, self.target_pos)
            # Rotate the wheel
            self.wheel_angle -= 0.1
            if self.wheel_angle <= -math.pi:
                self.wheel_angle = math.pi
        
        # Create the transform for world -> odom and odom -> base_link
        world_odom_tf.header.stamp = time
        world_odom_tf.header.frame_id = "world"
        world_odom_tf.child_frame_id = "odom"
        temp_tf = self.new_transform(Position(), self.odom)
        world_odom_tf = self.update_tf(world_odom_tf, temp_tf)
        self.broadcaster.sendTransform(world_odom_tf)

        odom_base_tf.header.stamp = time
        odom_base_tf.header.frame_id = "odom"
        odom_base_tf.child_frame_id = "base_link"
        temp_tf = self.new_transform(self.odom, self.new_pos)
        odom_base_tf = self.update_tf(odom_base_tf, temp_tf)
        odom_base_tf.transform.translation.z = self.wheel_radius * 2 # raise the base_link so that the turtle_robot is standing on the ground
        self.broadcaster.sendTransform(odom_base_tf)

        # Publishing the joint state
        joint_state.header = Header()
        joint_state.header.stamp.sec = time.sec
        joint_state.header.stamp.nanosec = time.nanosec
        joint_state.name = ['platform_joint', 'stem_joint', 'wheel_joint']
        joint_state.position = [self.platform_angle, self.target_pos.theta, self.wheel_angle]
        self.pub_jointstate.publish(joint_state)

        # Publish the movement command for the turtlesim
        self.pub_cmdvel.publish(move_msg)

    
    ###
    ### SUBSCRIBER CALLBACKS
    ###
    def sub_goal_callback(self, msg):
        """ Callback function for the goal_pose topic.

            Receives a goal position for the turtle robot to move towards,
            hence it also changes the state to MOVING
            
            Args:
                msg (geometry_msgs/PoseStamped): A message that contains a PoseStamped message, containing the
                    target x, y and z coordinates
        """
        self.state = state.MOVING
        target_theta = self.target_theta(self.new_pos.x, self.new_pos.y, msg.pose.position.x, msg.pose.position.y)
        self.target_pos = Position(msg.pose.position.x, msg.pose.position.y, target_theta) # theta is the orientation the turtle robot should face

    def sub_tilt_callback(self, msg):
        """ Callback function for the tilt topic.

            Receives a tilt angle for the platform
            
            Args:
                msg (turtle_brick_interfaces/Tilt): A message that contains a float for the angle
        """
        self.platform_angle = msg.angle

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
    ### NAVIGATION FUNCTIONS
    ###
    def move_turtle(self, input_msg, vel_x, vel_y):
        """ Returns a geometry_msgs/Twist message that moves the turtle at a given velocity.

            Args:
                input_msg (geometry_msgs/Twist): The initial movement command of the turtle, corresponding with linear x & y velocity
                vel (float): The desired velocity of the turtle
            
            Returns:
                geometry_msgs/Twist: The new movement command for the turtle to move towards the target waypoint
        """
        new_msg = input_msg
        new_msg.linear.x = vel_x
        new_msg.linear.y = vel_y
        return new_msg

    
    def nav_to_goal(self, input_msg, goal_pos):
        """ Returns geometry_msgs/Twist message to guide turtle robot to a given waypoint from its current position.

            Args:
                input_msg (geometry_msgs/Twist): The initial movement command of the turtle, corresponding with linear & angular velocity
                goal_pos (Position): The current target Position (or goal) the turtle is moving towards
            
            Returns:
                geometry_msgs/Twist: The new movement command for the turtle to move towards the target waypoint
        """
        new_msg = input_msg
        # If turtle has reached the goal, change state to STOPPED
        if self.is_near(self.new_pos, goal_pos, self.tolerance):
            self.state = state.STOPPED
            new_msg = Twist()
        # Else, move towards the goal Position
        else:
            vel = self.max_velocity
            # normalize the translation vector between current and goal Positions
            diff = [goal_pos.x - self.new_pos.x, goal_pos.y - self.new_pos.y]
            norm_diff = self.unit_vector_two(diff)
            # If the turtle is close enough to the goal, slow down the velocity
            if self.is_near(self.new_pos, goal_pos, 0.5):
                vel *= 0.2
            new_msg = self.move_turtle(new_msg, vel * norm_diff[0], vel * norm_diff[1])

        return new_msg
    
    ###
    ### HELPER FUNCTIONS
    ###
    def new_transform(self, old, new):
        """ Returns a Position that reflects the transform between old and new positions
        """
        tf = Position()
        tf.x = new.x - old.x
        tf.y = new.y - old.y
        tf.theta = new.theta - old.theta
        return tf

    def update_tf(self, input_tf, change):
        """ Returns an updated TransformStamped that reflects the transform/change in position
        """
        tf = input_tf
        tf.transform.translation.x = change.x
        tf.transform.translation.y = change.y
        tf.transform.rotation = angle_axis_to_quaternion(change.theta, [0, 0, -1.0])
        return tf
    
    def is_near(self, start_pos, end_pos, rad):
        """ Returns a boolean that indicates if a set of given 2D coordinates are within a given radius of another set of coordinates.

            Args:
                start_pos (Position): The current Position
                end_pos (Position): The target Position
                rad (float): The radius the two points have to be within of each other to be marked as 'near' each other
            
            Returns:
                Bool: States whether the two points are near each other is True/False
        """
        dist = self.distance_helper(start_pos, end_pos)
        return dist <= rad
    
    def distance_helper(self, start_pos, end_pos):
        """ Returns the distance between two positions on the x & y coords
        """
        return math.sqrt((start_pos.x - end_pos.x)**2 + (start_pos.y - end_pos.y)**2)
    
    def unit_vector_two(self, input_v):
        """ Converts a two element vector (array) to a unit vector (array) and returns it.
            input_v[0] is the x component
            input_v[1] is the y component
        """
        mag = math.sqrt(input_v[0]**2 + input_v[1]**2)
        return [input_v[0]/mag, input_v[1]/mag]
    
    def target_theta(self, start_x, start_y, end_x, end_y):
        """ Returns a float that indicates the angle a given end point is relative to a given starting point.
            (This value is within [-pi, pi])

            Args:
                start_x (float): The current x-coord
                start_y (float): The current y-coord
                end_x (float): The target x-coord
                end_y (float): The target y-coord
            
            Returns:
                Float: The angle between the starting and target points are
        """
        delt_x = end_x - start_x
        delt_y = end_y - start_y
        theta = math.atan2(delt_y, delt_x)
        return theta

def turtle_robot_entry(args=None):
    rclpy.init(args=args)
    node = TurtleRobot()
    rclpy.spin(node)
    rclpy.shutdown()
