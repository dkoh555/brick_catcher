import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.msg import ParameterDescriptor

from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped, Pose
from std_srvs.srv import Empty
from turtle_brick_interfaces.msg import Tilt
from turtle_brick_interfaces.srv import Place

from .quaternion import angle_axis_to_quaternion
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import math

from enum import Enum

from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class state(Enum):
    """ Current state of the system (arena node).
        Determines what movement commands are published to the turtle robot,
        whether it is MOVING, HOVERING, SLIDING (off the platform)
    """
    FALLING = 0
    HOVERING = 1
    CAUGHT = 2
    GROUNDED = 3

class Position3D:
    """ Class for storing the position of the brick in the arena,
        containing x, y, z, and theta.
        (Only one DoF for rotation because turtle_robot platform also only has one rotational DoF)
    """
    def __init__(self, x=0.0, y=0.0, z=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta

class Catcher(Node):
    """ Node for simulating the environment in rviz,
        containing the walls of the arena and the brick with its physics
    """
    def __init__(self):
        # Initialize the node
        super().__init__('catcher')
        # Initialize variables
        self.init_var()

        ###
        ### PARAMETERS
        ###
        # Declare and get the following parameters: platform_height, max_velocity, gravity_accel
        self.declare_parameter("platform_height", 2.0,
                               ParameterDescriptor(description="The height between the turtle platform and the ground"))
        self.platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        self.declare_parameter("max_velocity", 5.0,
                               ParameterDescriptor(description="The velocity of the turtle"))
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        self.declare_parameter("gravity_accel", 9.81,
                               ParameterDescriptor(description="The acceleration caused by gravity"))
        self.gravity_accel = self.get_parameter("gravity_accel").get_parameter_value().double_value
        # Declare and get extra parameters
        self.declare_parameter("platform_radius", 0.5,
                               ParameterDescriptor(description="The platform radius of the turtle robot"))
        self.platform_radius = self.get_parameter("platform_radius").get_parameter_value().double_value

        ###
        ### PUBLISHERS
        ###
        # Create publisher for goal_pose messages
        self.pub_goal = self.create_publisher(PoseStamped, 'goal_pose', 10)
        # Create publisher for tilt messages
        self.pub_tilt = self.create_publisher(Tilt, 'tilt', 10)
        # Create publisher for the text markers
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_marker = self.create_publisher(Marker, "visualization_marker_", markerQoS)

        ###
        ### LISTENER
        ###
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        ###
        ### TIMER
        ###
        self.timer = self.create_timer(self.period, self.timer_callback)

    def init_var(self):
        """ Initialize all of the arena node's variables
        """
        # initialize brick variables
        self.brick_pos = Position3D(3.0, 3.0, 7.5, 0.0)
        self.old_brick_pos = Position3D()
        self.brick_vel = 0.0
        self.time_start_falling = None 
        # initialize turtle_robot/platform variables
        self.robot_pos = Position3D()
        self.odom_pos = Position3D()
        self.platform_pos = Position3D()
        # initialize general node variables
        self.reachable = False
        self.state = state.HOVERING
        self.frequency = 250
        self.period = 1/self.frequency

    ###
    ### TIMER CALLBACK
    ### 
    def timer_callback(self):
        # Update robot, brick, and odom positions
        self.update_robot_pos()
        self.update_brick_pos()
        self.update_odom_pos()
        self.update_platform_pos()

        # Initialize Tilt message
        tilt_msg = Tilt()
        tilt_msg.angle = 0.0

        # If at any point, the brick's new position is larger than the old position then it has been placed there/reset
        if self.is_brick_reset():
            self.state = state.HOVERING
        # If the brick is hovering, check if it's reachable
        if self.state == state.HOVERING:
            self.reachable = self.can_reach()
            # If the brick's new position is lower than the old position, then it is falling
            if self.is_brick_falling():
                self.state = state.FALLING
            self.time_start_falling = self.get_clock().now().to_msg().sec
        # If the brick is falling...
        if self.state == state.FALLING:
            # And if the robot can reach it in time, move to catch it; otherwise, publish marker saying it cannot
            if self.reachable:
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'world'
                msg.pose = Pose()
                msg.pose.position.x = self.brick_pos.x
                msg.pose.position.y = self.brick_pos.y
                self.pub_goal.publish(msg)
                # Keep checking if the brick is on the platform yet, and change state to CAUGHT if so
                if self.is_caught():
                    self.state = state.CAUGHT
            else:
                if (self.get_clock().now().to_msg().sec - self.time_start_falling <= 3):
                    self.pub_unreachable()
                else:
                    self.remove_unreachable()

        # If the brick is caught...
        if self.state == state.CAUGHT:
            # And f not near the odom point, move towards it;
            # else send the message to tilt the platform
            if not self.is_near_xy(self.robot_pos, self.odom_pos, 0.1):
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'world'
                msg.pose = Pose()
                msg.pose.position.x = self.odom_pos.x
                msg.pose.position.y = self.odom_pos.y
                self.pub_goal.publish(msg)
            else:
                # Send message to tilt the platform
                tilt_msg.angle = 0.5
                # If the brick and platform are a platform_radius distance away from each other,
                # center the platform and set state to GROUNDED
                if self.distance_helper3D(self.platform_pos, self.brick_pos) >= (self.platform_radius + 0.75):
                    tilt_msg.angle = 0.0
                    self.state = state.GROUNDED
        
        # Publish Tilt message
        self.pub_tilt.publish(tilt_msg)

    ###
    ### SERVICE CALLBACKS
    ###
    def drop_callback(self, request, response):
        """ Callback function for the drop service.

            When provided with a std_srvs/Empty message,
            the node will switch from HOVERING to FALLING states
            
            Args:
                request (Empty): A message that contains nothing

                response (Empty): The response object

            Returns:
                Empty: Contains nothing
        """
        # If brick is HOVERING and not on the ground, it will begin to fall
        if self.state == state.HOVERING:
            self.state = state.FALLING
        return response
        
    def place_callback(self, request, response):
        """ Callback function for the place service.

            When provided with a Place request, the brick will relocate in the requested 3D coordinates
            
            Args:
                x (float64): The desired x coord of the brick
                y (float64): The desired y coord of the brick
                z (float64): The desired z coord of the brick

                response (Empty): The response object

            Returns:
                Empty: Contains nothing
        """
        self.state = state.HOVERING
        return response

    ###
    ### TF LISTENER
    ###
    def update_robot_pos(self):
        trans_ready = self._tf_buffer.can_transform('world', 'base_link', rclpy.time.Time())
        if trans_ready:
            trans = self._tf_buffer.lookup_transform('world', 'base_link', rclpy.time.Time())
            self.robot_pos = Position3D(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z, 0.0)

    def update_brick_pos(self):
        self.old_brick_pos = self.brick_pos # Update the old brick position
        trans_ready = self._tf_buffer.can_transform('world', 'brick', rclpy.time.Time())
        if trans_ready:
            trans = self._tf_buffer.lookup_transform('world', 'brick', rclpy.time.Time())
            self.brick_pos = Position3D(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z, 0.0) # The new brick position

    def update_odom_pos(self):
        trans_ready = self._tf_buffer.can_transform('world', 'odom', rclpy.time.Time())
        if trans_ready:
            trans = self._tf_buffer.lookup_transform('world', 'odom', rclpy.time.Time())
            self.odom_pos = Position3D(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z, 0.0)
    
    def update_platform_pos(self):
        trans_ready = self._tf_buffer.can_transform('world', 'platform_link', rclpy.time.Time())
        if trans_ready:
            trans = self._tf_buffer.lookup_transform('world', 'platform_link', rclpy.time.Time())
            self.platform_pos = Position3D(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z, 0.0)

    ###
    ### CATCHING FUNCTIONS
    ###
    def is_brick_falling(self):
        if self.brick_pos.z < self.old_brick_pos.z and self.brick_pos.x == self.old_brick_pos.x and self.brick_pos.y and self.old_brick_pos.y:
            return True
        else:
            return False

    def is_brick_reset(self):
        if self.brick_pos.z > self.old_brick_pos.z:
            return True
        else:
            return False
    
    def can_reach(self):
        # Calculating the time for the brick to hit the platform
        dist_to_platform = self.brick_pos.z - self.platform_height
        start_vel = 0.0
        accel = self.gravity_accel
        # Calculate the discriminant
        discriminant = start_vel**2 + 2 * accel * dist_to_platform
        # Ensure the discriminant is non-negative
        if discriminant < 0:
            return False
        # Solve for time using quadratic formula
        r1 = (-start_vel + math.sqrt(discriminant))/accel
        r2 = (-start_vel - math.sqrt(discriminant))/accel
        # Return the positive root
        time_to_fall = max(r1, r2)

        # Calculating the time for the robot to make it to the brick
        dist_to_brick = self.distance_helper(self.robot_pos, self.brick_pos)
        time_to_reach = dist_to_brick/self.max_velocity

        # Return boolean
        return time_to_reach <= time_to_fall
    
    def is_caught(self):
        # If the brick is on the platform, return True
        return self.is_near_xy(self.robot_pos, self.brick_pos, self.platform_radius) and (self.brick_pos.z <= self.platform_height)

    def pub_unreachable(self):
        msg = Marker()
        msg.header.frame_id = "brick"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.id = 6
        msg.type = Marker.TEXT_VIEW_FACING
        msg.action = Marker.ADD
        msg.scale.x = 1.0
        msg.scale.y = 1.0
        msg.scale.z = 1.5
        msg.pose.position.z = 1.0
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 1.0
        msg.color.a = 1.0
        msg.text = "Unreachable"
        self.pub_marker.publish(msg)

    def remove_unreachable(self):
        msg = Marker()
        msg.header.frame_id = "brick"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.id = 6
        msg.action = Marker.DELETE
        self.pub_marker.publish(msg)


    ###
    ### HELPER FUNCTIONS
    ###
    def is_near_xy(self, start_pos, end_pos, rad):
        """ Returns a boolean that indicates if a set of given x & y coordinates are within a given radius of another set of coordinates.

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
    
    def distance_helper3D(self, start_pos, end_pos):
        """ Returns the distance between two positions on the x & y coords
        """
        return math.sqrt((start_pos.x - end_pos.x)**2 + (start_pos.y - end_pos.y)**2 + (start_pos.z - end_pos.z)**2)


def catcher_entry(args=None):
    rclpy.init(args=args)
    node = Catcher()
    rclpy.spin(node)
    rclpy.shutdown()