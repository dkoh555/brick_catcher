import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.msg import ParameterDescriptor

from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import TransformStamped, Quaternion
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from turtle_brick_interfaces.srv import Place

from .quaternion import angle_axis_to_quaternion
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import math

from enum import Enum

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class state(Enum):
    """ Current state of the system (arena node).
        Determines what movement commands are published to the turtle robot,
        whether it is MOVING, STOPPED, SLIDING (off the platform)
    """
    FALLING = 0
    STOPPED = 1
    SLIDING = 2

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

class Arena(Node):
    """ Node for simulating the environment in rviz,
        containing the walls of the arena and the brick with its physics
    """
    def __init__(self):
        # Initialize the node
        super().__init__('arena')
        # Initialize variables
        self.init_var()
        # This node will use Reentrant Callback Groups for nested services
        self.cbgroup = ReentrantCallbackGroup()

        ###
        ### PARAMETERS
        ###
        # Declare and get the following parameters: gravity_accel, platform_height
        self.declare_parameter("gravity_accel", 9.81,
                               ParameterDescriptor(description="The acceleration caused by gravity"))
        self.gravity_accel = self.get_parameter("gravity_accel").get_parameter_value().double_value
        self.declare_parameter("platform_height", 2.0,
                               ParameterDescriptor(description="The height between the turtle platform and the ground"))
        self.platform_height = self.get_parameter("platform_height").get_parameter_value().double_value
        # Declare and get extra parameters
        self.declare_parameter("platform_radius", 0.5,
                               ParameterDescriptor(description="The platform radius of the turtle robot"))
        self.platform_radius = self.get_parameter("platform_radius").get_parameter_value().double_value

        ###
        ### PUBLISHERS
        ###
        # Create publisher for the Arena markers
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_marker = self.create_publisher(MarkerArray, "visualization_marker_", markerQoS)

        ###
        ### SUBSCRIBERS
        ###
        # Create subscriber for getting the turtle's position in turtlesim
        self.sub_pos = self.create_subscription(Pose, 'turtle1/pose', self.sub_pos_callback, 10)
        self.sub_pos # Used to prevent warnings

        ###
        ### SERVICES
        ###
        # Create service that moves the brick to a provided location in the world
        self.srv_place = self.create_service(Place, 'place', self.place_callback, callback_group=self.cbgroup)
        # Create service that triggers the brick to drop towards the ground
        self.srv_drop = self.create_service(Empty, 'drop', self.drop_callback, callback_group=self.cbgroup)

        ###
        ### BROADCASTER
        ###
        # Create the broadcaster for transforms
        self.broadcaster = TransformBroadcaster(self)

        ###
        ### LISTENER
        ###
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ###
        ### TIMER
        ###
        self.timer = self.create_timer(self.period, self.timer_callback)


    def init_var(self):
        """ Initialize all of the arena node's variables
        """
        # initialize arena variables
        self.arena_length = 11.12
        self.wall_width = 0.005
        self.wall_length = self.arena_length + 2.0 * self.wall_width
        self.wall_height = 1.0
        # initialize brick variables
        self.brick_dim = [0.5, 0.3, 0.3] # x, y, z measurements
        self.brick_pos = Position3D(3.0, 3.0, 7.5, 0.0)
        self.brick_vel = 0.0
        # initialize turtle_robot/platform variables
        self.robot_pos = Position3D()
        self.platform_angle = 0.0
        self.robot_tf_ready = False
        # initialize general node variables
        self.state = state.STOPPED
        self.frequency = 250
        self.period = 1/self.frequency

    ###
    ### TIMER CALLBACK
    ### 
    def timer_callback(self):
        # Initialize the current time
        self.time = self.get_clock().now().to_msg()

        # Declaring all the import variables/transforms/joint states
        world_brick_tf = TransformStamped()

        # If the brick is in a FALLING state, adjust the tf
        if self.state == state.FALLING:
            self.falling_brick()
            # If the brick reaches the ground, switch to a STOPPED state
            if self.is_on_ground():
                self.state = state.STOPPED
                self.brick_vel = 0.0
            if self.is_on_platform():
                self.state = state.SLIDING
                self.brick_vel = 0.0

        # Create the transform for world -> brick
        world_brick_tf.header.stamp = self.time
        world_brick_tf.header.frame_id = "world"
        world_brick_tf.child_frame_id = "brick"
        temp_tf = self.new_transform(self.brick_pos, Position3D())
        world_brick_tf = self.update_tf(world_brick_tf, temp_tf)
        self.broadcaster.sendTransform(world_brick_tf)

        self.wall_markers()
        self.brick_marker()
        self.pub_marker.publish(self.marker_array)

    ###
    ### SUBSCRIBER CALLBACKS
    ###
    def sub_pos_callback(self, msg):
        self.robot_pos = Position3D(msg.x, msg.y, 0.0, self.platform_angle) # z = 0.0 because robot is on the ground
    
    ###
    ### SERVICE CALLBACKS
    ###
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
        self.state = state.STOPPED
        self.brick_pos = Position3D(request.x, request.y, request.z, 0.0)
        return response

    def drop_callback(self, request, response):
        """ Callback function for the drop service.

            When provided with a std_srvs/Empty message,
            the node will switch from STOPPED to FALLING states
            
            Args:
                request (Empty): A message that contains nothing

                response (Empty): The response object

            Returns:
                Empty: Contains nothing
        """
        # If brick is STOPPED and not on the ground, it will begin to fall
        if self.state == state.STOPPED and self.brick_pos.z >= 0.0:
            self.state = state.FALLING
        return response

    ###
    ### BRICK FUNCTIONS
    ###
    def falling_brick(self):
        """ Updates the brick's position that reflects the falling brick, updating the brick's velocity in the process
        """
        displacement = self.find_displacement(self.brick_vel, self.gravity_accel, self.period)
        self.brick_vel = self.find_vel(self.brick_vel, self.gravity_accel, self.period)
        self.brick_pos.z -= displacement

    def is_on_ground(self):
        """ Returns true if the brick has landed on the ground
        """
        return self.brick_pos.z <= 0.0

    def is_on_platform(self):
        """ Returns true if the brick has landed on the platform
        """
        cond1 = (self.brick_pos.z <= self.platform_height)
        cond2 = self.is_near_xy(self.brick_pos, self.robot_pos, self.platform_radius)
        return cond1 and cond2

    ###
    ### HELPER FUNCTIONS
    ###
    def find_vel(self, old_vel, acceleration, time):
        """ Returns the new velocity after taking into account old velocity, acceleration, and time difference
        """
        return old_vel + acceleration * time
    
    def find_displacement(self, old_vel, acceleration, time):
        """ Returns the displacement over a time period after taking into account old velocity, acceleration, and the period itself
        """
        return old_vel * time + 0.5 * acceleration * time**2

    def new_transform(self, new, old):
        """ Returns a Position3D that reflects the transform between old and new positions
        """
        tf = Position3D()
        tf.x = new.x - old.x
        tf.y = new.y - old.y
        tf.z = new.z - old.z
        tf.theta = new.theta - old.theta
        return tf
    
    def update_tf(self, input_tf, change):
        """ Returns an updated TransformStamped that reflects the transform/change in position
        """
        tf = input_tf
        tf.transform.translation.x = change.x
        tf.transform.translation.y = change.y
        tf.transform.translation.z = change.z
        tf.transform.rotation = angle_axis_to_quaternion(change.theta, [0.0, 0.0, -1.0])
        return tf
    
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

    ###
    ### MARKER FUNCTIONS
    ###
    def brick_marker(self):
        self.brick = Marker()
        self.brick.header.frame_id = "brick"
        self.brick.header.stamp = self.get_clock().now().to_msg()
        self.brick.id = 5
        self.brick.type = Marker.CUBE
        self.brick.action = Marker.ADD
        self.brick.scale.x = self.brick_dim[0]
        self.brick.scale.y = self.brick_dim[1]
        self.brick.scale.z = self.brick_dim[2]
        self.brick.pose.position.x = 0.0
        self.brick.pose.position.y = 0.0
        self.brick.pose.position.z = self.brick_dim[2]/2 # The brick frame is at the bottom face of the brick marker 
        quaternion = angle_axis_to_quaternion(0.0, [0.0, 0.0, 1.0])
        self.brick.pose.orientation.x = quaternion.x
        self.brick.pose.orientation.y = quaternion.y
        self.brick.pose.orientation.z = quaternion.z
        self.brick.pose.orientation.w = quaternion.w
        self.brick.color.r = 1.0
        self.brick.color.g = 0.0
        self.brick.color.b = 1.0
        self.brick.color.a = 1.0
        # Filling the Marker Array
        self.marker_array.markers[4] = self.brick

    def wall_markers(self):
        # Initialize the Marker Array
        self.marker_array = MarkerArray()
        
        self.n_wall = Marker()
        self.n_wall.header.frame_id = "world"
        self.n_wall.header.stamp = self.get_clock().now().to_msg()
        self.n_wall.id = 1
        self.n_wall.type = Marker.CUBE
        self.n_wall.action = Marker.ADD
        self.n_wall.scale.x = self.wall_length
        self.n_wall.scale.y = self.wall_width
        self.n_wall.scale.z = self.wall_height
        self.n_wall.pose.position.x = self.arena_length/2
        self.n_wall.pose.position.y = self.arena_length
        self.n_wall.pose.position.z = self.wall_height/2
        quaternion = angle_axis_to_quaternion(0.0, [0.0, 0.0, 1.0])
        self.n_wall.pose.orientation.x = quaternion.x
        self.n_wall.pose.orientation.y = quaternion.y
        self.n_wall.pose.orientation.z = quaternion.z
        self.n_wall.pose.orientation.w = quaternion.w
        self.n_wall.color.r = 1.0
        self.n_wall.color.g = 0.0
        self.n_wall.color.b = 0.0
        self.n_wall.color.a = 1.0

        self.s_wall = Marker()
        self.s_wall.header.frame_id = "world"
        self.s_wall.header.stamp = self.get_clock().now().to_msg()
        self.s_wall.id = 2
        self.s_wall.type = Marker.CUBE
        self.s_wall.action = Marker.ADD
        self.s_wall.scale.x = self.wall_length
        self.s_wall.scale.y = self.wall_width
        self.s_wall.scale.z = self.wall_height
        self.s_wall.pose.position.x = self.arena_length/2
        self.s_wall.pose.position.y = 0.0
        self.s_wall.pose.position.z = self.wall_height/2
        quaternion = angle_axis_to_quaternion(0.0, [0.0, 0.0, 1.0])
        self.s_wall.pose.orientation.x = quaternion.x
        self.s_wall.pose.orientation.y = quaternion.y
        self.s_wall.pose.orientation.z = quaternion.z
        self.s_wall.pose.orientation.w = quaternion.w
        self.s_wall.color.r = 0.0
        self.s_wall.color.g = 0.0
        self.s_wall.color.b = 1.0
        self.s_wall.color.a = 1.0

        self.e_wall = Marker()
        self.e_wall.header.frame_id = "world"
        self.e_wall.header.stamp = self.get_clock().now().to_msg()
        self.e_wall.id = 3
        self.e_wall.type = Marker.CUBE
        self.e_wall.action = Marker.ADD
        self.e_wall.scale.x = self.wall_length
        self.e_wall.scale.y = self.wall_width
        self.e_wall.scale.z = self.wall_height
        self.e_wall.pose.position.x = self.arena_length
        self.e_wall.pose.position.y = self.arena_length/2
        self.e_wall.pose.position.z = self.wall_height/2
        quaternion = angle_axis_to_quaternion(math.pi/2, [0.0, 0.0, 1.0])
        self.e_wall.pose.orientation.x = quaternion.x
        self.e_wall.pose.orientation.y = quaternion.y
        self.e_wall.pose.orientation.z = quaternion.z
        self.e_wall.pose.orientation.w = quaternion.w
        self.e_wall.color.r = 0.0
        self.e_wall.color.g = 1.0
        self.e_wall.color.b = 0.0
        self.e_wall.color.a = 1.0

        self.w_wall = Marker()
        self.w_wall.header.frame_id = "world"
        self.w_wall.header.stamp = self.get_clock().now().to_msg()
        self.w_wall.id = 4
        self.w_wall.type = Marker.CUBE
        self.w_wall.action = Marker.ADD
        self.w_wall.scale.x = self.wall_length
        self.w_wall.scale.y = self.wall_width
        self.w_wall.scale.z = self.wall_height
        self.w_wall.pose.position.x = 0.0
        self.w_wall.pose.position.y = self.arena_length/2
        self.w_wall.pose.position.z = self.wall_height/2
        quaternion = angle_axis_to_quaternion(math.pi/2, [0.0, 0.0, 1.0])
        self.w_wall.pose.orientation.x = quaternion.x
        self.w_wall.pose.orientation.y = quaternion.y
        self.w_wall.pose.orientation.z = quaternion.z
        self.w_wall.pose.orientation.w = quaternion.w
        self.w_wall.color.r = 0.0
        self.w_wall.color.g = 1.0
        self.w_wall.color.b = 1.0
        self.w_wall.color.a = 1.0
        # Filling the Marker Array
        self.marker_array.markers = [self.n_wall, self.s_wall, self.e_wall, self.w_wall, None]

def arena_entry(args=None):
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()