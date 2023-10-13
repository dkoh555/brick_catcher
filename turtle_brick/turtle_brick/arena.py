import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker
from visualization_msgs.msg import Marker, InteractiveMarkerControl, MarkerArray
from rcl_interfaces.msg import ParameterDescriptor

from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import TransformStamped, Pose, Quaternion
from std_srvs.srv import Empty
from turtle_brick_interfaces.srv import Place

from .quaternion import angle_axis_to_quaternion
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import math

from enum import Enum

class state(Enum):
    """ Current state of the system (arena node).
        Determines what movement commands are published to the turtle robot,
        whether it is MOVING or STOPPED
    """
    FALLING = 0
    STOPPED = 1
    PLATFORM = 2

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
    """ Node for simulating the environment in rviz
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
        # Declare and get the following parameters: gravity_accel
        self.declare_parameter("gravity_accel", 9.81,
                               ParameterDescriptor(description="The acceleration caused by gravity"))
        self.gravity_accel = self.get_parameter("gravity_accel").get_parameter_value().double_value

        ###
        ### PUBLISHERS
        ###
        # Create publisher for the Arena markers
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_wall = self.create_publisher(Marker, "visualization_marker", markerQoS)

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
        # create the broadcaster for transforms
        self.broadcaster = TransformBroadcaster(self)

        ###
        ### TIMER
        ###
        timer_period = 1.0/self.frequency  # seconds
        # create timer and timer callback
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def init_var(self):
        # initialize arena variables
        self.arena_length = 11.12
        self.wall_width = 0.005
        self.wall_length = self.arena_length + 2.0 * self.wall_width
        self.wall_height = 1.0
        # initialize brick variables
        self.brick_dim = [0.5, 0.3, 0.3] # x, y, z measurements
        self.brick_pos = Position3D(3.0, 3.0, 7.5, 0.0)
        # initialize general node variables
        self.state = state.STOPPED
        self.frequency = 250

    ###
    ### TIMER CALLBACK
    ### 
    def timer_callback(self):
        # Initialize the current time
        time = self.get_clock().now().to_msg()

        # Declaring all the import variables/transforms/joint states
        world_brick_tf = TransformStamped()

        # If the brick is in a FALLING state, adjust the tf
        if self.state == state.FALLING:
            pass

        # Create the transform for world -> brick
        world_brick_tf.header.stamp = time
        world_brick_tf.header.frame_id = "world"
        world_brick_tf.child_frame_id = "brick"
        temp_tf = self.new_transform(self.brick_pos, Position3D())
        world_brick_tf = self.update_tf(world_brick_tf, temp_tf)
        self.broadcaster.sendTransform(world_brick_tf)

        # Publishing Markers
        self.pub_walls()
        self.pub_brick()

    ###
    ### SERVICE CALLBACKS
    ###
    def place_callback(self, request, response):
        pass

    def drop_callback(self, request, response):
        pass

    ###
    ### HELPER FUNCTIONS
    ###
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

    ###
    ### MARKER FUNCTIONS
    ###
    def pub_brick(self):
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
        self.pub_wall.publish(self.brick)

    def pub_walls(self):
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
        self.pub_wall.publish(self.n_wall)

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
        self.pub_wall.publish(self.s_wall)

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
        self.pub_wall.publish(self.e_wall)

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
        self.pub_wall.publish(self.w_wall)

def arena_entry(args=None):
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()