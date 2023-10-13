import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker
from visualization_msgs.msg import Marker, InteractiveMarkerControl, MarkerArray
from rcl_interfaces.msg import ParameterDescriptor

from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import TransformStamped, Pose
from std_srvs.srv import Empty
from turtle_brick_interfaces.srv import Place

from .quaternion import angle_axis_to_quaternion
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import math

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
        # initialize variables
        self.arena_length = 11.12
        self.wall_width = 0.005
        self.wall_length = self.arena_length + 2.0 * self.wall_width
        self.wall_height = 1.0
        self.frequency = 250

    ###
    ### TIMER CALLBACK
    ### 
    def timer_callback(self):
        self.pub_walls()

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