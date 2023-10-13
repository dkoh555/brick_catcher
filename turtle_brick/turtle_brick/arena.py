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

from rclpy.qos import QoSProfile, QoSDurabilityPolicy

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
        ### PUBLISHERS
        ###
        # Create publisher for the Arena markers
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub1 = self.create_publisher(Marker, "visualization_marker", markerQoS)

        ###
        ### PARAMETERS
        ###
        # Declare and get the following parameters: gravity_accel
        self.declare_parameter("gravity_accel", 9.81,
                               ParameterDescriptor(description="The acceleration caused by gravity"))
        self.gravity_accel = self.get_parameter("gravity_accel").get_parameter_value().double_value

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
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.cbgroup)

    def init_var(self):
        self.frequency = 250

    ###
    ### TIMER CALLBACK
    ### 
    def timer_callback(self):
        self.m = Marker()
        self.m.header.frame_id = "world"
        self.m.header.stamp = self.get_clock().now().to_msg()
        self.m.id = 1
        self.m.type = Marker.CYLINDER
        self.m.action = Marker.ADD
        self.m.scale.x = 1.0
        self.m.scale.y = 1.0
        self.m.scale.z = 3.0
        self.m.pose.position.x = 5.0
        self.m.pose.position.y = 2.0
        self.m.pose.position.z = -1.0
        self.m.pose.orientation.x = .707
        self.m.pose.orientation.y = 0.0
        self.m.pose.orientation.z = 0.0
        self.m.pose.orientation.w = .707
        self.m.color.r = 0.0
        self.m.color.g = 0.0
        self.m.color.b = 1.0
        self.m.color.a = 1.0
        self.pub1.publish(self.m)

        self.m1 = Marker()
        self.m1.header.frame_id = "world"
        self.m1.header.stamp = self.get_clock().now().to_msg()
        self.m1.id = 2
        self.m1.type = Marker.CYLINDER
        self.m1.action = Marker.ADD
        self.m1.scale.x = 1.0
        self.m1.scale.y = 1.0
        self.m1.scale.z = 3.0
        self.m1.pose.position.x = -5.0
        self.m1.pose.position.y = 2.0
        self.m1.pose.position.z = -1.0
        self.m1.pose.orientation.x = .707
        self.m1.pose.orientation.y = 0.0
        self.m1.pose.orientation.z = 0.0
        self.m1.pose.orientation.w = .707
        self.m1.color.r = 1.0
        self.m1.color.g = 0.0
        self.m1.color.b = 0.0
        self.m1.color.a = 1.0
        self.pub1.publish(self.m1)

    ###
    ### SERVICE CALLBACKS
    ###
    def place_callback(self, request, response):
        pass

    def drop_callback(self, request, response):
        pass

def arena_entry(args=None):
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()