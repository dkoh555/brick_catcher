"""
Broadcasts the TFs of the brick to appropriately simulate the simple physics of a brick
falling, landing, and sliding. Additionally, takes in positions to place the brick above
the arena and service requests on when to drop said brick.

PUBLISHERS:
    visualization_marker_array_ (visualization_msgs/MarkerArray) - Contains markers that visualize
        the walls and brick objects in rviz

SERVICES:
    place (turtle_brick_interfaces/Place) - Receives the position for where to place the brick in
        the arena to be dropped
    drop (std_srvs/Empty) - Receives the signal to drop the brick

BROADCASTERS:
    world_brick - The transform from the world to the brick frame

LISTENER:
    world_base_link - The transform from the world to the base_link frame of the robot
    world_platform_link - The transform from the world to the platform_link frame of the robot

PARAMETERS:
    gravity_accel (double) - The acceleration caused by gravity
    platform_height (double) - The height between the turtle robot's platform and the ground
    platform_radius (double) - The platform radius of the turtle robot

"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.msg import ParameterDescriptor

from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import TransformStamped, Quaternion
from std_srvs.srv import Empty
from turtle_brick_interfaces.srv import Place

from .quaternion import angle_axis_to_quaternion
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import math

from enum import Enum
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class state(Enum):
    """Current state of the system (arena node).
    Determines the current movement state of the brick in the environment,
    whether it is FALLING, GROUNDED, PLATFORMED, or SLIDING
    """

    FALLING = 0
    GROUNDED = 1
    PLATFORMED = 2
    SLIDING = 3


class Position3D:
    """Class for storing the position of the brick in the arena,
    containing x, y, z, and theta.
    (Only one DoF for rotation because turtle_robot platform also only has one rotational DoF)
    """

    def __init__(self, x=0.0, y=0.0, z=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta


class Arena(Node):
    """Node for simulating the environment in rviz,
    containing the walls of the arena and the brick with its physics
    """

    def __init__(self):
        # Initialize the node
        super().__init__("arena")
        # Initialize variables
        self.init_var()
        # This node will use Reentrant Callback Groups for nested services
        self.cbgroup = ReentrantCallbackGroup()

        #
        # PARAMETERS
        #
        # Declare and get the following parameters: gravity_accel, platform_height
        self.declare_parameter(
            "gravity_accel",
            9.81,
            ParameterDescriptor(description="The acceleration caused by gravity"),
        )
        self.gravity_accel = (
            self.get_parameter("gravity_accel").get_parameter_value().double_value
        )
        self.declare_parameter(
            "platform_height",
            2.0,
            ParameterDescriptor(
                description="The height between the turtle robot's platform and the ground"
            ),
        )
        self.platform_height = (
            self.get_parameter("platform_height").get_parameter_value().double_value
        )
        # Declare and get extra parameters
        self.declare_parameter(
            "platform_radius",
            0.5,
            ParameterDescriptor(description="The platform radius of the turtle robot"),
        )
        self.platform_radius = (
            self.get_parameter("platform_radius").get_parameter_value().double_value
        )

        #
        # PUBLISHERS
        #
        # Create publisher for the Arena markers
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_marker = self.create_publisher(
            MarkerArray, "visualization_marker_array_", markerQoS
        )

        #
        # SERVICES
        #
        # Create service that moves the brick to a provided location in the world
        self.srv_place = self.create_service(
            Place, "place", self.place_callback, callback_group=self.cbgroup
        )
        # Create service that triggers the brick to drop towards the ground
        self.srv_drop = self.create_service(
            Empty, "drop", self.drop_callback, callback_group=self.cbgroup
        )

        #
        # BROADCASTER
        #
        # Create the broadcaster for transforms
        self.broadcaster = TransformBroadcaster(self)

        #
        # LISTENER
        #
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        #
        # TIMER
        #
        self.timer = self.create_timer(self.period, self.timer_callback)

    def init_var(self):
        """Initialize all of the arena node's variables"""
        # initialize arena variables
        self.arena_length = 11.12
        self.wall_width = 0.005
        self.wall_length = self.arena_length + 2.0 * self.wall_width
        self.wall_height = 1.0
        # initialize brick variables
        self.brick_dim = [0.5, 0.3, 0.3]  # x, y, z measurements
        self.brick_pos = Position3D(3.0, 3.0, 7.5, 0.0)
        self.brick_vel = 0.0  # velocity along the z-axis
        self.brick_vel_x = 0.0
        # initialize turtle_robot/platform variables
        self.robot_pos = Position3D()
        self.platform_angle = 0.0
        self.raw_platform_angle = Quaternion()
        self.robot_tf_ready = False
        self.platform_pos = Position3D()
        # initialize general node variables
        self.state = state.GROUNDED
        self.frequency = 250
        self.period = 1 / self.frequency

    #
    # TIMER CALLBACK
    #
    def timer_callback(self):
        """Timer callback for the arena node.

        Depending on the transforms and positions of the robot, platform, and brick,
        modify the arena's state and the resulting transform/new position of brick
        with each passing unit of time
        """
        # Initialize the current time
        self.time = self.get_clock().now().to_msg()

        # Update the robot position and platform angle in the arena
        self.update_robot_pos()
        self.update_platform_pos()
        self.update_raw_platform_angle()

        # Declaring all the import variables/transforms/joint states
        world_brick_tf = TransformStamped()

        # If the brick is in a GROUNDED state, all brick velocities are equal zero
        if self.state == state.GROUNDED:
            self.brick_vel = 0.0
            self.brick_vel_x = 0.0

        # If the brick is in a FALLING state, update it's new positions accordingly
        if self.state == state.FALLING:
            self.falling_brick()
            # If the brick reaches the ground, switch to a STOPPED state
            if self.is_on_ground():
                self.state = state.GROUNDED
                self.brick_vel = 0.0
            if self.is_on_platform():
                self.state = state.PLATFORMED
                self.brick_vel = 0.0

        # If the brick is in a PLATFORMED state, it needs to follow the platform
        if self.state == state.PLATFORMED:
            self.following_brick()
            # If the platform is not centered, change to SLIDING state
            if not self.raw_platform_angle == Quaternion():
                self.brick_pos.theta = 0.5
                self.platform_angle = 0.5
                self.state = state.SLIDING

        # If the brick is in a SLIDING state, update it's new positions accordingly
        if self.state == state.SLIDING:
            self.sliding_brick()
            # If the brick and platform are a platform_radius distance away from each other,
            # the brick is considered off the platform and switches to the GROUNDED state
            if self.distance_helper3D(self.platform_pos, self.brick_pos) >= (
                self.platform_radius + 0.75
            ):
                self.state = state.GROUNDED

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

    #
    # SERVICE CALLBACKS
    #
    def place_callback(self, request, response):
        """Callback function for the place service.

        When provided with a turtle_brick_interfaces/Place message,
        the arena node will switch to a GROUNDED state and update the
        brick's position accordingly

        Args:
            request (Place): A message that contains the desired x, y,
                and z coordinates of the brick

            response (Empty): The response object

        Returns:
            Empty: Contains nothing
        """
        self.state = state.GROUNDED
        self.brick_pos = Position3D(request.x, request.y, request.z, 0.0)
        return response

    def drop_callback(self, request, response):
        """Callback function for the drop service.

        When provided with a std_srvs/Empty message,
        the node will switch conditionally switch from
        the GROUNDED to the FALLING state

        Args:
            request (Empty): A message that contains nothing

            response (Empty): The response object

        Returns:
            Empty: Contains nothing
        """
        # If brick is STOPPED and not on the ground, it will begin to fall
        if self.state == state.GROUNDED and self.brick_pos.z >= 0.0:
            self.state = state.FALLING
        return response

    #
    # TF LISTENER
    #
    def update_robot_pos(self):
        """Checks the appropriate transform and updates the robot's current base_link position
        """
        trans_ready = self._tf_buffer.can_transform(
            "world", "base_link", rclpy.time.Time()
        )
        if trans_ready:
            trans = self._tf_buffer.lookup_transform(
                "world", "base_link", rclpy.time.Time()
            )
            self.robot_pos = Position3D(
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
                0.0,
            )

    def update_raw_platform_angle(self):
        """Checks the appropriate transform and updates the robot's current platform angle
        """
        trans_ready = self._tf_buffer.can_transform(
            "world", "platform_link", rclpy.time.Time()
        )
        if trans_ready:
            trans = self._tf_buffer.lookup_transform(
                "world", "platform_link", rclpy.time.Time()
            )
            self.raw_platform_angle = trans.transform.rotation

    def update_platform_pos(self):
        """Checks the appropriate transform and updates the robot's current platform position
        """
        trans_ready = self._tf_buffer.can_transform(
            "world", "platform_link", rclpy.time.Time()
        )
        if trans_ready:
            trans = self._tf_buffer.lookup_transform(
                "world", "platform_link", rclpy.time.Time()
            )
            self.platform_pos = Position3D(
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
                0.0,
            )

    #
    # BRICK FUNCTIONS
    #
    def falling_brick(self):
        """Updates the brick's position that reflects the falling brick, updating the
        brick's velocity in the process
        """
        displacement = self.find_displacement(
            self.brick_vel, self.gravity_accel, self.period
        )
        self.brick_vel = self.find_vel(self.brick_vel, self.gravity_accel, self.period)
        self.brick_pos.z -= displacement

    def following_brick(self):
        """Updates the brick's position that mirrors the change in posiiton of the platform,
        hence following the platform
        """
        self.brick_pos.x = self.robot_pos.x
        self.brick_pos.y = self.robot_pos.y

    def sliding_brick(self):
        """Updates the brick's position that reflects the sliding brick according
        to platform'sangle, updating the brick's velocity in the process
        """
        # Find the z component
        z_accel = self.gravity_accel * math.sin(self.platform_angle) ** 2
        displacement_z = self.find_displacement(self.brick_vel, z_accel, self.period)
        self.brick_vel = self.find_vel(self.brick_vel, z_accel, self.period)
        self.brick_pos.z -= displacement_z
        # Find the x component
        x_accel = (
            self.gravity_accel
            * math.sin(self.platform_angle)
            * math.cos(self.platform_angle)
        )
        displacement_x = self.find_displacement(self.brick_vel_x, x_accel, self.period)
        self.brick_vel_x = self.find_vel(self.brick_vel_x, x_accel, self.period)
        self.brick_pos.x += displacement_x

    def is_on_ground(self):
        """Returns true if the brick has landed on the ground"""
        return self.brick_pos.z <= 0.0

    def is_on_platform(self):
        """Returns true if the brick has landed on the platform
        """
        cond1 = self.brick_pos.z <= self.platform_height
        cond2 = self.is_near_xy(self.brick_pos, self.robot_pos, self.platform_radius)
        return cond1 and cond2

    #
    # HELPER FUNCTIONS
    #
    def find_vel(self, old_vel, acceleration, time):
        """Returns the new velocity after taking into account old velocity, acceleration,
            and time period

        Args:
            old_vel (float): The current/old velocity
            acceleration (float): The current acceleration
            time (float): The size of the time period that has passed during this calculation

        Returns:
            float: The new velocity
        """
        return old_vel + acceleration * time

    def find_displacement(self, old_vel, acceleration, time):
        """Returns the displacement over a time period after taking into account old velocity,
            acceleration, and time period

        Args:
            old_vel (float): The current/old velocity
            acceleration (float): The current acceleration
            time (float): The size of the time period that has passed during this calculation

        Returns:
            float: The new velocity
        """
        return old_vel * time + 0.5 * acceleration * time**2

    def new_transform(self, new, old):
        """Returns a Position3D that reflects the transform between old and new positions

        Args:
            new (Position3D): The new position
            old (Position3D): The old position

        Returns:
            Position3D: The resulting change in posiiton
        """
        tf = Position3D()
        tf.x = new.x - old.x
        tf.y = new.y - old.y
        tf.z = new.z - old.z
        tf.theta = new.theta - old.theta
        return tf

    def update_tf(self, input_tf, change):
        """Returns an updated TransformStamped that reflects the transform/change in position

        Args:
                input_tf (TransformStamped): The starting transform
                change (Position3D): The change in position

            Returns:
                TransformStamped: The appropriately modified transform
        """
        tf = input_tf
        tf.transform.translation.x = change.x
        tf.transform.translation.y = change.y
        tf.transform.translation.z = change.z
        tf.transform.rotation = angle_axis_to_quaternion(change.theta, [0.0, 1.0, 0.0])
        return tf

    def is_near_xy(self, start_pos, end_pos, rad):
        """Returns a boolean that indicates if a set of given x & y coordinates are within a given
            radius of another set of coordinates.

        Args:
            start_pos (Position): The current Position
            end_pos (Position): The target Position
            rad (float): The radius the two points have to be within of each other to be
                marked as 'near' each other

        Returns:
            Bool: States whether the two points are near each other is True/False
        """
        dist = self.distance_helper(start_pos, end_pos)
        return dist <= rad

    def distance_helper(self, start_pos, end_pos):
        """Returns the distance between two positions (Posiiton3D) on the x & y coords
        """
        return math.sqrt(
            (start_pos.x - end_pos.x) ** 2 + (start_pos.y - end_pos.y) ** 2
        )

    def distance_helper3D(self, start_pos, end_pos):
        """Returns the distance between two positions (Posiiton3D) on the x, y, and z coords
        """
        return math.sqrt(
            (start_pos.x - end_pos.x) ** 2
            + (start_pos.y - end_pos.y) ** 2
            + (start_pos.z - end_pos.z) ** 2
        )

    #
    # MARKER FUNCTIONS
    #
    def brick_marker(self):
        """Creates the marker representing the brick and adds it to the marker array
        """
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
        self.brick.pose.position.z = (
            self.brick_dim[2] / 2
        )  # The brick frame is at the bottom face of the brick marker
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
        """Creates a marker array, and then a marker for each of the walls and adds them to
        said marker array
        """
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
        self.n_wall.pose.position.x = self.arena_length / 2
        self.n_wall.pose.position.y = self.arena_length
        self.n_wall.pose.position.z = self.wall_height / 2
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
        self.s_wall.pose.position.x = self.arena_length / 2
        self.s_wall.pose.position.y = 0.0
        self.s_wall.pose.position.z = self.wall_height / 2
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
        self.e_wall.pose.position.y = self.arena_length / 2
        self.e_wall.pose.position.z = self.wall_height / 2
        quaternion = angle_axis_to_quaternion(math.pi / 2, [0.0, 0.0, 1.0])
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
        self.w_wall.pose.position.y = self.arena_length / 2
        self.w_wall.pose.position.z = self.wall_height / 2
        quaternion = angle_axis_to_quaternion(math.pi / 2, [0.0, 0.0, 1.0])
        self.w_wall.pose.orientation.x = quaternion.x
        self.w_wall.pose.orientation.y = quaternion.y
        self.w_wall.pose.orientation.z = quaternion.z
        self.w_wall.pose.orientation.w = quaternion.w
        self.w_wall.color.r = 0.0
        self.w_wall.color.g = 1.0
        self.w_wall.color.b = 1.0
        self.w_wall.color.a = 1.0
        # Filling the Marker Array
        self.marker_array.markers = [
            self.n_wall,
            self.s_wall,
            self.e_wall,
            self.w_wall,
            None,
        ]


def arena_entry(args=None):
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()
