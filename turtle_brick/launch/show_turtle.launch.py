from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import Command, LaunchConfiguration, \
                                 TextSubstitution, PathJoinSubstitution, EqualsSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    return LaunchDescription([

          # Launch file arguments
          DeclareLaunchArgument(name='use_jsp',
                                   default_value='gui',
                                   description='controls the behaviour of the joint_state_publisher (gui | jsp | none)'),
     
          # Launch the robot_state_publisher, with the turtle robot xacro loaded
          Node(package="robot_state_publisher",
               executable="robot_state_publisher",
               name="robot_state_publisher",
               parameters=[{
                    "robot_description": ParameterValue(Command([ExecutableInPackage("xacro", "xacro"), " ",
                                                                      PathJoinSubstitution([FindPackageShare("turtle_brick"),
                                                                                     "turtle.urdf.xacro"])]))}]),

          # Launch rviz
          Node(package="rviz2",
             executable="rviz2",
             name="rviz2"),


          # Launch the joint_state_publisher w/ the GUI for testing the URDF
          Node(package="joint_state_publisher",
               executable="joint_state_publisher",
               name="joint_state_publisher",
               condition=IfCondition(EqualsSubstitution(LaunchConfiguration('use_jsp'), 'jsp'))),
          # Launch the joint_state_publisher w/out the GUI
          Node(package="joint_state_publisher_gui",
               executable="joint_state_publisher_gui",
               name="joint_state_publisher_gui",
               condition=IfCondition(EqualsSubstitution(LaunchConfiguration('use_jsp'), 'gui'))),
    ])

