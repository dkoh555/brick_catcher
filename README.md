# ME495 Embedded Systems Homework 2
Author: Damien Koh

## Package Description
This package uses `turtlesim` to simulate a robot in ROS, which is visualized in rviz.
This `turtle` robot will then use various ROS systems, including `tf`, `joint_states`, and `URDF`, to control and show itself catching a brick that drops from the sky.

## Quickstart
1. Use `ros2 launch turtle_brick turtle_arena.launch.xml` to start the arena, turtlesim, and rviz.
2. Use `ros2 service call /drop std_srvs/srv/Empty "{}"` to drop a brick.
3. Here is a video of the turtle when the brick is within catching range:


4. Here is a video of the turtle when the brick cannot be caught:



## Extra Commands
1. Use commands similar to `ros2 service call /place turtle_brick_interfaces/srv/Place "{x: 5.5444, y: 10.2444, z: 7.3}"` to relocate the brick in a new location.