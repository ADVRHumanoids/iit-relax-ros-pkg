# relax_ros_control

## Setup to use ros_control (and moveit) with xbot2!

*Initially copy pasted from inail2arm_ros_control*

Clone this repo and https://github.com/ADVRHumanoids/ros_xbot_hw_interface

- switch on the robot / launch gazebo (eg `roslaunch relax_gazebo relax_simple_world.launch`)
- run xbot2 (eg `xbot2-core`)
- put robot in homing, better safe than sorry (eg `xbot2-gui`)
- launch ros -> xbot hw interface and the controllers (`roslaunch relax_ros_control relax_control.launch`). check the args for configurations

Further istructions soon 
