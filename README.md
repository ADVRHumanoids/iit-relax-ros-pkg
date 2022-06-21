# iit-relax-ros-pkg 
Description of IIT's RELAX 6-DoF robot, plus configuration files enabling simulation inside the Gazebo environment via the XBot2 framework, and inverse kinematics via the CartesIO framework.

*NOTE:* we recommend the `rosmon` package as a replacement for the `roslaunch` command, as it is faster and provides useful introspection (`apt install ros-$ROS_DISTRO-rosmon`, then refresh the terminal).

## Installing the package
Drop it into a catkin workspace `src/` folder, and then invoke either `catkin_make` or `catkin build`.

Simulation and inverse kinematics also requires the `xbot2` (full desktop) binary distribution (instructions [here](https://advrhumanoids.github.io/xbot2/)).

## Visualizing the robot
Just type 
```bash
mon launch relax_urdf relax_slider.launch
```

## Simulating the robot

Terminal #1:
```bash
mon launch relax_gazebo relax_world.launch verbose:=true
```

Terminal #2:
```bash
xbot2-core --simtime --config $(rospack find relax_config)/relax_basic.yaml
```
*Note that the required configuration file can be set once and for all via `xbot2_set_config $(rospack find relax_config)/relax_basic.yaml`, allowing to omit the `--config` flag.*

It is now possible to play with the robot via the provided GUI (invoked with the `xbot2-gui` command), or to send joint space reference via ROS, e.g. using Python or C++ (see our [examples repository](https://github.com/ADVRHumanoids/xbot2_examples/blob/master/src/ros_api/README.md)).

## Inverse kinematics
Just type 
```bash
mon launch relax_cartesio relax_cartesio.launch gui:=true
```
The IK solver will attach to the `xbot2-core` program if available, and will otherwise run in "visual mode".

It is now possible to play wih the robot via interactive markers, or to send Cartesian space reference via ROS, e.g. using Python or C++ (see the docs [here](https://advrhumanoids.github.io/CartesianInterface/quickstart.html)). 

To move the end-effector via interacrive markers

 - right click on a control, select *Continuous control*
 - move the marker
 - the IK should track the provided reference

