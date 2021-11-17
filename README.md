HOW TO CREATE THE URDF, SRDF AND SDF FILES:
-------------------------------------------
First add the path of the packged to the ```ROS_PACKAGE_PATH``` env variable so that ROS can find it:

```export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:path/to/iit-relax-ros-pkg```

If everything goes well you should be able to ```roscd relax_urdf```.

Then go to the ```../iit-relax-ros-pkg/relax_urdf/scripts/``` folder and run:

```./create_urdf_srdf_sdf.sh relax```

Due to missing packages there could be some error messages, the final ```.urdf```, ```.srdf``` and ```.sdf``` should 
anyway be created.


# How to use it - Dummy mode

- set_xbot2_config */relax_config/relax_basic.yaml
- roscore
- xbot2-core -H dummy
- rviz (with robot model, tf)
- rosrun tf static_transform_publisher 0 0 0 0 0 0 ci/base_link base_link 2
- mon launch relax_cartesio_config relax_cartesio.launch
- Interactive marker to move the arm
