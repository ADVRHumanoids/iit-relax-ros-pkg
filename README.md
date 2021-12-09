HOW TO CREATE THE URDF, SRDF AND SDF FILES:
-------------------------------------------
First add the path of the packged to the ```ROS_PACKAGE_PATH``` env variable so that ROS can find it:

```export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:path/to/iit-relax-ros-pkg```

If everything goes well you should be able to ```roscd relax_urdf```.

Then go to the ```../iit-relax-ros-pkg/relax_urdf/scripts/``` folder and run:

```./create_urdf_srdf_sdf.sh relax```

Due to missing packages there could be some error messages, the final ```.urdf```, ```.srdf``` and ```.sdf``` should 
anyway be created.

# Dependency
- XBot2
- Cartesian Interface

# How to use it - Dummy mode (only RViz)

- clone this repository
- set_xbot2_config [local_path]/relax_config/relax_basic.yaml
- roscore
- xbot2-core -H dummy
- rviz (with robot model, tf:  relax_config/relax.rviz)
- rosrun tf static_transform_publisher 0 0 0 0 0 0 ci/base_link base_link 2
- mon launch relax_cartesio_config relax_cartesio.launch
- Interactive marker to move the arm or the base_link

You can change the file `relax_urdf/urdf/config/relax.urdf.xacro` you can select if you want the base, arm or both.
After the changes you have to create again the urdf.
