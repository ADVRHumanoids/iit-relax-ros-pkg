HOW TO CREATE THE URDF, SRDF AND SDF FILES:
-------------------------------------------
First add the path of the packged to the ```ROS_PACKAGE_PATH``` env variable so that ROS can find it:

```export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:path/to/iit-relax-ros-pkg```

If everything goes well you should be able to ```roscd relax_urdf```.

Then go to the ```../iit-relax-ros-pkg/relax_urdf/scripts/``` folder and run:

```./create_urdf_srdf_sdf.sh relax```

Due to missing packages there could be some error messages, the final ```.urdf```, ```.srdf``` and ```.sdf``` should 
anyway be created.
