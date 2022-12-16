cmake_minimum_required(VERSION 3.0)
project(relax_cartesio)

find_package(catkin REQUIRED)

catkin_package()

install(
  FILES 
  relax_stack.yaml
  relax_arm_stack.yaml
  relax.launch
  relax_arm.launch
  mir_stack.yaml
  mir.launch 
  relax_arm_force_estimation.launch
  relax.rviz
  DESTINATION 
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
