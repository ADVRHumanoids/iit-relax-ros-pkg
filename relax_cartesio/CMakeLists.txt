cmake_minimum_required(VERSION 3.0)
project(relax_cartesio)

find_package(catkin REQUIRED)

catkin_package()

install(
  FILES 
  config/relax_stack.yaml
  config/relax_arm_stack.yaml
  launch/relax.launch
  launch/relax_arm.launch
  config/mir_stack.yaml
  launch/mir.launch 
  launch/relax_arm_force_estimation.launch
  config/relax.rviz
  DESTINATION 
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
