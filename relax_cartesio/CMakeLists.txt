cmake_minimum_required(VERSION 3.0)
project(relax_cartesio)

find_package(catkin REQUIRED)

catkin_package()

install(
  FILES 
  relax_stack.yaml
  relax.launch
  mir_stack.yaml
  mir.launch 
  relax.rviz
  DESTINATION 
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
