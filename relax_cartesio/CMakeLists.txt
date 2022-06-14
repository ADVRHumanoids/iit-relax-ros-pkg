cmake_minimum_required(VERSION 3.0)
project(relax_cartesio)

find_package(catkin REQUIRED)

catkin_package()

install(
  FILES 
  relax_manipulation_stack.yaml
  relax_manipulation.launch 
  DESTINATION 
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
