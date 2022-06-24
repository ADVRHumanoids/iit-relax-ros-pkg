cmake_minimum_required(VERSION 3.0)
project(relax_config)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY hal
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY mir_config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES relax_basic.yaml mir_basic.yaml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
