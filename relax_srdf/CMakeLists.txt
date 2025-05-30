project(${ROBOT_NAME}_srdf)

find_package(catkin REQUIRED)

catkin_package()

# main srdf, with acm
generate_srdf(
  XACRO ${ROBOT_NAME}.srdf.xacro 
  CONFIG config/${ROBOT_NAME}.urdf.xacro 
  ADD_TO_ALL TRUE
  GEN_ACM TRUE
)

# arm srdf, with acm
generate_srdf(
  XACRO ${ROBOT_NAME}_arm.srdf.xacro 
  CONFIG config/${ROBOT_NAME}_arm.urdf.xacro 
  ADD_TO_ALL TRUE
  GEN_ACM TRUE
)


# updated acm for capsule model
generate_capsule_srdf_acm(
  CONFIG_NAME ${ROBOT_NAME}
  ADD_TO_ALL TRUE
)

# plain srdf for virtual frame model
generate_srdf(
  XACRO ${ROBOT_NAME}.srdf.xacro 
  CONFIG config/${ROBOT_NAME}_virtual_frame.urdf.xacro 
  ADD_TO_ALL TRUE
)

# mark srdf for installation
install(DIRECTORY srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)