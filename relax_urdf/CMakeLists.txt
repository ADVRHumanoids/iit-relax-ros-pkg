project(${ROBOT_NAME}_urdf)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

# mark urdf, meshes, launch and rviz files for installation
install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


# main urdf
generate_urdf(
  XACRO ${ROBOT_NAME}.urdf.xacro
  CONFIG config/${ROBOT_NAME}.urdf.xacro
  ADD_TO_ALL TRUE
)

# main urdf xbot2
generate_urdf(
  XACRO ${ROBOT_NAME}_xbot2.urdf.xacro
  CONFIG config/${ROBOT_NAME}_xbot2.urdf.xacro
  ADD_TO_ALL TRUE
)


# MIR urdf
generate_urdf(
  XACRO ${ROBOT_NAME}.urdf.xacro
  CONFIG config/${ROBOT_NAME}_mir.urdf.xacro
  ADD_TO_ALL TRUE
)

# arm urdf
generate_urdf(
  XACRO ${ROBOT_NAME}.urdf.xacro
  CONFIG config/${ROBOT_NAME}_arm.urdf.xacro
  ADD_TO_ALL TRUE
)

# capsule model for main urdf
generate_capsule_urdf(
  CONFIG_NAME ${ROBOT_NAME}
  ADD_TO_ALL TRUE
)
