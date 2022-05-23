
cmake_minimum_required(VERSION 3.0)

set(ROBOT_NAME relax)
project(iit-${ROBOT_NAME}-ros-pkg VERSION 1.0.0)

include(GenerateXrdf.cmake)

## add sub-packages
add_subdirectory(relax_urdf)
add_subdirectory(relax_srdf)
add_subdirectory(relax_gazebo)
add_subdirectory(relax_config)
add_subdirectory(relax_cartesio_config)

enable_testing()
add_subdirectory(relax_test)

include(GenerateDeb.cmake)