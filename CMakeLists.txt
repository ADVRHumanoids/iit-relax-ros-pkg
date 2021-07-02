cmake_minimum_required(VERSION 2.8)
PROJECT(iit-relax-ros-pkg)

add_custom_target(	remove_old_teleop_urdf_gazebo ALL
    				        COMMAND ${CMAKE_COMMAND} -E remove_directory 
        			      "~/.gazebo/models/relax_urdf/")

add_custom_target(  remove_old_relax_gazebo ALL
                    COMMAND ${CMAKE_COMMAND} -E remove_directory 
                    "~/.gazebo/models/relax/")

add_custom_target(  compile_urdf_sdf ALL
                    COMMAND ${CMAKE_SOURCE_DIR}/relax_urdf/scripts/create_urdf_srdf_sdf.sh relax)
