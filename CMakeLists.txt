cmake_minimum_required(VERSION 2.8)
PROJECT(iit-relax-ros-pkg)

add_custom_target(	remove_old_relax_urdf_gazebo ALL
    				        COMMAND ${CMAKE_COMMAND} -E remove_directory 
        			      "~/.gazebo/models/relax_urdf/")

add_custom_target(  remove_old_relax_gazebo ALL
                    COMMAND ${CMAKE_COMMAND} -E remove_directory 
                    "~/.gazebo/models/relax/")

add_custom_target(  compile_urdf_sdf ALL
                    COMMAND ${CMAKE_SOURCE_DIR}/relax_urdf/scripts/create_urdf_srdf_sdf.sh relax)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    add_custom_target(	copy_relax_gazebo ALL
        				COMMAND ${CMAKE_COMMAND} -E copy_directory 
            			"${PROJECT_SOURCE_DIR}/relax_gazebo/sdf/" 
            			"~/.gazebo/models/relax/"
            			DEPENDS remove_old_relax_gazebo)
endif()
