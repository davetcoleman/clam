cmake_minimum_required( VERSION 2.4 FATAL_ERROR )
project( gearbox )

#include_directories(include)
#include_directories(src/flexiport)

find_package(catkin REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
)

#
# project version
#
set( GBX_PROJECT_VERSION_MAJOR "10" )
set( GBX_PROJECT_VERSION_MINOR "11" )
set( GBX_PROJECT_VERSION_PATCH "0" )

set( GBX_PROJECT_VERSION
    ${GBX_PROJECT_VERSION_MAJOR}.${GBX_PROJECT_VERSION_MINOR}.${GBX_PROJECT_VERSION_PATCH} )

#
# The rest is done by a script
#
include( cmake/internal/Setup.cmake )


# DTC stolen from moveit_kinematics_model
install(DIRECTORY include/ DESTINATION include)