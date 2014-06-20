cmake_minimum_required(VERSION 2.8.3)
project(clam_moveit_plugins)

find_package(Boost REQUIRED thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  rosconsole
  urdf
  kdl_parser
  tf
  tf_conversions
  control_msgs
  clam_msgs
  dynamixel_hardware_interface
  )

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
  clam_moveit_controller_manager
  DEPENDS
  clam_msgs
  )

# Enable gdb to show line numbers
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")
#set(CMAKE_BUILD_TYPE Debug)

include_directories(clam_moveit_controller_manager/include)

add_subdirectory(clam_moveit_controller_manager)

install(
  FILES
    clam_moveit_controller_manager_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
