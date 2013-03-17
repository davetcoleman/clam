cmake_minimum_required(VERSION 2.8.3)
project(clam_block_manipulation)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread date_time system filesystem program_options python)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED
  roscpp
  std_msgs
  actionlib 
  visualization_msgs 
  clam_controller 
  clam_msgs
  pcl_ros
  cv_bridge
  eigen_conversions 
  moveit_ros_planning 
  moveit_ros_planning_interface
  moveit_core
  tf
  tf_conversions
)

find_package(OpenCV REQUIRED)

#generate_messages(DEPENDENCIES
#  std_msgs
#  geometry_msgs  
#  actionlib_msgs
#)

# Grasp Library
add_library(${PROJECT_NAME}_grasp_generator
  src/grasp_generator.cpp
)
target_link_libraries(${PROJECT_NAME}_grasp_generator ${catkin_LIBRARIES} ${Boost_LIBRARIES})


catkin_package(
  LIBRARIES
    ${PROJECT_NAME}_grasp_generator
  CATKIN_DEPENDS
    roscpp
    std_msgs
    actionlib 
    visualization_msgs 
    clam_controller 
    clam_msgs
    pcl_ros
    cv_bridge
    eigen_conversions 
    moveit_ros_planning 
    moveit_ros_planning_interface
    moveit_core
    tf
    tf_conversions
#  INCLUDE_DIRS include
)

# Enable gdb to show line numbers
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")
set(CMAKE_BUILD_TYPE Debug)

## Build 
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

# Action Servers 
add_executable(block_perception_server src/block_perception_server.cpp)
target_link_libraries(block_perception_server ${catkin_LIBRARIES})

add_executable(block_pick_place_server src/block_pick_place_server.cpp)
target_link_libraries(block_pick_place_server ${catkin_LIBRARIES} ${PROJECT_NAME}_grasp_generator)

add_executable(block_manipulation_logic src/block_manipulation_logic.cpp)
target_link_libraries(block_manipulation_logic ${catkin_LIBRARIES})

# Test script for simple_cache
add_executable(simple_cache_test src/simple_cache_test.cpp)
target_link_libraries(simple_cache_test ${catkin_LIBRARIES})

