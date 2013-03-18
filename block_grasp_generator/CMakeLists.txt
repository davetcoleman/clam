cmake_minimum_required(VERSION 2.8.3)
project(block_grasp_generator)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread system)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED
  roscpp
  std_msgs
  visualization_msgs 
  eigen_conversions 
  moveit_ros_planning 
  moveit_ros_planning_interface
  moveit_core
  tf
  tf_conversions
)

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
    visualization_msgs 
    eigen_conversions 
    moveit_ros_planning 
    moveit_ros_planning_interface
    moveit_core
    tf
    tf_conversions
  INCLUDE_DIRS include
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
#add_executable(grasp_generator_test src/grasp_generator_test.cpp)
#target_link_libraries(grasp_generator_test ${catkin_LIBRARIES})
