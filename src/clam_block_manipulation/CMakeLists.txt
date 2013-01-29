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
  interactive_markers 
  clam_controller 
  clam_msgs
  pcl_ros
  eigen_conversions 
  moveit_ros_planning 
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs  
  actionlib_msgs
)

catkin_package(DEPENDS
  roscpp
  std_msgs
  actionlib 
  visualization_msgs 
  interactive_markers 
  clam_controller 
  clam_msgs
  pcl_ros
  eigen_conversions 
  moveit_ros_planning 
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
)

## Build 
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})
 
#add_executable(block_manipulation test/block_manipulation.cpp)

add_executable(block_detection_action_server src/block_detection_action_server.cpp)
target_link_libraries(block_detection_action_server ${catkin_LIBRARIES})

add_executable(block_logic_action_server src/block_logic_action_server.cpp)
target_link_libraries(block_logic_action_server ${catkin_LIBRARIES})

add_executable(pick_place_action_server src/pick_place_action_server.cpp)
target_link_libraries(pick_place_action_server ${catkin_LIBRARIES})

add_executable(block_manipulation_demo src/demo/block_manipulation_demo.cpp)
target_link_libraries(block_manipulation_demo ${catkin_LIBRARIES})

#include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})





