cmake_minimum_required(VERSION 2.8.3)
project(clam_block_manipulation)


add_action_files(DIRECTORY action FILES
  ClamArmAction
)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED
  roscpp
  std_msgs
  actionlib 
  visualization_msgs 
  interactive_markers 
  dynamixel_hardware_interface 
  pcl_ros
)

#uncomment if you have defined messages
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
  dynamixel_hardware_interface 
  pcl_ros
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)
 
#add_executable(block_manipulation test/block_manipulation.cpp)
#add_executable(block_detection_action_server src/block_detection_action_server.cpp)
#add_executable(interactive_manipulation_action_server src/interactive_manipulation_action_server.cpp)
#add_executable(pick_and_place_action_server src/pick_and_place_action_server.cpp)
add_executable(clam_arm_action_server src/clam_arm_action_server.cpp)
target_link_libraries(clam_arm_action_server ${catkin_LIBRARIES})

#add_executable(block_manipulation_actions test/block_manipulation_actions.cpp)
#add_executable(block_manipulation_action_demo demo/block_manipulation_action_demo.cpp)

#include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

