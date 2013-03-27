cmake_minimum_required(VERSION 2.8.3)
project(clam_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs
  actionlib 
  dynamixel_hardware_interface 
  std_msgs 
  sensor_msgs
  clam_msgs
  tf
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs
    actionlib 
    dynamixel_hardware_interface 
    std_msgs 
    sensor_msgs
    tf
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
)

## Build 
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

# ---------------
add_executable(joint_state_aggregator src/joint_state_aggregator.cpp)
target_link_libraries(joint_state_aggregator ${catkin_LIBRARIES} )

# ---------------
#add_executable(clam_arm_action_server src/clam_arm_action_server.cpp)
# This line is needed to ensure that messages are done being built before this is built
#add_dependencies(clam_arm_action_server clam_msgs_gencpp)
#target_link_libraries(clam_arm_action_server ${catkin_LIBRARIES})

# ---------------
add_executable(clam_gripper_server src/clam_gripper_server.cpp)
# This line is needed to ensure that messages are done being built before this is built
add_dependencies(clam_gripper_server clam_msgs_gencpp)
target_link_libraries(clam_gripper_server ${catkin_LIBRARIES})

