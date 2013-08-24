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
add_executable(clam_gripper_controller src/clam_gripper_controller.cpp)
add_dependencies(clam_gripper_controller clam_msgs_gencpp ${PROJECT_NAME}_gencfg) # wait for clam_msgs to be built
target_link_libraries(clam_gripper_controller ${catkin_LIBRARIES})

# ---------------
add_executable(clam_gripper_controller_test src/clam_gripper_controller_test.cpp)
add_dependencies(clam_gripper_controller_test clam_msgs_gencpp ${PROJECT_NAME}_gencfg) # wait for clam_msgs to be built
target_link_libraries(clam_gripper_controller_test ${catkin_LIBRARIES})

