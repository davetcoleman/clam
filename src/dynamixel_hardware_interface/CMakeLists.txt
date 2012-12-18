cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_hardware_interface)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp rospy std_msgs genmsg gearbox pluginlib actionlib control_msgs trajectory_msgs diagnostic_updater std_srvs)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES
  JointState.msg
  MotorStateList.msg
  MotorState.msg
)

add_service_files(DIRECTORY srv FILES 
  ListControllers.srv
  RestartController.srv
  SetComplianceMargin.srv
  SetComplianceSlope.srv
  SetTorqueLimit.srv
  SetVelocity.srv
  StartController.srv
  StopController.srv
  TorqueEnable.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build 
include_directories(include ${catkin_INCLUDE_DIRS})

# Add additional libraries
add_library(${PROJECT_NAME} src/dynamixel_io.cpp src/serial_proxy.cpp)
target_link_libraries(${PROJECT_NAME} flexiport)

# Add Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})


## Link the executable against catkin - order matters:
#  include first local includes, than catkin, than any third party stuff
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# EXECUTABLE ----------------------------------------------------
add_executable(controller_manager src/controller_manager.cpp)
target_link_libraries(controller_manager ${PROJECT_NAME})


# LIBRARY -------------------------------------------------------
add_library(dynamixel_controllers src/joint_position_controller.cpp
                                  src/joint_torque_controller.cpp
                                  src/joint_trajectory_action_controller.cpp)
target_link_libraries(dynamixel_controllers ${PROJECT_NAME})

# Test code
add_executable(dynamixel_io test/main.cpp)
target_link_libraries(dynamixel_io ${PROJECT_NAME})


option (DYNAMIXEL_BUILD_BINDINGS "Build the Python bindings for Dynamixel Driver" ON)
if (DYNAMIXEL_BUILD_BINDINGS)
    add_subdirectory (python)
endif (DYNAMIXEL_BUILD_BINDINGS)


#--------------------------------------------------

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries







