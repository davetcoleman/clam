cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_hardware_interface)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  genmsg 
  gearbox 
  pluginlib 
  actionlib 
  control_msgs 
  trajectory_msgs 
  diagnostic_updater 
  diagnostic_msgs
  message_generation
  std_srvs
  std_msgs
  genmsg
  rospy
)

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
  LoadController.srv
  UnloadController.srv
  TorqueEnable.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    
)

## Build 
include_directories(include ${catkin_INCLUDE_DIRS} ${flexiport_INCLUDE_DIRS})

# Add additional libraries
add_library(${PROJECT_NAME} src/dynamixel_io.cpp src/serial_proxy.cpp)
target_link_libraries(${PROJECT_NAME} flexiport)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${flexiport_LIBRARIES} ${gearbox_LIBRARIES})


# Wait for messages to be ready
add_dependencies(${PROJECT_NAME} dynamixel_hardware_interface_gencpp) # This line is needed to ensure that messages are done being built before this is built

# Add Boost
find_package(Boost REQUIRED COMPONENTS thread python)
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
add_definitions(-DERROR_OUTPUT_LOG) # this tells the trajectory_action_controller to log position error to file
target_link_libraries(dynamixel_controllers ${PROJECT_NAME})

# Wait for messages to be ready
add_dependencies(controller_manager dynamixel_hardware_interface_gencpp) # This line is needed to ensure that messages are done being built before this is built
add_dependencies(dynamixel_controllers dynamixel_hardware_interface_gencpp) # This line is needed to ensure that messages are done being built before this is built

# Test code
add_executable(dynamixel_io test/main.cpp)
target_link_libraries(dynamixel_io ${PROJECT_NAME})


option (DYNAMIXEL_BUILD_BINDINGS "Build the Python bindings for Dynamixel Driver" ON)
if (DYNAMIXEL_BUILD_BINDINGS)
    add_subdirectory (python)
endif()

# Install libraries
install(TARGETS ${PROJECT_NAME} dynamixel_controllers
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install executables
install(TARGETS
  controller_manager
  dynamixel_io
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  FILES
    controller_plugins.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION})
