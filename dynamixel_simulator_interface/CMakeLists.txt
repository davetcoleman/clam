cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_simulator_interface)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  genmsg 
  gearbox 
  pluginlib 
  actionlib 
  control_msgs 
  trajectory_msgs 
  diagnostic_updater 
  std_srvs
  dynamixel_hardware_interface
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  genmsg 
  gearbox 
  pluginlib 
  actionlib 
  control_msgs 
  trajectory_msgs 
  diagnostic_updater 
  std_srvs
  dynamixel_hardware_interface
)

## Build 
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

# Add additional libraries
#add_library(${PROJECT_NAME} src/dynamixel_io.cpp src/serial_proxy.cpp)
#target_link_libraries(${PROJECT_NAME} flexiport)
# Wait for messages to be ready
#add_dependencies(${PROJECT_NAME} dynamixel_simulator_interface_gencpp) # This line is needed to ensure that messages are done being built before this is built

# Add Boost
#find_package(Boost REQUIRED COMPONENTS thread)
#include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})


## Link the executable against catkin - order matters:
#  include first local includes, than catkin, than any third party stuff
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# EXECUTABLE ----------------------------------------------------
add_executable(sim_controller_manager src/controller_manager.cpp)
target_link_libraries(sim_controller_manager ${catkin_LIBRARIES})


# LIBRARY -------------------------------------------------------
add_library(sim_dynamixel_controllers src/joint_position_controller.cpp
#                                      src/joint_torque_controller.cpp
                                      src/joint_trajectory_action_controller.cpp)
add_definitions(-DERROR_OUTPUT_LOG) # this tells the trajectory_action_controller to log position error to file
target_link_libraries(sim_dynamixel_controllers ${catkin_LIBRARIES})

# Wait for messages to be ready
add_dependencies(sim_controller_manager dynamixel_hardware_interface_gencpp) # This line is needed to ensure that messages are done being built before this is built
add_dependencies(sim_dynamixel_controllers dynamixel_hardware_interface_gencpp) # This line is needed to ensure that messages are done being built before this is built




