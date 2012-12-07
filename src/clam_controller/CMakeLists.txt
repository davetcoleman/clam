cmake_minimum_required(VERSION 2.8.3)
project(clam_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED)

catkin_package(DEPENDS rospy roscpp dynamixel_hardware_interface std_msgs sensor_msgs
    INCLUDE_DIRS include
)

include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

add_executable(joint_state_aggregator src/joint_state_aggregator.cpp)
target_link_libraries(joint_state_aggregator ${PROJECT_NAME})