cmake_minimum_required(VERSION 2.8.3)
project(clam_moveit_experimental)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  tf
  clam_block_manipulation
  eigen_conversions
  moveit_ros_planning
  moveit_ros_planning_interface
  visualization_msgs
  clam_controller
  clam_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread date_time system filesystem program_options python)


###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS
  roscpp
  actionlib
  tf
  clam_block_manipulation
  eigen_conversions
  moveit_ros_planning
  moveit_ros_planning_interface
  visualization_msgs
  clam_controller
  clam_msgs
  #  DEPENDS system_lib
  #   INCLUDE_DIRS
  #     include

  )

# Enable gdb to show line numbers
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")
set(CMAKE_BUILD_TYPE Debug)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  #  include
  ${catkin_INCLUDE_DIRS}
  )
link_directories(${catkin_LIBRARY_DIRS})

## Declare cpp executables

# Coverage test
#add_executable(coverage_test src/coverage_test.cpp)
#target_link_libraries(coverage_test ${catkin_LIBRARIES})

# End effector offset test
#add_executable(ee_offset_test src/ee_offset_test.cpp)
#target_link_libraries(ee_offset_test ${catkin_LIBRARIES})

# Up Down Test
#add_executable(up_down_test src/up_down_test.cpp)
#target_link_libraries(up_down_test ${catkin_LIBRARIES})

# Inverse Kinematics Test
#add_executable(ik_test src/ik_test.cpp)
#target_link_libraries(ik_test ${catkin_LIBRARIES})

