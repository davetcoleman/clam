cmake_minimum_required(VERSION 2.8.3)
project(clam_pick_place_planner)

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

find_package(Boost REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  moveit_msgs
  moveit_ros_planning
  )

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  DEPENDS
  roscpp 
  moveit_msgs
  moveit_ros_planning
  )

# BUILD

include_directories(include)


add_executable(pick_place_planner src/pick_place_planner.cpp)
target_link_libraries(pick_place_planner ${catkin_LIBRARIES} )
