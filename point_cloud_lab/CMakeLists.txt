cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_lab)

#find_package(Eigen REQUIRED)
#find_package(Boost REQUIRED thread date_time system filesystem program_options python)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED
  roscpp
  std_msgs
  pcl_ros
  eigen_conversions 
  tf
  tf_conversions
)

catkin_package(DEPENDS
  roscpp
  std_msgs
  pcl_ros
  eigen_conversions 
  tf
  tf_conversions
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
)

# Enable gdb to show line numbers
#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")
set(CMAKE_BUILD_TYPE Debug)

## Build 
include_directories(
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})
 
add_executable(point_cloud_lab src/point_cloud_lab.cpp)
target_link_libraries(point_cloud_lab ${catkin_LIBRARIES})

add_executable(point_cloud_lab_template src/point_cloud_lab_template.cpp)
target_link_libraries(point_cloud_lab_template ${catkin_LIBRARIES})

