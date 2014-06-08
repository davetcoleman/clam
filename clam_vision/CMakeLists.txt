cmake_minimum_required(VERSION 2.8.3)
project(clam_vision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  image_transport
  cv_bridge
  image_geometry
  tf
  visualization_msgs
  cmake_modules
  )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  pcl_ros
  pcl_conversions
  image_transport
  cv_bridge
  image_geometry
  tf
  visualization_msgs
  )

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS}
  )

## Declare a cpp library
# add_library(clam_bringup
#   src/${PROJECT_NAME}/clam_bringup.cpp
# )

## Declare a cpp executable
add_executable(calibrate_kinect_checkerboard
  src/detect_calibration_pattern.cpp
  src/calibrate_kinect_checkerboard.cpp
  )
target_link_libraries(calibrate_kinect_checkerboard ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(manual_square_calibrate src/manual_square_calibrate.cpp)
target_link_libraries(manual_square_calibrate ${catkin_LIBRARIES} ${OpenCV_LIBS})

