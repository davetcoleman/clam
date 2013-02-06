cmake_minimum_required(VERSION 2.8.3)
project(clam_vision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl
  image_transport
  cv_bridge
  opencv2
  image_geometry
  tf
  visualization_msgs
  )
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Eigen
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  pcl
  pcl_ros
  image_transport
  cv_bridge
  opencv2
  image_geometry
  vision_opencv
  eigen
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
target_link_libraries(calibrate_kinect_checkerboard ${catkin_LIBRARIES})

add_executable(findContours_demo
  src/findContours_demo.cpp
  )
target_link_libraries(findContours_demo ${catkin_LIBRARIES})


add_executable(manual_square_calibrate src/manual_square_calibrate.cpp)
target_link_libraries(manual_square_calibrate ${catkin_LIBRARIES})

