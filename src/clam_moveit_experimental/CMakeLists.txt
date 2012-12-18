cmake_minimum_required(VERSION 2.8.3)
project(clam_moveit_experimental)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread date_time system filesystem program_options python)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  actionlib 
  eigen_conversions 
  moveit_ros_planning 
  moveit_ros_warehouse 
  moveit_ros_planning_interface
  roscpp 
  tf
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS 
     include
#  LIBRARIES clam_moveit_experimental
   CATKIN_DEPENDS 
     actionlib 
     eigen_conversions 
     moveit_ros_planning 
     moveit_ros_warehouse 
     moveit_ros_planning_interface
     roscpp 
     tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

## Declare a cpp library
# add_library(clam_moveit_experimental
#   src/${PROJECT_NAME}/clam_moveit_experimental.cpp
# )

## Declare a cpp executable
add_executable(clam_moveit_experimental_node src/clam_moveit_experimental_node.cpp)

## Add dependencies to the executable
add_dependencies(clam_moveit_experimental_node ${catkin_LIBRARIES})

## Specify libraries to link a library or executable target against
target_link_libraries(clam_moveit_experimental_node
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS clam_moveit_experimental clam_moveit_experimental_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_clam_moveit_experimental.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
