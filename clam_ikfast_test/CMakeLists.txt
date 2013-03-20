cmake_minimum_required(VERSION 2.8.3)
project(clam_ikfast_test)

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  #eigen_conversions
  )

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
  DEPENDS
  moveit_core
  pluginlib
  roscpp
  #eigen_conversions
  )

include_directories(include)

set(MOVEIT_LIB_NAME clam_moveit_ikfast_kinematics_plugin)

find_package(LAPACK REQUIRED)

add_library(${MOVEIT_LIB_NAME} src/clam_arm_ikfast_moveit_plugin.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

install(
  FILES
  clam_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )















