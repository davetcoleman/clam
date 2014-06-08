cmake_minimum_required(VERSION 2.8.3)
project(clam_ikfast_arm_plugin)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
  DEPENDS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
)

include_directories(include)

set(IKFAST_LIBRARY_NAME clam_arm_moveit_ikfast_plugin)

find_package(LAPACK REQUIRED)

add_library(${IKFAST_LIBRARY_NAME} src/clam_arm_ikfast_moveit_plugin.cpp)
target_link_libraries(${IKFAST_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

install(TARGETS ${IKFAST_LIBRARY_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

install(
  FILES
  clam_arm_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)