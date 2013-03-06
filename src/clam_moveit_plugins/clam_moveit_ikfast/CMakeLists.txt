set(MOVEIT_LIB_NAME clam_moveit_ikfast_kinematics_plugin)

add_library(${MOVEIT_LIB_NAME} src/clam_clam_arm_ikfast_moveit_plugin.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)


# Library containing IKFast plugin
#rosbuild_add_library(clam_moveit_arm_kinematics
#  src/clam_clam_arm_ikfast_moveit_plugin.cpp
#  )