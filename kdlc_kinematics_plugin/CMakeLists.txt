set(MOVEIT_LIB_NAME moveit_kdlc_kinematics_plugin)

add_library(${MOVEIT_LIB_NAME} src/kdlc_kinematics_plugin.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} moveit_rdf_loader ${catkin_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION include)

# Test script for simple_cache
add_executable(simple_cache_test src/simple_cache_test.cpp)
target_link_libraries(simple_cache_test ${catkin_LIBRARIES})
