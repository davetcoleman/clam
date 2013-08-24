set(MOVEIT_LIB_NAME clam_moveit_controller_manager)

add_library(${MOVEIT_LIB_NAME} src/clam_moveit_controller_manager.cpp)
add_dependencies(${MOVEIT_LIB_NAME} clam_msgs_gencpp ${PROJECT_NAME}_gencfg) # wait for msgs to be buil
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

