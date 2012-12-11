cmake_minimum_required(VERSION 2.8.3)
project(clam_interactive_markers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED)

catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES ${PROJECT_NAME}
)

add_executable(clam_marker_server src/clam_marker_server.cpp)

