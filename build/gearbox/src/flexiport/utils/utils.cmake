PROJECT (FlexiPortUtils)

INCLUDE_DIRECTORIES (/home/dave/ros/clam/install)

ADD_EXECUTABLE (porttoport porttoport.cpp)
TARGET_LINK_LIBRARIES (porttoport flexiport)
SET_TARGET_PROPERTIES (porttoport PROPERTIES
                       LINK_FLAGS "-L/home/dave/ros/clam/install/lib/gearbox"
                       INSTALL_RPATH "${INSTALL_RPATH};/home/dave/ros/clam/install/lib/gearbox"
                       BUILD_WITH_INSTALL_RPATH TRUE)
