CMAKE_MINIMUM_REQUIRED (VERSION 2.4)

PROJECT( hokuyo_aist_example )

INCLUDE_DIRECTORIES( /home/dave/ros/clam/install/include/gearbox )

ADD_EXECUTABLE( hokuyo_aist_example example.cpp )
TARGET_LINK_LIBRARIES( hokuyo_aist_example hokuyo_aist )
SET_TARGET_PROPERTIES( hokuyo_aist_example PROPERTIES
                       LINK_FLAGS "-L/home/dave/ros/clam/install/lib/gearbox"
                       INSTALL_RPATH "${INSTALL_RPATH};/home/dave/ros/clam/install/lib/gearbox"
                       BUILD_WITH_INSTALL_RPATH TRUE )
