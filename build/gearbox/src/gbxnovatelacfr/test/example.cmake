CMAKE_MINIMUM_REQUIRED (VERSION 2.4)

PROJECT( gbxnovatelacfr_example )

INCLUDE_DIRECTORIES( /home/dave/ros/clam/install/include/gearbox )

ADD_EXECUTABLE( gbxnovatelacfrtest test.cpp )
TARGET_LINK_LIBRARIES( gbxnovatelacfrtest GbxNovatelAcfr )
SET_TARGET_PROPERTIES( gbxnovatelacfrtest PROPERTIES
                       LINK_FLAGS "-L/home/dave/ros/clam/install/lib/gearbox"
                       INSTALL_RPATH "${INSTALL_RPATH};/home/dave/ros/clam/install/lib/gearbox"
                       BUILD_WITH_INSTALL_RPATH TRUE )
