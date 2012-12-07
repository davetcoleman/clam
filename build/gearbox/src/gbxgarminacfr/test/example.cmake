PROJECT( gbxgarminacfr_example )

INCLUDE_DIRECTORIES( /home/dave/ros/clam/install/include/gearbox )

ADD_EXECUTABLE( gbxgarminacfrtest test.cpp )
TARGET_LINK_LIBRARIES( gbxgarminacfrtest GbxSickAcfr )
SET_TARGET_PROPERTIES( gbxgarminacfrtest PROPERTIES
                       LINK_FLAGS "-L/home/dave/ros/clam/install/lib/gearbox"
                       INSTALL_RPATH "${INSTALL_RPATH};/home/dave/ros/clam/install/lib/gearbox"
                       BUILD_WITH_INSTALL_RPATH TRUE )
