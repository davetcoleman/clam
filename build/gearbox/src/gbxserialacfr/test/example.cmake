PROJECT( gbxserialacfr_example )

INCLUDE_DIRECTORIES( /home/dave/ros/clam/install/include/gearbox )

ADD_EXECUTABLE( serialechotest serialechotest.cpp )
TARGET_LINK_LIBRARIES( serialechotest GbxSerialAcfr )
SET_TARGET_PROPERTIES( serialechotest PROPERTIES
                       LINK_FLAGS "-L/home/dave/ros/clam/install/lib/gearbox"
                       INSTALL_RPATH "${INSTALL_RPATH};/home/dave/ros/clam/install/lib/gearbox"
                       BUILD_WITH_INSTALL_RPATH TRUE )

ADD_EXECUTABLE( serialloopbacktest serialloopbacktest.cpp )
TARGET_LINK_LIBRARIES( serialloopbacktest GbxSerialAcfr )
SET_TARGET_PROPERTIES( serialloopbacktest PROPERTIES
                       LINK_FLAGS "-L/home/dave/ros/clam/install/lib/gearbox"
                       INSTALL_RPATH "${INSTALL_RPATH};/home/dave/ros/clam/install/lib/gearbox"
                       BUILD_WITH_INSTALL_RPATH TRUE )
