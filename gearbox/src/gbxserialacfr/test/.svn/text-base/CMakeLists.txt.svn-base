include( ${GBX_CMAKE_DIR}/UseBasicRules.cmake )

link_libraries( GbxSerialAcfr )

GBX_ADD_EXECUTABLE( serialechotest serialechotest.cpp )
GBX_ADD_EXECUTABLE( serialloopbacktest serialloopbacktest.cpp )

GBX_ADD_EXAMPLE( gbxserialacfr example.cmake.in example.cmake serialechotest.cpp serialloopbacktest.cpp )