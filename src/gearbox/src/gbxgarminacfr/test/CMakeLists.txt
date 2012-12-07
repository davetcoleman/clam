include( ${GBX_CMAKE_DIR}/UseBasicRules.cmake )

GBX_ADD_EXECUTABLE( gbxgarminacfrtest test.cpp )
target_link_libraries( gbxgarminacfrtest GbxGarminAcfr )

GBX_ADD_EXAMPLE( gbxgarminacfr example.cmake.in example.cmake test.cpp example.readme )