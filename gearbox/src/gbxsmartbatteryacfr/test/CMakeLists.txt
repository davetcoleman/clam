include( ${GBX_CMAKE_DIR}/UseBasicRules.cmake )

GBX_ADD_EXECUTABLE( gbxsmartbatteryshorttest shorttest.cpp )
target_link_libraries( gbxsmartbatteryshorttest GbxSmartBatteryAcfr )

GBX_ADD_EXECUTABLE( gbxsmartbatterylongtest longtest.cpp )
target_link_libraries( gbxsmartbatterylongtest GbxSmartBatteryAcfr )

GBX_ADD_EXAMPLE( gbxsmartbatteryshorttest example.cmake.in example.cmake shorttest.cpp example.readme )

if( GBX_BUILD_TESTS )
    add_subdirectory( darttest )
endif( GBX_BUILD_TESTS )


