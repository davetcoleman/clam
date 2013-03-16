set( lib_name GbxSickAcfr )
set( lib_version 1.0.0 )
set( lib_desc "SICK laser scanner driver. Part of GearBox." )
GBX_ADD_LICENSE( LGPL2+ )

set( build TRUE )
GBX_REQUIRE_OPTION( build LIB ${lib_name} ON )
GBX_REQUIRE_VAR( build LIB ${lib_name} GBX_OS_LINUX "only Linux OS is supported" )

add_subdirectory( gbxiceutilacfr )

set( dep_libs GbxUtilAcfr GbxIceUtilAcfr GbxSerialAcfr )
GBX_REQUIRE_LIBS( build LIB ${lib_name} ${dep_libs} )

# these are built internally, so we don't have to check for them
set( dep_libs ${dep_libs} GbxSerialDeviceAcfr )

if( build )

    add_subdirectory( gbxserialdeviceacfr )

    include( ${GBX_CMAKE_DIR}/UseBasicRules.cmake )
    include( ${GBX_CMAKE_DIR}/UseIceUtil.cmake )

    file( GLOB hdrs *.h )
    file( GLOB srcs *.cpp )

    GBX_ADD_LIBRARY( ${lib_name} DEFAULT ${lib_version} ${srcs} )
    target_link_libraries( ${lib_name} ${dep_libs} )
    GBX_ADD_PKGCONFIG( ${lib_name} ${lib_desc} "" dep_libs "" "" ${lib_version} )

    GBX_ADD_HEADERS( gbxsickacfr ${hdrs} )

    if( GBX_BUILD_TESTS )
        add_subdirectory( test )
    endif( GBX_BUILD_TESTS )

endif( build )
