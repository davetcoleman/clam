set( lib_name GbxGarminAcfr )
set( lib_version 1.0.0 )
set( lib_desc "Garmin GPS driver. Part of GearBox." )
GBX_ADD_LICENSE( LGPL2+ )

set( build TRUE )
GBX_REQUIRE_OPTION( build LIB ${lib_name} ON )
GBX_REQUIRE_VAR( build LIB ${lib_name} GBX_OS_LINUX "only Linux OS is supported" )

set( dep_libs GbxUtilAcfr GbxSerialAcfr )
GBX_REQUIRE_LIBS( build LIB ${lib_name} ${dep_libs} )

if( build )

    include( ${GBX_CMAKE_DIR}/UseBasicRules.cmake )

    file( GLOB hdrs *.h )
    file( GLOB srcs *.cpp )

    GBX_ADD_LIBRARY( ${lib_name} DEFAULT ${lib_version} ${srcs} )
    target_link_libraries( ${lib_name} ${dep_libs} )
    GBX_ADD_PKGCONFIG( ${lib_name} ${lib_desc} "" dep_libs "" "" ${lib_version} )

    GBX_ADD_HEADERS( gbxgarminacfr ${hdrs} )

    if( GBX_BUILD_TESTS )
        add_subdirectory( test )
    endif( GBX_BUILD_TESTS )

endif( build )
