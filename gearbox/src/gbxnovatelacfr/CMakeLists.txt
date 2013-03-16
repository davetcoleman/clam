set( lib_name GbxNovatelAcfr )
set( lib_version 1.0.0 )
set( lib_desc "NOVATEL INS/GPS driver. Part of GearBox." )
GBX_ADD_LICENSE( LGPL2+ )

set( build TRUE )
GBX_REQUIRE_OPTION( build LIB ${lib_name} ON )

set( dep_libs GbxUtilAcfr GbxSerialAcfr )
GBX_REQUIRE_LIBS( build LIB ${lib_name} ${dep_libs} )

# built internally
set( dep_libs ${dep_libs} GbxNovatelUtilAcfr )

if( build )

    add_subdirectory( gbxnovatelutilacfr)

    include( ${GBX_CMAKE_DIR}/UseBasicRules.cmake )

    file( GLOB hdrs *.h )
    file( GLOB srcs *.cpp )

    GBX_ADD_LIBRARY( ${lib_name} DEFAULT ${lib_version} ${srcs} )
    target_link_libraries( ${lib_name} ${dep_libs} )
    GBX_ADD_PKGCONFIG( ${lib_name} ${lib_desc} "" dep_libs "" "" ${lib_version} )

    GBX_ADD_HEADERS( gbxnovatelacfr ${hdrs} )

    if( GBX_BUILD_TESTS )
        add_subdirectory( test )
    endif( GBX_BUILD_TESTS )

endif( build )
