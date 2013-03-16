set( lib_name GbxSerialAcfr )
set( lib_version 1.0.0 )
set( lib_desc "C++ class wrapping a serial port. Part of GearBox." )
GBX_ADD_LICENSE( LGPL2+ )

set( build TRUE )
GBX_REQUIRE_OPTION( build LIB ${lib_name} ON )
GBX_REQUIRE_VAR( build LIB ${lib_name} GBX_OS_LINUX "only Linux OS is supported" )

set( dep_libs GbxLockFileAcfr )
# this is currently internal, so we don't have to check it
# GBX_REQUIRE_LIBS( build LIB ${lib_name} ${dep_libs} )

if( build )

    add_subdirectory( lockfile )

    include( ${GBX_CMAKE_DIR}/UseBasicRules.cmake )

    # for config.h
    include_directories( ${PROJECT_BINARY_DIR} )

    file( GLOB hdrs *.h )
    file( GLOB srcs *.cpp )

    GBX_ADD_LIBRARY( ${lib_name} DEFAULT ${lib_version} ${srcs} )
    target_link_libraries( ${lib_name} ${dep_libs} )
    GBX_ADD_PKGCONFIG( ${lib_name} ${lib_desc} "" dep_libs "" "" ${lib_version})

    GBX_ADD_HEADERS( gbxserialacfr ${hdrs} )

    if( GBX_BUILD_TESTS )
        add_subdirectory( test )
    endif( GBX_BUILD_TESTS )

endif( build )

