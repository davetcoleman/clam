set( lib_name GbxLockFileAcfr )
set( lib_version 1.0.0 )
set( lib_desc "Create lock files in /var/lock to control resource access. Part of GearBox." )
GBX_ADD_LICENSE( LGPL2+ )

set( build TRUE )
# don't give user an option
# GBX_REQUIRE_OPTION( build LIB ${lib_name} ON )
# this was already tested in the dir above
# GBX_REQUIRE_VAR( build LIB ${lib_name} GBX_OS_LINUX "only Linux OS is supported" )

if( build )

    include( ${GBX_CMAKE_DIR}/UseBasicRules.cmake )

    file( GLOB hdrs *.h )
    file( GLOB srcs *.cpp )

    GBX_ADD_LIBRARY( ${lib_name} DEFAULT ${lib_version} ${srcs} )
    target_link_libraries( ${lib_name} ${dep_libs} )
    GBX_ADD_PKGCONFIG( ${lib_name} ${lib_desc} "" "" "" "" ${lib_version} )

    GBX_ADD_HEADERS( gbxserialacfr/lockfile ${hdrs} )

    if( GBX_BUILD_TESTS )
        add_subdirectory( test )
    endif( GBX_BUILD_TESTS )

endif( build )
