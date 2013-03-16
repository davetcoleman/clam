set( lib_name GbxUtilAcfr )
set( lib_version 1.0.0 )
set( lib_desc "Utilities including status tracing and exceptions. Part of GearBox." )
GBX_ADD_LICENSE( LGPL2+ )

set( build TRUE )
GBX_REQUIRE_OPTION( build LIB ${lib_name} ON )

if( build )
    if (WIN32)
        if (GBX_DEFAULT_LIB_TYPE STREQUAL SHARED)
            add_definitions (-DGBXUTILACFR_EXPORTS)
        else (GBX_DEFAULT_LIB_TYPE STREQUAL SHARED)
            add_definitions (-DGBXUTILACFR_STATIC -DFLEXIPORT_STATIC)
        endif (GBX_DEFAULT_LIB_TYPE STREQUAL SHARED)
    endif (WIN32)

    include( ${GBX_CMAKE_DIR}/UseBasicRules.cmake )

    file( GLOB hdrs *.h )
    file( GLOB srcs *.cpp )

    # for config.h
    include_directories( ${PROJECT_BINARY_DIR} )

    GBX_ADD_LIBRARY( ${lib_name} DEFAULT ${lib_version} ${srcs} )
    GBX_ADD_PKGCONFIG( ${lib_name} ${lib_desc} "" "" "" "" ${lib_version} )

    GBX_ADD_HEADERS( gbxutilacfr ${hdrs} )

    if( GBX_BUILD_TESTS )
        add_subdirectory( test )
    endif( GBX_BUILD_TESTS )

endif( build )
