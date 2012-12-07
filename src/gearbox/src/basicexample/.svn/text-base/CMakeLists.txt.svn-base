set( lib_name basicexample )
set( libVersion 1.0.0 )
GBX_ADD_LICENSE( LGPL2+ )

set( build TRUE )
GBX_REQUIRE_OPTION( build LIB ${lib_name} OFF )

if( build )

    include( ${GBX_CMAKE_DIR}/UseBasicRules.cmake )

    file( GLOB hdrs *.h )
    file( GLOB srcs *.cpp )

    GBX_ADD_LIBRARY( ${lib_name} DEFAULT ${libVersion} ${srcs} )

    GBX_ADD_HEADERS( basicexample ${hdrs} )
    # the custom command above is equivalent to this
#     install( FILES ${hdrs} DESTINATION include/gearbox/basicexample )

#     if( GBX_BUILD_TESTS )
#         add_subdirectory( test )
#     endif( GBX_BUILD_TESTS )

endif( build )
