set( lib_name GbxAdvancedExample )
set( libVersion 1.0.0 )
GBX_ADD_LICENSE( GPL2+ )

set( build TRUE )
GBX_REQUIRE_OPTION( build LIB ${lib_name} OFF )
GBX_REQUIRE_VAR( build LIB ${lib_name} GBX_OS_LINUX "only Linux OS is supported" )

set( dep_libs basicexample )
GBX_REQUIRE_LIBS( build LIB ${lib_name} ${dep_libs} )

if( build )

    include( ${GBX_CMAKE_DIR}/UseBasicRules.cmake )

    file( GLOB hdrs *.h )
    file( GLOB srcs *.cpp )

    GBX_ADD_LIBRARY( ${lib_name} DEFAULT ${libVersion} ${srcs} )
    target_link_libraries( ${lib_name} ${dep_libs} )

    GBX_ADD_HEADERS( gbxadvancedexample ${hdrs} )

endif( build )
