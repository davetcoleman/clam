set( lib_name GbxSerialDeviceAcfr )
set( lib_version 1.0.0 )
set( lib_desc "Serial port handler for GbxSickAcfr." )
GBX_ADD_LICENSE( LGPL2+ )

set( build TRUE )
# don't give user an option
# GBX_REQUIRE_OPTION( build LIB ${lib_name} ON )

# this was already tested in the dir above
set( dep_libs GbxSerialAcfr GbxIceUtilAcfr )
# GBX_REQUIRE_LIBS( build LIB ${lib_name} ${proj_libs} )

if( build )

    include( ${GBX_CMAKE_DIR}/UseBasicRules.cmake )
    include( ${GBX_CMAKE_DIR}/UseIceUtil.cmake )

    file( GLOB hdrs *.h )
    file( GLOB srcs *.cpp )

    GBX_ADD_LIBRARY( ${lib_name} DEFAULT ${lib_version} ${srcs} )
    target_link_libraries( ${lib_name} ${dep_libs} )
    GBX_ADD_PKGCONFIG( ${lib_name} ${lib_desc} dep_libs "" "" "" ${lib_version} )

    GBX_ADD_HEADERS( gbxsickacfr/gbxserialdeviceacfr ${hdrs} )

endif( build )
