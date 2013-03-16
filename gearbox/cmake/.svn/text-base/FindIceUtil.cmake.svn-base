# Locate IceUtil home
#
# This module defines the following variables:
# ICEUTIL_FOUND : 1 if Ice is found, 0 otherwise
# ICEUTIL_HOME  : path where to find include, lib, bin, etc.

# start with 'not found'
set( ICEUTIL_FOUND 0 CACHE BOOL "Do we have libIceUtil?" )

find_path( iceutil_home_include_iceutil_dir IceUtil.h
  # rational for this search order:
  #    source install w/env.var -> source install
  #    package -> package
  #    package + source install w/env.var -> source install
  #    package + source install w/out env.var -> package 
  #
  # installation selected by user
  ${ICEUTIL_HOME}/include/IceUtil
  ${ICE_HOME}/include/IceUtil
  $ENV{ICEUTIL_HOME}/include/IceUtil
  $ENV{ICE_HOME}/include/IceUtil
  # debian package installs Ice here
  /usr/include/IceUtil
  # Test standard installation points: newer versions first
  /opt/Ice-4.0/include/IceUtil
  /opt/Ice-3.6/include/IceUtil
  /opt/Ice-3.5/include/IceUtil
  /opt/Ice-3.4/include/IceUtil
  /opt/Ice-3.3/include/IceUtil
  # some people may manually choose to install Ice here
  /usr/local/include/IceUtil
  # windows
  C:/Ice-3.3.0-VC90/include/IceUtil
  C:/Ice-3.3.0-VC80/include/IceUtil
  C:/Ice-3.3.0/include/IceUtil
  C:/Ice-3.2.1-VC80/include/IceUtil
  C:/Ice-3.2.1/include/IceUtil
  C:/Ice-3.2.0-VC80/include/IceUtil
  C:/Ice-3.2.0/include/IceUtil
)

# NOTE: if ICEUTIL_HOME_INCLUDE_ICE is set to *-NOTFOUND it will evaluate to FALSE
if( iceutil_home_include_iceutil_dir )

    set( ICEUTIL_FOUND 1 CACHE BOOL "Do we have Ice?" FORCE )

    # strip 'file' twice to get rid off 'include/IceUtil'
#     message( STATUS "DEBUG: ICEUTIL_HOME_INCLUDE_ICE=" ${ICEUTIL_HOME_INCLUDE_ICE} )
    get_filename_component( iceutil_home_include_dir ${iceutil_home_include_iceutil_dir} PATH )
#     message( STATUS "DEBUG: iceutil_home_include_dir=" ${iceutil_home_include_dir} )
    get_filename_component( ICEUTIL_HOME ${iceutil_home_include_dir} PATH CACHE )
#     message( STATUS "Setting ICEUTIL_HOME to ${ICEUTIL_HOME}" )

endif( iceutil_home_include_iceutil_dir )

if( ICEUTIL_FOUND )
    message( STATUS "Looking for libIceUtil - found in ${ICEUTIL_HOME}")
else( ICEUTIL_FOUND )
    message( STATUS "Looking for libIceUtil - not found")
endif( ICEUTIL_FOUND )
