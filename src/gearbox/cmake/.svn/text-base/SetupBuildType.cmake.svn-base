if( NOT CMAKE_BUILD_TYPE )

  if( NOT GBX_OS_WIN )
    # For gcc, RelWithDebInfo gives '-O2 -g'
    set( CMAKE_BUILD_TYPE RelWithDebInfo )
  else( NOT GBX_OS_WIN )
    # windows... a temp hack: VCC does not seem to respect the cmake
    # setting and always defaults to debug, we have to match it here.
    set( CMAKE_BUILD_TYPE Debug )
  endif( NOT GBX_OS_WIN )

  message( STATUS "Setting build type to '${CMAKE_BUILD_TYPE}'" )

else( NOT CMAKE_BUILD_TYPE )

  message( STATUS "Build type set to '${CMAKE_BUILD_TYPE}' by user." )

endif( NOT CMAKE_BUILD_TYPE )
