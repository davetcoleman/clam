#
# If we're using gcc, make sure the version is OK.
#
if( ${CMAKE_C_COMPILER} MATCHES gcc )

    exec_program ( ${CMAKE_C_COMPILER} ARGS --version OUTPUT_VARIABLE gcc_version )
    message( STATUS "gcc version: ${gcc_version}")

    # Why doesn't this work?
    #string( REGEX MATCHALL "gcc\.*" VERSION_STRING ${CMAKE_C_COMPILER} )

    if( gcc_version MATCHES ".*4\\.[0-9]\\.[0-9]" )
        set( GCC_VERSION_OK 1 )
    endif( gcc_version MATCHES ".*4\\.[0-9]\\.[0-9]")

    GBX_ASSERT ( GCC_VERSION_OK
      "Checking gcc version - failed. ${PROJECT_NAME} requires gcc v. 4.x"
      "Checking gcc version - ok"
      1 )
    
    if( gcc_version MATCHES ".*4\\.0.*" )
      # gcc 4.0.x
    endif( gcc_version MATCHES ".*4\\.0.*" )

    if( gcc_version MATCHES ".*4\\.1.*" )
      # gcc 4.1.x
      # gcc-4.1 adds stack protection, which makes code robust to buffer-overrun attacks
      #      (see: http://www.trl.ibm.com/projects/security/ssp/)
      # However for some reason this can result in the symbol '__stack_chk_fail_local' not being found.
      # So turn it off.
      # Tobi: it looks like stack protection is off by default from version gcc 4.1.2, so we don't need this any more.
      # Will keep it for now, it doesn't hurt.
      add_definitions( -fno-stack-protector )
    endif( gcc_version MATCHES ".*4\\.1.*" )


endif( ${CMAKE_C_COMPILER} MATCHES gcc )
