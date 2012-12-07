# Libraries submitted for consideration for inclusion in Gearbox.

OPTION ( GBX_BUILD_SUBMITTED "Build submitted libraries" OFF )
MARK_AS_ADVANCED ( FORCE GBX_BUILD_SUBMITTED )

if( GBX_BUILD_SUBMITTED )

    message( STATUS "== SUBMITTED ==" )

    # When adding new directories, please maintain order of inter-dependencies.
    # Otherwise, maintain alphabetical order.
    # E.g. add_subdirectory( mydir )

endif( GBX_BUILD_SUBMITTED )
