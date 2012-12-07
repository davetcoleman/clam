#
# GBX_ASSERT( TEST COMMENT_FAIL [COMMENT_PASS=''] [IS_FATAL=FALSE] )
#
macro( GBX_ASSERT TEST COMMENT_FAIL )

    if( ${TEST} )
#         message( STATUS "DEBUG: assertion passed : ${TEST}" )

        # ARG2 holds COMMENT_PASS
        if( ${ARGC} GREATER 2 )
            message( STATUS ${ARGV2} )
        endif( ${ARGC} GREATER 2 )

    else( ${TEST} )
#         message( STATUS "DEBUG: assertion failed : ${TEST}" )

        set( IS_FATAL 0 )
        if( ${ARGC} GREATER 3 )
            set( IS_FATAL ${ARGV3} )
        endif( ${ARGC} GREATER 3 )

        if( ${IS_FATAL} )
#             message( STATUS "DEBUG: failure is fatal : ${IS_FATAL}" )
            message( FATAL_ERROR ${COMMENT_FAIL} )
        else( ${IS_FATAL} )
#             message( STATUS "DEBUG: failure is NOT fatal : ${IS_FATAL}" )
            message( STATUS ${COMMENT_FAIL} )
        endif( ${IS_FATAL} )

    endif( ${TEST} )

endmacro( GBX_ASSERT )
