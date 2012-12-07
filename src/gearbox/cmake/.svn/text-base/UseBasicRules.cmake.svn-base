#
# Make it easy to include headers from library directories.
# e.g. #include <somelib/somelib.h>
#
include_directories( ${GBX_PROJECT_SOURCE_DIR}/src )

#
# Do the same for the submitted directory when its compilation
# is enabled.
#
if( GBX_BUILD_SUBMITTED )
    include_directories( ${GBX_PROJECT_SOURCE_DIR}/submitted )
endif( GBX_BUILD_SUBMITTED )

#
# Platform-specific compiler and linker flags
#
if( NOT GBX_OS_WIN )
    add_definitions( "-Wall -Wnon-virtual-dtor" )
#
# AlexB: Using -Wconversion finds some real bugs, but turns up lots of
#        false positives, especially in gcc4.3.
#        (see: http://gcc.gnu.org/ml/gcc/2008-05/msg00363.html)
#
#    add_definitions( "-Wall -Wconversion" )
else( NOT GBX_OS_WIN )
    # With Visual C++, -Wall causes the compiler to spew thousands upon
    # thousands (I'm not kidding here) of warnings in the standard lib
    # headers that are supplied with the compiler. Woo.
    add_definitions( "-D_CRT_SECURE_NO_DEPRECATE" )
endif( NOT GBX_OS_WIN )
