#
# Give feedback on custom entries
#
message( STATUS "Setting project name to ${PROJECT_NAME}" )
message( STATUS "Setting project version to ${GBX_PROJECT_VERSION}" )

#
# Set CMake policies
# For help on policy CMPxxxx: $ cmake --help-policy CMPxxxx
# 
if( COMMAND cmake_policy )
    if( POLICY CMP0011 )
        cmake_policy(SET CMP0011 NEW )
    endif( POLICY CMP0011 )
endif( COMMAND cmake_policy )

set( GBX_CMAKE_DIR ${${PROJECT_NAME}_SOURCE_DIR}/cmake CACHE INTERNAL "Location of CMake scripts" )

#
# Process project name
#
include( ${GBX_CMAKE_DIR}/SetupProjectName.cmake )

#
# Process version number
#
include( ${GBX_CMAKE_DIR}/SetupVersion.cmake )

#
# Determine OS, and make os-specific choices
#
include( ${GBX_CMAKE_DIR}/SetupOs.cmake )

#
# Project directories, including installation
#
include( ${GBX_CMAKE_DIR}/SetupDirectories.cmake )

#
# Set the build type (affects debugging symbols and optimization)
#
include( ${GBX_CMAKE_DIR}/SetupBuildType.cmake )


#
# Include internal macro definitions
#
include( ${GBX_CMAKE_DIR}/Assert.cmake )
include( ${GBX_CMAKE_DIR}/TargetUtils.cmake )
include( ${GBX_CMAKE_DIR}/DependencyUtils.cmake )

#
# Defaults for big source code switches
# (these are defaults. after the user modifies these in GUI they stay in cache)
#
option( GBX_BUILD_LICENSE  "Enables writing LICENCE file. For admins only." OFF )
mark_as_advanced( GBX_BUILD_LICENSE )

#
# check compiler type and version
#
include( ${GBX_CMAKE_DIR}/CheckCompiler.cmake )

#
# Defaults for big source code switches
# (these are defaults. after the user modifies these in GUI they stay in cache)
#
option( GBX_BUILD_TESTS    "Enables compilation of all tests" ON  )

#
# Look for low-level C headers, write defines to config.h
#
include( ${GBX_CMAKE_DIR}/WriteConfigH.cmake )

#
# Installation preferences
#
# CMake defaults
# see: \http://www.cmake.org/Wiki/CMake_RPATH_handling
#
# use, i.e. don't skip the full RPATH for the build tree
# set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
# set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

# the RPATH to be used when installing
set( CMAKE_INSTALL_RPATH ${GBX_LIB_INSTALL_DIR} )

# Enable testing by including the Dart module
# (must be done *before* entering source directories )
include(${CMAKE_ROOT}/Modules/Dart.cmake)
enable_testing()

#
# Enter the source tree
#
add_subdirectory( src )
add_subdirectory( submitted )
add_subdirectory( retired )

# Some cmake and shell scripts need to be installed
add_subdirectory( cmake )

#
# Experimental
#
include( ${GBX_CMAKE_DIR}/WritePackageConfig.cmake )

#
# Write results of CMake activity to file
#
# GBX_WRITE_MANIFEST()
# GBX_WRITE_OPTIONS()

#
# Print license information to a file
#
if( GBX_BUILD_LICENSE )
    GBX_WRITE_LICENSE()
endif( GBX_BUILD_LICENSE )

#
# Print results of CMake activity
#
GBX_CONFIG_REPORT( "Nothing special" )

#
# House-keeping, clear lists of targets, licenses, options, etc.
#
GBX_RESET_ALL_TARGET_LISTS()
GBX_RESET_ALL_DEPENDENCY_LISTS()
