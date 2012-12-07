#
# Generate and install files to be used with CMake's PackageConfig system.
# (This is different from Linux's PkgConfig)
#

set( _input_dir ${PROJECT_SOURCE_DIR}/cmake/internal )
set( _output_dir ${PROJECT_BINARY_DIR} )
set( _destination ${GBX_CMAKE_PKGCONFIG_INSTALL_SUFFIX} )

set( _input_file config-external.cmake.in )
set( _output_file ${PROJECT_NAME}-config.cmake )
configure_file(
    ${_input_dir}/${_input_file}
    ${_output_dir}/${_output_file}
    @ONLY )
install(
    FILES ${_output_dir}/${_output_file}
    DESTINATION ${_destination} )

set( _input_file config-version.cmake.in )
set( _output_file ${PROJECT_NAME}-config-version.cmake )
configure_file(
    ${_input_dir}/${_input_file}
    ${_output_dir}/${_output_file}
    @ONLY )
install(
    FILES ${_output_dir}/${_output_file}
    DESTINATION ${_destination} )

# export targets (make sure there's something to export)
# TODO: it's not clear how to check if ${PROJECT_NAME}-targets is non-empty
# what is this thing? a list? does not seem to a variable?
# As it is now it will barf when all targets are disabled, e.g. with GBX_DISABLE_ALL=ON
# if( DEFINED ${PROJECT_NAME}-targets )
    install(
        EXPORT ${PROJECT_NAME}-targets
    #     NAMESPACE import_
        DESTINATION ${_destination} )
# endif()

set( _input_dir )
set( _output_dir )
set( _input_file )
set( _output_file )
set( _destination )
