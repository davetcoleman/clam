#
# This is a utility macro for internal use.
# If module_type is not equal "EXE" or "LIB", prints error message and quits.
#
# Have not tested this macro when the variables is_* are not same in the calling context.
macro( GBX_UTIL_CHECK_MODULE_TYPE type is_exe is_lib )
#     message( STATUS "GBX_UTIL_CHECK_MODULE_TYPE type=${type}" )

    string( COMPARE EQUAL ${type} "EXE" is_exe )
    string( COMPARE EQUAL ${type} "LIB" is_lib )
    if( NOT is_exe AND NOT is_lib )
        message( FATAL_ERROR "In GBX_REQUIRE_* macros, module_type must be either 'EXE' or 'LIB'" )
    endif( NOT is_exe AND NOT is_lib )
    if( is_exe AND is_lib )
        message( FATAL_ERROR "In GBX_REQUIRE_* macros, module_type must be either 'EXE' or 'LIB'" )
    endif( is_exe AND is_lib )

endmacro( GBX_UTIL_CHECK_MODULE_TYPE type is_exe is_lib )

#
# This is a utility macro for internal use.
#
macro( GBX_UTIL_MAKE_OPTION_NAME option_name module_type module_name )
#     message( STATUS "GBX_UTIL_MAKE_OPTION_NAME [ option_name=${option_name}, MOT_TYPE=${module_type}, MOD_NAME=${module_name} ]" )

    GBX_UTIL_CHECK_MODULE_TYPE( ${module_type} is_exe is_lib )

    string( TOUPPER ${module_name} module_name_upper )
    # dereference the variable name once, so that we are setting the variable in the calling context!
    if( is_exe )
        set( ${option_name} "ENABLE_${module_name_upper}" )
    else( is_exe )
        set( ${option_name} "ENABLE_LIB_${module_name_upper}" )
    endif( is_exe )

#     message( STATUS "GBX_UTIL_MAKE_OPTION_NAME output: option_name=${${option_name}}" )
endmacro( GBX_UTIL_MAKE_OPTION_NAME option_name module_name )

#
# GBX_REQUIRE_OPTION( cumulative_var [EXE | LIB] module_name devel_option_value [option_name] [OPTION DESCRIPTION] )
#
# E.g.
# Initialize a variable first
#   set( build TRUE )
# Now set up and test option value
#   GBX_REQUIRE_OPTION ( build EXE localiser ON )
# This does the same thing
#   GBX_REQUIRE_OPTION ( build EXE localiser ON BUILD_LOCALISER )
#
macro( GBX_REQUIRE_OPTION cumulative_var module_type module_name devel_option_value )

    GBX_UTIL_CHECK_MODULE_TYPE( ${module_type} is_exe is_lib )

    if( ${ARGC} GREATER 5 )
        set( option_name ${ARGV6} )
    else()
        GBX_UTIL_MAKE_OPTION_NAME( option_name ${module_type} ${module_name} )
    endif()

    if( ${ARGC} GREATER 6 )
        set( option_descr ${ARGV7} )
    else()
        set( option_descr "disabled by user, use ccmake to enable" )
    endif()

    # only affects the default option for this target
    # individual targets can still be enabled
    if( GBX_DISABLE_ALL )
        set( default_option_value OFF )
        set( option_descr "disabled by GBX_DISABLE_ALL, use ccmake to enable" )
    else()
        set( default_option_value ${devel_option_value} )
    endif()

    # debug
#     message( STATUS
#         "GBX_REQUIRE_OPTION (CUM_VAR=${cumulative_var}, MOD_TYPE=${module_type}, MOD_NAME=${module_name}, devel_option_value=${devel_option_value}, OPT_NAME=${option_name}, OPT_DESC=${option_descr})" )

    # set up the option
    if( is_exe )
        option( ${option_name} "Try to build ${module_name}" ${default_option_value} )
    else( is_exe )
        option( ${option_name} "Try to build lib${module_name} library" ${default_option_value} )
    endif( is_exe )

    # add option to the list: this has nothing to do with the build system.
    # it is useful to have a text list of all options if you want to build a particular
    # configuration from the command line.
    set( templist ${OPTION_LIST} )
    # (escaping \)
    list( APPEND templist "${option_name}=${default_option_value}" )
    set( OPTION_LIST ${templist} CACHE INTERNAL "Global list of cmake options" FORCE )

    # must dereference both var and option names once (!) and IF will evaluate their values
    if( ${cumulative_var} AND NOT ${option_name}  )
        set( ${cumulative_var} FALSE )
        if( is_exe )
            GBX_NOT_ADD_EXECUTABLE( ${module_name} ${option_descr} )
        else( is_exe )
            GBX_NOT_ADD_LIBRARY( ${module_name} ${option_descr} )
        endif( is_exe )
    endif( ${cumulative_var} AND NOT ${option_name} )

endmacro( GBX_REQUIRE_OPTION cumulative_var module_type module_name default_option_value )

#
# GBX_REQUIRE_VAR ( cumulative_var [EXE | LIB] module_name test_var reason )
#
# E.g.
# Initialize a variable first
#   set( build TRUE )
# Now test the variable value
#   GBX_REQUIRE_VAR ( build LIB HydroStuff GOOD_TO_GO "good-to-go is no good" )
#
macro( GBX_REQUIRE_VAR cumulative_var module_type module_name test_var reason )

    # debug
#     message( STATUS "GBX_REQUIRE_VAR [ CUM_VAR=${cumulative_var}, MOD_TYPE=${module_type}, MOD_NAME=${module_name}, test_var=${${test_var}}, reason=${reason} ]" )

    GBX_UTIL_CHECK_MODULE_TYPE( ${module_type} is_exe is_lib )

    # must dereference both var names once (!) and IF will evaluate their values
    if( ${cumulative_var} AND NOT ${test_var}  )
        set( ${cumulative_var} FALSE )
        if( is_exe )
            GBX_NOT_ADD_EXECUTABLE( ${module_name} ${reason} )
        else( is_exe )
            GBX_NOT_ADD_LIBRARY( ${module_name} ${reason} )
        endif( is_exe )
    endif( ${cumulative_var} AND NOT ${test_var} )

endmacro( GBX_REQUIRE_VAR cumulative_var module_type module_name test_var reason )

#
# GBX_REQUIRE_LIB( cumulative_var [EXE | LIB] module_name target_name [reason] )
#
# Despite the name, this macro can enforce dependence on any target, not just a library.
#
# E.g.
# Initialize a variable first
#   set( build TRUE )
# Now set up and test option value
#   GBX_REQUIRE_LIB ( build EXE localiser HydroStuff )
#
macro( GBX_REQUIRE_LIB cumulative_var module_type module_name target_name )
    # debug
#     message( STATUS "GBX_REQUIRE_LIB [ CUM_VAR=${${cumulative_var}}, MOD_TYPE=${module_type}, MOD_NAME=${module_name}, target_name=${target_name} ]" )

    if( ${cumulative_var} )

        GBX_UTIL_CHECK_MODULE_TYPE( ${module_type} is_exe is_lib )
    
        if( ${ARGC} GREATER 5 )
            set( reason ${ARGV6} )
        else( ${ARGC} GREATER 5 )
            set( reason "${target_name} is not being built" )
        endif( ${ARGC} GREATER 5 )
        
        if( NOT TARGET ${target_name} )
            set( ${cumulative_var} FALSE )
            GBX_UTIL_MAKE_OPTION_NAME( option_name ${module_type} ${module_name} )
            if( is_exe )
                GBX_NOT_ADD_EXECUTABLE( ${module_name} ${reason} )
            else( is_exe )
                GBX_NOT_ADD_LIBRARY( ${module_name} ${reason} )
            endif( is_exe )
        endif( NOT TARGET ${target_name} )

    endif( ${cumulative_var} )

endmacro( GBX_REQUIRE_LIB cumulative_var module_type module_name target_name )

#
# GBX_REQUIRE_LIBS( cumulative_var [EXE | LIB] module_name TARGET0 [TARGET1 TARGET2 ...] )
#
# Despite the name, this macro can enforce dependence on a set of target, not just a set of libraries.
#
macro( GBX_REQUIRE_LIBS cumulative_var module_type module_name )
    # debug
#     message( STATUS "GBX_REQUIRE_LIBS [ CUM_VAR=${${cumulative_var}}, MOD_TYPE=${module_type}, MOD_NAME=${module_name}, target_names=${ARGN} ]" )

    if( ${ARGC} LESS 4 )
        message( FATAL_ERROR "GBX_REQUIRE_LIBS macro needs to at least one target name (${ARGC} params were given)." )
    endif( ${ARGC} LESS 4 )

    foreach( trgt ${ARGN} )
        GBX_REQUIRE_LIB( ${cumulative_var} ${module_type} ${module_name} ${trgt} )
    endforeach( trgt ${ARGN} )

endmacro( GBX_REQUIRE_LIBS cumulative_var module_type module_name )

#
# This is a utility macro for internal use.
#
macro( GBX_WRITE_OPTIONS )
    set( output_file ${GBX_PROJECT_BINARY_DIR}/${PROJECT_NAME}_options.cmake )
    write_file( ${output_file} "\# Autogenerated by CMake for ${PROJECT_NAME} project" )

    foreach( a ${OPTION_LIST} )
        write_file( ${output_file} "-D${a} \\" APPEND )
    endforeach( a ${LIB_LIST} )
endmacro( GBX_WRITE_OPTIONS )

#
# This is a utility macro for internal use.
# Reset global lists of components, libraries, etc.
#
macro( GBX_RESET_ALL_DEPENDENCY_LISTS )
    # message( STATUS "DEBUG: Resetting global dependency lists" )
    set( OPTION_LIST    "" CACHE INTERNAL "Global list of cmake options" FORCE )
endmacro( GBX_RESET_ALL_DEPENDENCY_LISTS )
