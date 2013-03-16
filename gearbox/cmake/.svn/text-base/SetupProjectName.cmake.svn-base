# this is a copy of GBX_STRING_TO_CAPITAL in StringToCapital.cmake
# (making a copy so that we can use this file here and in derived projects)
macro( UTIL_STRING_TO_CAPITAL input_string OUTPUT_VAR )

    # jump through the hoops to construct project name string with the first character capitalized
    string( LENGTH ${input_string} string_length )

    math( EXPR string_length_minus_one "${string_length} - 1" )

    string( SUBSTRING ${input_string} 0 1 output_start )
    string( SUBSTRING ${input_string} 1 ${string_length_minus_one} output_rest )

    string( TOUPPER ${output_start} output_start_upper )
    string( TOLOWER ${output_rest} output_rest_lower )

    set( ${OUTPUT_VAR} "${output_start_upper}${output_rest_lower}" )

endmacro( UTIL_STRING_TO_CAPITAL input_string OUTPUT_VAR )

#
# We often use different variants of the project name.
# E.g. for project "gearbox" we want "GEARBOX", "gearbox", "Gearbox".
#

UTIL_STRING_TO_CAPITAL( ${PROJECT_NAME} GBX_PROJECT_NAME_CAP )
# message( STATUS "Setting capitalized project name to ${PROJECT_NAME_CAP}" )

string( TOUPPER ${PROJECT_NAME} GBX_PROJECT_NAME_UPPER )

string( TOLOWER ${PROJECT_NAME} GBX_PROJECT_NAME_LOWER )
