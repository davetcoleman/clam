# GBX_STRING_TO_CAPITAL( input_string OUTPUT_VAR )
#
# E.g. converts "gearbox" to "Gearbox"
#
macro( GBX_STRING_TO_CAPITAL input_string OUTPUT_VAR )

    # jump through the hoops to construct project name string with the first character capitalized
    string( LENGTH ${input_string} string_length )

    math( EXPR string_length_minus_one "${string_length} - 1" )

    string( SUBSTRING ${input_string} 0 1 output_start )
    string( SUBSTRING ${input_string} 1 ${string_length_minus_one} output_rest )

    string( TOUPPER ${output_start} output_start_upper )
    string( TOLOWER ${output_rest} output_rest_lower )

    set( ${OUTPUT_VAR} "${output_start_upper}${output_rest_lower}" )

endmacro( GBX_STRING_TO_CAPITAL input_string OUTPUT_VAR )
