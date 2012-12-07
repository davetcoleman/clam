#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
SET(CMAKE_IMPORT_FILE_VERSION 1)

# Compute the installation prefix relative to this file.
GET_FILENAME_COMPONENT(_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
GET_FILENAME_COMPONENT(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
GET_FILENAME_COMPONENT(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)

# Import target "flexiport" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET flexiport APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(flexiport PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib64/gearbox/libflexiport.so.1.0.0"
  IMPORTED_SONAME_RELWITHDEBINFO "libflexiport.so.1.0.0"
  )

LIST(APPEND _IMPORT_CHECK_TARGETS flexiport )
LIST(APPEND _IMPORT_CHECK_FILES_FOR_flexiport "${_IMPORT_PREFIX}/lib64/gearbox/libflexiport.so.1.0.0" )

# Import target "GbxLockFileAcfr" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET GbxLockFileAcfr APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(GbxLockFileAcfr PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib64/gearbox/libGbxLockFileAcfr.so.1.0.0"
  IMPORTED_SONAME_RELWITHDEBINFO "libGbxLockFileAcfr.so.1.0.0"
  )

LIST(APPEND _IMPORT_CHECK_TARGETS GbxLockFileAcfr )
LIST(APPEND _IMPORT_CHECK_FILES_FOR_GbxLockFileAcfr "${_IMPORT_PREFIX}/lib64/gearbox/libGbxLockFileAcfr.so.1.0.0" )

# Import target "GbxSerialAcfr" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET GbxSerialAcfr APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(GbxSerialAcfr PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "GbxLockFileAcfr"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib64/gearbox/libGbxSerialAcfr.so.1.0.0"
  IMPORTED_SONAME_RELWITHDEBINFO "libGbxSerialAcfr.so.1.0.0"
  )

LIST(APPEND _IMPORT_CHECK_TARGETS GbxSerialAcfr )
LIST(APPEND _IMPORT_CHECK_FILES_FOR_GbxSerialAcfr "${_IMPORT_PREFIX}/lib64/gearbox/libGbxSerialAcfr.so.1.0.0" )

# Import target "GbxUtilAcfr" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET GbxUtilAcfr APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(GbxUtilAcfr PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib64/gearbox/libGbxUtilAcfr.so.1.0.0"
  IMPORTED_SONAME_RELWITHDEBINFO "libGbxUtilAcfr.so.1.0.0"
  )

LIST(APPEND _IMPORT_CHECK_TARGETS GbxUtilAcfr )
LIST(APPEND _IMPORT_CHECK_FILES_FOR_GbxUtilAcfr "${_IMPORT_PREFIX}/lib64/gearbox/libGbxUtilAcfr.so.1.0.0" )

# Import target "GbxGarminAcfr" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET GbxGarminAcfr APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(GbxGarminAcfr PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "GbxUtilAcfr;GbxSerialAcfr"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib64/gearbox/libGbxGarminAcfr.so.1.0.0"
  IMPORTED_SONAME_RELWITHDEBINFO "libGbxGarminAcfr.so.1.0.0"
  )

LIST(APPEND _IMPORT_CHECK_TARGETS GbxGarminAcfr )
LIST(APPEND _IMPORT_CHECK_FILES_FOR_GbxGarminAcfr "${_IMPORT_PREFIX}/lib64/gearbox/libGbxGarminAcfr.so.1.0.0" )

# Import target "GbxNovatelUtilAcfr" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET GbxNovatelUtilAcfr APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(GbxNovatelUtilAcfr PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "GbxUtilAcfr;GbxSerialAcfr"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib64/gearbox/libGbxNovatelUtilAcfr.so.1.0.0"
  IMPORTED_SONAME_RELWITHDEBINFO "libGbxNovatelUtilAcfr.so.1.0.0"
  )

LIST(APPEND _IMPORT_CHECK_TARGETS GbxNovatelUtilAcfr )
LIST(APPEND _IMPORT_CHECK_FILES_FOR_GbxNovatelUtilAcfr "${_IMPORT_PREFIX}/lib64/gearbox/libGbxNovatelUtilAcfr.so.1.0.0" )

# Import target "GbxNovatelAcfr" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET GbxNovatelAcfr APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(GbxNovatelAcfr PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "GbxUtilAcfr;GbxSerialAcfr;GbxNovatelUtilAcfr"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib64/gearbox/libGbxNovatelAcfr.so.1.0.0"
  IMPORTED_SONAME_RELWITHDEBINFO "libGbxNovatelAcfr.so.1.0.0"
  )

LIST(APPEND _IMPORT_CHECK_TARGETS GbxNovatelAcfr )
LIST(APPEND _IMPORT_CHECK_FILES_FOR_GbxNovatelAcfr "${_IMPORT_PREFIX}/lib64/gearbox/libGbxNovatelAcfr.so.1.0.0" )

# Import target "hokuyo_aist" for configuration "RelWithDebInfo"
SET_PROPERTY(TARGET hokuyo_aist APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
SET_TARGET_PROPERTIES(hokuyo_aist PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELWITHDEBINFO "flexiport;rt"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib64/gearbox/libhokuyo_aist.so.2.0.0"
  IMPORTED_SONAME_RELWITHDEBINFO "libhokuyo_aist.so.2.0.0"
  )

LIST(APPEND _IMPORT_CHECK_TARGETS hokuyo_aist )
LIST(APPEND _IMPORT_CHECK_FILES_FOR_hokuyo_aist "${_IMPORT_PREFIX}/lib64/gearbox/libhokuyo_aist.so.2.0.0" )

# Loop over all imported files and verify that they actually exist
FOREACH(target ${_IMPORT_CHECK_TARGETS} )
  FOREACH(file ${_IMPORT_CHECK_FILES_FOR_${target}} )
    IF(NOT EXISTS "${file}" )
      MESSAGE(FATAL_ERROR "The imported target \"${target}\" references the file
   \"${file}\"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   \"${CMAKE_CURRENT_LIST_FILE}\"
but not all the files it references.
")
    ENDIF()
  ENDFOREACH()
  UNSET(_IMPORT_CHECK_FILES_FOR_${target})
ENDFOREACH()
UNSET(_IMPORT_CHECK_TARGETS)

# Cleanup temporary variables.
SET(_IMPORT_PREFIX)

# Commands beyond this point should not need to know the version.
SET(CMAKE_IMPORT_FILE_VERSION)
