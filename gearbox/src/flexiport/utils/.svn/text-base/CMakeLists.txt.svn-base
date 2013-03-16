INCLUDE (${GBX_CMAKE_DIR}/UseBasicRules.cmake)

if(NOT WIN32)
	GBX_ADD_EXECUTABLE(porttoport porttoport.cpp)
	TARGET_LINK_LIBRARIES (porttoport flexiport)

GBX_ADD_EXAMPLE (flexiport/utils utils.cmake.in utils.cmake
	porttoport.cpp utils.readme)
endif(NOT WIN32)
