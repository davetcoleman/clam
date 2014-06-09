set (libName flexiport)
set (libDesc "FlexiPort generic comms library")
set (libVersion 1.0.0)
GBX_ADD_LICENSE (LGPL3)

set (build TRUE)
GBX_REQUIRE_OPTION (build LIB ${libName} ON)

if (build)
	include (${GBX_CMAKE_DIR}/UseBasicRules.cmake)

	option (FLEXIPORT_INCLUDE_SERIAL "Include the serial port in FlexiPort" ON)
	option (FLEXIPORT_INCLUDE_TCP "Include the TCP network port in FlexiPort" ON)
	option (FLEXIPORT_INCLUDE_UDP "Include the UDP network port in FlexiPort" ON)
	option (FLEXIPORT_INCLUDE_LOGGING "Include the log reader/writer ports in FlexiPort" ON)
	mark_as_advanced (FLEXIPORT_INCLUDE_SERIAL FLEXIPORT_INCLUDE_TCP FLEXIPORT_INCLUDE_UDP)

	if (GBX_OS_QNX)
		set (CMAKE_REQUIRED_LIBRARIES socket)
	endif (GBX_OS_QNX)
	check_function_exists (getaddrinfo FLEXIPORT_HAVE_GETADDRINFO)
	set (CMAKE_REQUIRED_LIBRARIES)

	set (flexiport_config_h_in ${CMAKE_CURRENT_SOURCE_DIR}/flexiport_config.h.in)
	set (flexiport_config_h ${CMAKE_CURRENT_BINARY_DIR}/flexiport_config.h)
	configure_file (${flexiport_config_h_in} ${flexiport_config_h})
	include_directories (${CMAKE_CURRENT_BINARY_DIR})

	set (hdrs flexiport.h port.h timeout.h flexiport_types.h ${flexiport_config_h})
	set (srcs flexiport.cpp port.cpp timeout.cpp ${flexiport_config_h})
	if (FLEXIPORT_INCLUDE_SERIAL)
		set (hdrs ${hdrs} serialport.h)
		set (srcs ${srcs} serialport.cpp)
	endif (FLEXIPORT_INCLUDE_SERIAL)
	if (FLEXIPORT_INCLUDE_TCP)
		set (hdrs ${hdrs} tcpport.h)
		set (srcs ${srcs} tcpport.cpp)
	endif (FLEXIPORT_INCLUDE_TCP)
	if (FLEXIPORT_INCLUDE_UDP)
		set (hdrs ${hdrs} udpport.h)
		set (srcs ${srcs} udpport.cpp)
	endif (FLEXIPORT_INCLUDE_UDP)
	if (FLEXIPORT_INCLUDE_LOGGING)
		set (hdrs ${hdrs} logwriterport.h logreaderport.h)
		set (srcs ${srcs} logwriterport.cpp logreaderport.cpp logfile.cpp)
	endif (FLEXIPORT_INCLUDE_LOGGING)

	if (WIN32)
		if (GBX_DEFAULT_LIB_TYPE STREQUAL SHARED)
			add_definitions (-DFLEXIPORT_EXPORTS)
		else (GBX_DEFAULT_LIB_TYPE STREQUAL SHARED)
			add_definitions (-DFLEXIPORT_STATIC)
		endif (GBX_DEFAULT_LIB_TYPE STREQUAL SHARED)
	endif (WIN32)
	GBX_ADD_LIBRARY (${libName} DEFAULT ${libVersion} ${srcs})
	GBX_ADD_PKGCONFIG (${libName} ${libDesc} "" "" "" "" ${libVersion})
	GBX_ADD_HEADERS (${libName} ${hdrs})
	if (WIN32)
		target_link_libraries (${libName} Ws2_32)
	endif (WIN32)

	add_subdirectory (utils)
	if (GBX_BUILD_TESTS)
		add_subdirectory (test)
	endif (GBX_BUILD_TESTS)
	
	option (FLEXIPORT_BUILD_BINDINGS "Build the Python bindings for Flexiport" OFF)
	if (FLEXIPORT_BUILD_BINDINGS)
		add_subdirectory (python)
	endif (FLEXIPORT_BUILD_BINDINGS)




	MESSAGE( STATUS "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! DAVE")
	MESSAGE( STATUS "lib name:         " ${libName} )
	MESSAGE( STATUS "install to:         " ${CMAKE_INSTALL_PREFIX} )

	# added by DTC
	#install(TARGETS ${libName} LIBRARY DESTINATION "/usr/lib")

endif (build)
