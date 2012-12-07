# Find the Python libraries and headers
FIND_PACKAGE (PythonLibs)
if(NOT PYTHON_LIBRARIES)
	MESSAGE (STATUS "Python libaries not found. Cannot build Python bindings for Flexiport.")
endif(NOT PYTHON_LIBRARIES)

# Find Boost::Python
# There is a new, much better, FindBoost.cmake since 2.6
if(CMAKE_MAJOR_VERSION EQUAL 2 AND CMAKE_MINOR_VERSION GREATER 5)
	OPTION (Boost_USE_STATIC_LIBS "Use the static versions of the Boost libraries" OFF)
	MARK_AS_ADVANCED (Boost_USE_STATIC_LIBS)

	SET (BOOST_COMPONENTS python)
	FIND_PACKAGE (Boost COMPONENTS ${BOOST_COMPONENTS})
	if(Boost_FOUND)
		INCLUDE_DIRECTORIES (${Boost_INCLUDE_DIR})
		LINK_DIRECTORIES (${Boost_LIBRARY_DIRS})

		if(Boost_PYTHON_FOUND)
			GET_FILENAME_COMPONENT (boostPythonLib ${Boost_PYTHON_LIBRARY} NAME_WE CACHE)
			# Chop off the lib at the front, too, if present
			STRING (REGEX REPLACE "^lib" "" boostPythonLib ${boostPythonLib})
		else(Boost_PYTHON_FOUND)
			MESSAGE (STATUS
				"Boost::Python library was not found. Cannot build Python bindings for Flexiport.")
		endif(Boost_PYTHON_FOUND)
	else(Boost_FOUND)
		MESSAGE (STATUS
				"Boost libraries were not found. Cannot build Python bindings for Flexiport.")
	endif(Boost_FOUND)
else(CMAKE_MAJOR_VERSION EQUAL 2 AND CMAKE_MINOR_VERSION GREATER 5)
	FIND_PACKAGE (Boost)
	if(Boost_FOUND)
		# For 2.4, assume that if boost is found then boost::python is present
		OPTION (Boost_USE_MULTITHREAD "Use the multithreaded versions of the Boost libraries" ON)
		MARK_AS_ADVANCED (Boost_USE_MULTITHREAD)
		if(Boost_USE_MULTITHREAD)
			SET (BOOST_LIB_SUFFIX "-mt" CACHE STRING "Boost library name suffix")
		else(Boost_USE_MULTITHREAD)
			SET (BOOST_LIB_SUFFIX "" CACHE STRING "Boost library name suffix")
		endif(Boost_USE_MULTITHREAD)
		MARK_AS_ADVANCED (BOOST_LIB_SUFFIX)

		SET (boostPythonLib boost_python${BOOST_LIB_SUFFIX})
		INCLUDE_DIRECTORIES (${Boost_INCLUDE_DIRS})
		LINK_DIRECTORIES (${Boost_LIBRARY_DIRS})
	else(Boost_FOUND)
		MESSAGE (STATUS
				"Boost libraries were not found. Cannot build Python bindings for Flexiport.")
	endif(Boost_FOUND)
endif(CMAKE_MAJOR_VERSION EQUAL 2 AND CMAKE_MINOR_VERSION GREATER 5)

if(PYTHON_LIBRARIES AND Boost_FOUND AND GBX_DEFAULT_LIB_TYPE STREQUAL SHARED)
	MESSAGE (STATUS "Flexiport Python bindings will be built.")
	SET (srcs	flexiport.cpp logreaderport.cpp logwriterport.cpp port.cpp
				serialport.cpp tcpport.cpp timeout.cpp)

	SET (pyModuleTarget flexiportpy)
	INCLUDE_DIRECTORIES (${PROJECT_SOURCE_DIR}/src/flexiport ${PYTHON_INCLUDE_PATH})
	add_library(${pyModuleTarget} MODULE ${srcs})
	TARGET_LINK_LIBRARIES (${pyModuleTarget} flexiport ${boostPythonLib} ${PYTHON_LIBRARIES})
	SET_TARGET_PROPERTIES (${pyModuleTarget} PROPERTIES PREFIX "" OUTPUT_NAME "flexiport")
	install(TARGETS ${pyModuleTarget} LIBRARY DESTINATION lib/python/site-packages)

	ADD_SUBDIRECTORY (test)
ELSEIF(NOT GBX_DEFAULT_LIB_TYPE STREQUAL SHARED)
	MESSAGE (STATUS "Flexiport Python bindings will not be built - must build flexiport as a shared library.")
endif(PYTHON_LIBRARIES AND Boost_FOUND AND GBX_DEFAULT_LIB_TYPE STREQUAL SHARED)
