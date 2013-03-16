set (libName hokuyo_aist)
set (libDesc "Hokuyo laser scanner driver")
set (libVersion 2.0.0)
GBX_ADD_LICENSE (EPL1.0)

set (build TRUE)
GBX_REQUIRE_OPTION (build LIB ${libName} ON)

set (reqLibs flexiport)
GBX_REQUIRE_LIBS (build LIB ${libName} ${reqLibs})

if (build)
    include (${GBX_CMAKE_DIR}/UseBasicRules.cmake)

    set (hdrs hokuyo_aist.h
              hokuyo_errors.h
              sensor_info.h
              scan_data.h
              sensor.h
              utils.h)
    set (srcs hokuyo_errors.cpp
              sensor_info.cpp
              scan_data.cpp
              sensor.cpp)

    if (WIN32)
        if (GBX_DEFAULT_LIB_TYPE STREQUAL SHARED)
            add_definitions (-DHOKUYO_AIST_EXPORTS)
        else (GBX_DEFAULT_LIB_TYPE STREQUAL SHARED)
            add_definitions (-DHOKUYO_AIST_STATIC -DFLEXIPORT_STATIC)
        endif (GBX_DEFAULT_LIB_TYPE STREQUAL SHARED)
    endif (WIN32)

    include (CheckFunctionExists)
    set (CMAKE_REQUIRED_INCLUDES time.h)
    set (CMAKE_REQUIRED_LIBRARIES rt)
    CHECK_FUNCTION_EXISTS (clock_gettime HAVE_CLOCK_GETTIME)
    set (CMAKE_REQUIRED_INCLUDES)
    set (CMAKE_REQUIRED_LIBRARIES)

    GBX_ADD_LIBRARY (${libName} DEFAULT ${libVersion} ${srcs})
    target_link_libraries (${libName} ${reqLibs})
    if (HAVE_CLOCK_GETTIME)
        target_link_libraries (${libName} rt)
    endif (HAVE_CLOCK_GETTIME)
    GBX_ADD_PKGCONFIG (${libName} ${libDesc} "" reqLibs "" "" ${libVersion})

    GBX_ADD_HEADERS (${libName} ${hdrs})

    add_subdirectory (getid)
    if (GBX_BUILD_TESTS)
        add_subdirectory (test)
    endif (GBX_BUILD_TESTS)

    option (HOKUYO_AIST_BUILD_BINDINGS "Build the Python bindings for hokuyo_aist" ON)
    if (HOKUYO_AIST_BUILD_BINDINGS)
        add_subdirectory (python)
    endif (HOKUYO_AIST_BUILD_BINDINGS)
endif (build)
