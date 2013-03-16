INCLUDE (${GBX_CMAKE_DIR}/UseBasicRules.cmake)

GBX_ADD_EXECUTABLE(hokuyo_aist_example example.cpp)
TARGET_LINK_LIBRARIES (hokuyo_aist_example hokuyo_aist flexiport)

GBX_ADD_EXAMPLE (hokuyo_aist example.cmake.in example.cmake
    example.cpp example.readme example_urg_04lx.logr example_urg_04lx.logw
    example_utm_30lx.logr example_utm_30lx.logw)

