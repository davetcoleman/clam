INCLUDE (${GBX_CMAKE_DIR}/UseBasicRules.cmake)

GBX_ADD_EXECUTABLE(serial_example serial_example.cpp)
TARGET_LINK_LIBRARIES (serial_example flexiport)

GBX_ADD_EXECUTABLE(tcp_example tcp_example.cpp)
TARGET_LINK_LIBRARIES (tcp_example flexiport)

GBX_ADD_EXECUTABLE(udp_example udp_example.cpp)
TARGET_LINK_LIBRARIES (udp_example flexiport)

GBX_ADD_EXAMPLE (flexiport/example example.cmake.in example.cmake
	serial_example.cpp tcp_example.cpp udp_example.cpp example.readme example.logr example.logw)
