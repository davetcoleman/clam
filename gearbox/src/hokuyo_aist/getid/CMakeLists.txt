INCLUDE (${GBX_CMAKE_DIR}/UseBasicRules.cmake)

GBX_ADD_EXECUTABLE(hokuyo_aist_getid getid.cpp)
TARGET_LINK_LIBRARIES (hokuyo_aist_getid hokuyo_aist flexiport)

