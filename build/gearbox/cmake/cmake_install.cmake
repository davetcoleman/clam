# Install script for directory: /home/dave/ros/clam/src/gearbox/cmake

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/dave/ros/clam/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gearbox/cmake" TYPE FILE FILES
    "/home/dave/ros/clam/src/gearbox/cmake/SetupDirectories.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/UseGearbox.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/SetupProjectName.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/TargetUtils.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/FindIceUtil.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/SetupBuildType.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/DependencyUtils.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/gearbox-use-file.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/SetupOs.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/CheckCompiler.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/WritePackageConfig.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/StringToCapital.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/UseBasicRules.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/Assert.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/WriteConfigH.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/FindGearbox.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/SetupVersion.cmake"
    "/home/dave/ros/clam/src/gearbox/cmake/UseIceUtil.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

