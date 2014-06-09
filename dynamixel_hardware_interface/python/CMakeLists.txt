# Find the Python libraries and headers
find_package (PythonLibs)
if(NOT PYTHON_LIBRARIES)
    message (STATUS "Python libaries not found. Cannot build Python bindings for Flexiport.")
endif()

# Find Boost::Python
find_package (Boost)
if(Boost_FOUND)
    set (boostPythonLib boost_python)
    include_directories (${Boost_INCLUDE_DIRS})
    link_directories (${Boost_LIBRARY_DIRS})
else(Boost_FOUND)
    message (STATUS
             "Boost libraries were not found. Cannot build Python bindings for Flexiport.")
endif()

if(PYTHON_LIBRARIES AND Boost_FOUND)
    message (STATUS "Dynamixel Python bindings will be built.")
    set (srcs dynamixel_io.cpp)
    set (pyModuleTarget dynamixeliopy)

    INCLUDE_DIRECTORIES (${PROJECT_SOURCE_DIR}/include/dynamixel_hardware_interface ${PYTHON_INCLUDE_PATH})
    #include_directories (${PYTHON_INCLUDE_PATH})
    add_library(${pyModuleTarget} MODULE ${srcs})
    target_link_libraries (${pyModuleTarget} dynamixel_hardware_interface ${boostPythonLib} ${PYTHON_LIBRARIES})
    set_target_properties (${pyModuleTarget} PROPERTIES PREFIX "" OUTPUT_NAME "dynamixel_io" COMPILE_FLAGS -fPIC)
endif()
