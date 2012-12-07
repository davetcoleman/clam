# Find the Python libraries and headers
FIND_PACKAGE (PythonLibs)
if(NOT PYTHON_LIBRARIES)
    MESSAGE (STATUS "Python libaries not found. Cannot build Python bindings for Flexiport.")
endif(NOT PYTHON_LIBRARIES)

# Find Boost::Python
FIND_PACKAGE (Boost)
if(Boost_FOUND)
    SET (boostPythonLib boost_python)
    INCLUDE_DIRECTORIES (${Boost_INCLUDE_DIRS})
    LINK_DIRECTORIES (${Boost_LIBRARY_DIRS})
else(Boost_FOUND)
    MESSAGE (STATUS
             "Boost libraries were not found. Cannot build Python bindings for Flexiport.")
endif(Boost_FOUND)

if(PYTHON_LIBRARIES AND Boost_FOUND)
    MESSAGE (STATUS "Dynamixel Python bindings will be built.")
    SET (srcs dynamixel_io.cpp)
    SET (pyModuleTarget dynamixeliopy)

    INCLUDE_DIRECTORIES (${PROJECT_SOURCE_DIR}/include/dynamixel_hardware_interface ${PYTHON_INCLUDE_PATH})
    add_library(${pyModuleTarget} MODULE ${srcs})
    TARGET_LINK_LIBRARIES (${pyModuleTarget} dynamixel_hardware_interface ${boostPythonLib} ${PYTHON_LIBRARIES})
    SET_TARGET_PROPERTIES (${pyModuleTarget} PROPERTIES PREFIX "" OUTPUT_NAME "dynamixel_io" COMPILE_FLAGS -fPIC)
endif(PYTHON_LIBRARIES AND Boost_FOUND)
