# include( ${GBX_CMAKE_DIR}/UseBasicRules.cmake )

link_libraries( GbxUtilAcfr )

add_executable( mathtest mathtest.cpp )
GBX_ADD_TEST( GbxUtilAcfr_MathTest mathtest )

add_executable( tokenisetest tokenisetest.cpp )
GBX_ADD_TEST( GbxUtilAcfr_TokeniseTest tokenisetest )

add_executable( trivialtracertest trivialtracertest.cpp )
GBX_ADD_TEST( GbxUtilAcfr_TrivialTracerTest trivialtracertest )

add_executable( trivialstatustest trivialstatustest.cpp )
GBX_ADD_TEST( GbxUtilAcfr_TrivialStatusTest trivialstatustest )
