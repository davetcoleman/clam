# include( ${HYDRO_CMAKE_DIR}/UseBasicRules.cmake )
# include( ${CMAKE_CURRENT_SOURCE_DIR}/../UseHydroIceUtil.cmake )
link_libraries( GbxIceUtilAcfr )

add_executable( buffertest buffertest.cpp )
GBX_ADD_TEST( GbxIceUtilAcfr_BufferTest buffertest )

add_executable( storetest storetest.cpp )
GBX_ADD_TEST( GbxIceUtilAcfr_StoreTest storetest )

add_executable( notifytest notifytest.cpp )
GBX_ADD_TEST( GbxIceUtilAcfr_NotifyTest notifytest )

add_executable( threadtest threadtest.cpp )
GBX_ADD_TEST( GbxIceUtilAcfr_ThreadTest threadtest )

add_executable( safethreadtest safethreadtest.cpp )
GBX_ADD_TEST( GbxIceUtilAcfr_SafeThreadTest safethreadtest )
