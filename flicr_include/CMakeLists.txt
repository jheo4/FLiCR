list( APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake )

set( FLICR_INC_DIR "flicr_include" )

install( DIRECTORY ${CMAKE_SOURCE_DIR}/${FLICR_INC_DIR} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/flexr_include)

