cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(3DPCC CXX)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(PkgConfig REQUIRED)

# OS platform check
if (NOT UNIX AND NOT ${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  message(STATUS "Not support your OS platform")
endif(NOT UNIX AND NOT ${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

add_definitions(-D__SILENCE__=0) # set 1 for silence

# PCC compile info
set(PCC_CXX_FLAGS "")
set(PCC_LINKER_FLAGS "")
set(PCC_LINKER_LIBS "")

# Include cmake modules
include_directories("include")
include(cmake/rosbag.cmake)
include(cmake/realsense.cmake)
include(cmake/yaml-cpp.cmake)
include(cmake/opencv.cmake)
include(cmake/opengl.cmake)

message(STATUS "[PCC COMPILE INFO]")

message("\t PCC_CXX_FLAGS: ${PCC_CXX_FLAGS}")
message("\t PCC_LINKER_FLAGS: ${PCC_LINKER_FLAGS}")
message("\t PCC_LINKER_LIBS: ${PCC_LINKER_LIBS}")

add_subdirectory(source)
add_subdirectory(include)
add_subdirectory(example)

# Install
#string(REPLACE ";" " " 3DPCC_CXX_FLAGS "${3DPCC_CXX_FLAGS}")
#string(REPLACE ";" " " 3DPCC_LINKER_FLAGS "${3DPCC_LINKER_FLAGS}")
#string(REPLACE ";" " " 3DPCC_LINKER_LIBS "${3DPCC_LINKER_LIBS}")
#set(3DPCC_HEADER mxre)
#install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/${3DPCC_HEADER} DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

#if(NOT DEFINED ENV{PKG_CONFIG_PATH})
#  set( PKG_CONFIG_PATH "/usr/local/lib/pkgconfig")
#  message("PKG_CONFIG_PATH is not found. Set it with ${PKG_CONFIG_PATH}")
#endif(NOT DEFINED ENV{PKG_CONFIG_PATH})
#configure_file("3d_pcc.pc.in" "3d_pcc.pc" @ONLY)
#install(FILES ${CMAKE_CURRENT_BINARY_DIR}/3d_pcc.pc DESTINATION ${PKG_CONFIG_PATH}/)

