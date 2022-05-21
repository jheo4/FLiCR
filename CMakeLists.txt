cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(FLICR CXX)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(PkgConfig REQUIRED)

# OS platform check
if (NOT UNIX AND NOT ${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  message(STATUS "Not support your OS platform")
endif(NOT UNIX AND NOT ${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

add_definitions(-D__SILENCE__=0) # set 1 for silence

# PCC compile info
set(FLICR_CXX_FLAGS "")
set(FLICR_LINKER_FLAGS "")
set(FLICR_LINKER_LIBS "")

# Include cmake modules
include_directories("include")
include(cmake/boost.cmake)
include(cmake/rosbag.cmake)
include(cmake/yaml-cpp.cmake)
include(cmake/opencv.cmake)
include(cmake/ffmpeg.cmake)
include(cmake/pcl.cmake)
include(cmake/spdlog.cmake)
include(cmake/openmp.cmake)

message(STATUS "[FLICR COMPILE INFO]")

message("\t FLICR_CXX_FLAGS: ${FLICR_CXX_FLAGS}")
message("\t FLICR_LINKER_FLAGS: ${FLICR_LINKER_FLAGS}")
message("\t FLICR_LINKER_LIBS: ${FLICR_LINKER_LIBS}")

add_subdirectory(flicr_include)
add_subdirectory(source)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
add_subdirectory(example)

# Install
string(REPLACE ";" " " FLICR_CXX_FLAGS    "${FLICR_CXX_FLAGS}")
string(REPLACE ";" " " FLICR_LINKER_FLAGS "${FLICR_LINKER_FLAGS}")
string(REPLACE ";" " " FLICR_LINKER_LIBS  "${FLICR_LINKER_LIBS}")

set(FLICR_HEADER flicr)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/${FLICR_HEADER} DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

configure_file("flicr.pc.in" "flicr.pc" @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/flicr.pc DESTINATION /usr/local/lib/pkgconfig/)

