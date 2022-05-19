file(GLOB_RECURSE FLICR_SRCS *.cc *.cpp)
message("\t FLICR_SRCS: ${FLICR_SRCS}")

add_library(flicr ${FLICR_SRCS})
target_include_directories(flicr PUBLIC
                                "${PROJECT_SOURCE_DIR}"
                                "${PROJECT_SOURCE_DIR}/flicr_include")

target_compile_options(flicr PUBLIC "-W" "-Wall" "-fPIC")
target_compile_options(flicr PUBLIC ${FLICR_CXX_FLAGS})
target_link_libraries(flicr ${FLICR_LINKER_FLAGS})
target_link_libraries(flicr ${FLICR_LINKER_LIBS})

