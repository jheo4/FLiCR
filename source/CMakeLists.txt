file(GLOB_RECURSE PCC_SRCS *.cc *.cpp)
message("\t PCC_SRCS: ${PCC_SRCS}")

add_library(pcc ${PCC_SRCS})
target_include_directories(pcc PUBLIC
                           "${PROJECT_SOURCE_DIR}"
                           "${PROJECT_SOURCE_DIR}/include")

target_compile_options(pcc PUBLIC "-W" "-Wall" "-fPIC")
target_compile_options(pcc PUBLIC ${PCC_CXX_FLAGS})
target_link_libraries(pcc ${PCC_LINKER_FLAGS})
target_link_libraries(pcc ${PCC_LINKER_LIBS})

