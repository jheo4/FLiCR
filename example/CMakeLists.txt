file(GLOB_RECURSE EXAMPLE_SRCS *.cc *.cpp)

foreach(EXAMPLE_SRC ${EXAMPLE_SRCS})
  get_filename_component (NAME_WITHOUT_EXTENSION ${EXAMPLE_SRC} NAME_WE)
  set(EXAMPLE_NAME "${NAME_WITHOUT_EXTENSION}")

  add_executable(${EXAMPLE_NAME} ${EXAMPLE_SRC})
  target_include_directories(${EXAMPLE_NAME} PUBLIC
                            "${PROJECT_SOURCE_DIR}/include")

  target_compile_options(${EXAMPLE_NAME} PUBLIC ${PCC_CXX_FLAGS})
  target_link_libraries(${EXAMPLE_NAME} ${PCC_LINKER_FLAGS})
  target_link_libraries(${EXAMPLE_NAME} ${PCC_LINKER_LIBS})
  target_link_libraries(${EXAMPLE_NAME} pcc)
endforeach(EXAMPLE_SRC ${EXAMPLE_SRCS})

