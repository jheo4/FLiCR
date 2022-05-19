macro(use_yaml_cpp)
  pkg_search_module(YAMLCPP REQUIRED yaml-cpp)
  if(YAMLCPP_FOUND)
    message(STATUS "[YAMLCPP Details]")
    message("\t YAMLCPP_INCLUDE_DIRS: ${YAMLCPP_INCLUDE_DIRS}")
    message("\t YAMLCPP_LDFLAGS: ${YAMLCPP_LDFLAGS}")

    include_directories(${YAMLCPP_INCLUDE_DIRS})
    list(APPEND FLICR_CXX_FLAGS ${YAMLCPP_CFLAGS_OTHER})
    list(APPEND FLICR_LINKER_FLAGS ${YAMLCPP_LDFLAGS})
  endif(YAMLCPP_FOUND)
endmacro()

use_yaml_cpp()

