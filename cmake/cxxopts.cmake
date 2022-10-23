macro(use_cxxopts)
  pkg_search_module(CXXOPTS REQUIRED cxxopts)
  if(CXXOPTS_FOUND)
    message(STATUS "[CXXOPTS Details]")
    message("\t CXXOPTS_INCLUDE_DIRS: ${CXXOPTS_INCLUDE_DIRS}")
    message("\t CXXOPTS_LDFLAGS: ${CXXOPTS_LDFLAGS}")

    include_directories(${CXXOPTS_INCLUDE_DIRS})
    list(APPEND PCC_CXX_FLAGS ${CXXOPTS_CFLAGS_OTHER})
    list(APPEND PCC_LINKER_FLAGS ${CXXOPTS_LDFLAGS})
  endif(CXXOPTS_FOUND)
endmacro()

use_cxxopts()

