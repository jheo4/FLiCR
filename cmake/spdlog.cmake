macro(use_spdlog)
  pkg_search_module(SPDLOG REQUIRED spdlog)
  if(SPDLOG_FOUND)
    message(STATUS "SPDLOG Details")
    message("\t SPDLOG_INCLUDE_DIRS: ${SPDLOG_INCLUDE_DIRS}")
    message("\t SPDLOG_LDFLAGS: ${SPDLOG_LDFLAGS}")

    include_directories(${SPDLOG_INCLUDE_DIRS})
    list(APPEND FLICR_CXX_FLAGS ${SPDLOG_CFLAGS_OTHER})
    list(APPEND FLICR_LINKER_FLAGS ${SPDLOG_LIBRARIES})
  endif(SPDLOG_FOUND)
endmacro()

use_spdlog()

