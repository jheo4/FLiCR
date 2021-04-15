macro(use_realsense2)
  pkg_search_module(REALSENSE2 REQUIRED realsense2)
  if(REALSENSE2_FOUND)
    message(STATUS "[REALSENSE2 Details]")
    message("\t REALSENSE2_INCLUDE_DIRS: ${REALSENSE2_INCLUDE_DIRS}")
    message("\t REALSENSE2_LDFLAGS: ${REALSENSE2_LDFLAGS}")

    include_directories(${REALSENSE2_INCLUDE_DIRS})
    list(APPEND PCC_CXX_FLAGS ${REALSENSE2_CFLAGS_OTHER})
    list(APPEND PCC_LINKER_FLAGS ${REALSENSE2_LDFLAGS})
  endif(REALSENSE2_FOUND)
endmacro()

use_realsense2()

