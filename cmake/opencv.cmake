macro(use_opencv)
  pkg_search_module(OPENCV REQUIRED opencv4)
  if(OPENCV_FOUND)
    message(STATUS "[OpenCV Details]")
    message("\t OPENCV_INCLUDE_DIRS: ${OPENCV_INCLUDE_DIRS}")
    message("\t OPENCV_LDFLAGS: ${OPENCV_LDFLAGS}")

    include_directories(${OPENCV_INCLUDE_DIRS})
    list(APPEND FLICR_CXX_FLAGS ${OPENCV_CFLAGS_OTHER})
    list(APPEND FLICR_LINKER_FLAGS ${OPENCV_LDFLAGS})
  endif(OPENCV_FOUND)
endmacro()

use_opencv()

