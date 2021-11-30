

macro(use_openmp)
  find_package(OpenMP)
  if (OPENMP_FOUND)
    message(STATUS "[OpenMP Details]")
    message("\t OpenMP_CXX_FLAGS: ${OpenMP_CXX_FLAGS}")
    message("\t OpenMP_C_FLAGS: ${OpenMP_C_FLAGS}")

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_C_FLAGS} ${OpenMP_CXX_FLAGS}")
  endif()
endmacro()

use_openmp()

