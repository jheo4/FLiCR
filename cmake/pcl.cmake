macro(use_pcl)
  find_package(PCL 1.3 REQUIRED)
  if(PCL_FOUND)
    message(STATUS "[PCL Details]")
    message("\t PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
    message("\t PCL_LDFLAGS: ${PCL_LDFLAGS}")
    message("\t PCL_LIBRARIES: ${PCL_LIBRARIES}")
    message("\t PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")
    message("\t PCL_DEFINITIONS: ${PCL_DEFINITIONS}")

    include_directories(${PCL_INCLUDE_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    list(APPEND FLICR_CXX_FLAGS ${PCL_CFLAGS_OTHER})
    list(APPEND FLICR_LINKER_FLAGS ${PCL_LIBRARIES})
  endif(PCL_FOUND)
endmacro()

use_pcl()

