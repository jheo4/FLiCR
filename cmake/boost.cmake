macro(use_boost)
  find_package(Boost COMPONENTS system filesystem REQUIRED)
  if(Boost_FOUND)
    message(STATUS "[Boost Details]")
    message("\t Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
    message("\t Boost_LDFLAGS: ${Boost_LDFLAGS}")

    include_directories(${Boost_INCLUDE_DIRS})
    list(APPEND PCC_CXX_FLAGS ${Boost_CFLAGS_OTHER})
    list(APPEND PCC_LINKER_FLAGS ${Boost_LDFLAGS})
  endif(Boost_FOUND)
endmacro()

use_boost()

