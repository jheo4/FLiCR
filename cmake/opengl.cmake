macro(use_opengl)
  pkg_search_module(GLFW REQUIRED glfw3 glfw)
  if(GLFW_FOUND)
    message(STATUS "[GLFW Details]")
    message("\t GLFW_INCLUDE_DIRS: ${GLFW_INCLUDE_DIRS}")
    message("\t GLFW_LDFLAGS: ${GLFW_LDFLAGS}")

    list(APPEND PCC_CXX_FLAGS ${GLFW_CFLAGS_OTHER})
    list(APPEND PCC_LINKER_FLAGS ${GLFW_LDFLAGS})
  endif()


  pkg_search_module(OGL REQUIRED gl)
  if(OGL_FOUND)
    message(STATUS "[OpenGL Details]")
    message("\t OPENGL_INCLUDE_DIRS: ${OGL_INCLUDE_DIRS}")
    message("\t OPENGL_LDFLAGS: ${OGL_LDFLAGS}")
    list(APPEND PCC_CXX_FLAGS ${OGL_CFLAGS_OTHER})
    list(APPEND PCC_LINKER_FLAGS ${OGL_LDFLAGS})
  endif()


  pkg_search_module(EGL REQUIRED egl)
  if(EGL_FOUND)
    message(STATUS "[EGL Details]")
    message("\t EGL_INCLUDE_DIRS: ${EGL_INCLUDE_DIRS}")
    message("\t EGL_LDFLAGS: ${EGL_LDFLAGS}")

    list(APPEND PCC_CXX_FLAGS ${EGL_CFLAGS_OTHER})
    list(APPEND PCC_LINKER_FLAGS ${EGL_LDFLAGS})
  endif()


  pkg_search_module(GLEW REQUIRED glew)
  if(GLEW_FOUND)
    message(STATUS "[GLEW Details]")
    message("\t GLEW_INCLUDE_DIRS: ${GLEW_INCLUDE_DIRS}")
    message("\t GLEW_LDFLAGS: ${GLEW_LDFLAGS}")

    #message("\t GLEW_LIBRARIES: ${GLEW_LIBRARIES}")
    #list(APPEND 3D_PCC_LINKER_LIBS ${GLEW_LIBRARIES})
    list(APPEND PCC_CXX_FLAGS ${GLEW_CFLAGS_OTHER})
    list(APPEND PCC_LINKER_FLAGS ${GLEW_LDFLAGS};-lrt;-lm;-ldl;-lX11;-lpthread;-lxcb;-lXau;-lXdmcp)
  endif()

endmacro()

use_opengl()

