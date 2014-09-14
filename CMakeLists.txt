cmake_minimum_required (VERSION 2.6)
if (COMMAND cmake_policy)
  cmake_policy (SET CMP0003 NEW)
  cmake_policy (SET CMP0005 NEW)
  if (POLICY CMP0011)
    cmake_policy (SET CMP0011 NEW)
  endif (POLICY CMP0011)
  if (POLICY CMP0042)
    cmake_policy (SET CMP0042 NEW)
  endif (POLICY CMP0042)
endif (COMMAND cmake_policy)

project (DEIS)
set (CMAKE_VERBOSE_MAKEFILE ON)
if (NOT BUILD_SHARED_LIBS)
  set (BUILD_SHARED_LIBS True)
endif (NOT BUILD_SHARED_LIBS)
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

##################################################
# OS detection

if (WIN32)
  add_definitions (-DWIN32)
else (WIN32)
  if (APPLE)
    add_definitions (-DOSX)
    include_directories (
      /opt/local/include
      )
    list (APPEND CMAKE_REQUIRED_INCLUDES
      /opt/local/include
      )
    link_directories (
      /opt/local/lib
      )
  endif (APPLE)
  if (CMAKE_SYSTEM_NAME MATCHES Linux)
    add_definitions (-DLINUX)
  endif (CMAKE_SYSTEM_NAME MATCHES Linux)
  if (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    add_definitions (-DOPENBSD)
    include_directories (
      /usr/local/include
      )
    link_directories (
      /usr/local/lib
      )
  endif (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
endif(WIN32)

##################################################
# compiler flags (other than header and library paths)

include (CheckCXXCompilerFlag)

check_cxx_compiler_flag (-pipe CXX_FLAG_pipe)
if (CXX_FLAG_pipe)
  add_definitions (-pipe)
endif (CXX_FLAG_pipe)
check_cxx_compiler_flag (-Wall CXX_FLAG_Wall)
if (CXX_FLAG_Wall)
  add_definitions (-Wall)
endif (CXX_FLAG_Wall)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  check_cxx_compiler_flag (-O0 CXX_FLAG_O0)
  if (CXX_FLAG_O0)
    add_definitions (-O0)
  endif (CXX_FLAG_O0)
endif (${CMAKE_BUILD_TYPE} STREQUAL "Debug")

include (CheckCCompilerFlag)

check_c_compiler_flag (-pipe C_FLAG_pipe)
if (C_FLAG_pipe)
  add_definitions (-pipe)
endif (C_FLAG_pipe)
check_c_compiler_flag (-Wall C_FLAG_Wall)
if (C_FLAG_Wall)
  add_definitions (-Wall)
endif (C_FLAG_Wall)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  check_c_compiler_flag (-O0 C_FLAG_O0)
  if (C_FLAG_O0)
    add_definitions (-O0)
  endif (C_FLAG_O0)
endif (${CMAKE_BUILD_TYPE} STREQUAL "Debug")

##################################################
# configure-time checks

include (FindOpenGL)
if (OPENGL_FOUND)
  message ("*** found OpenGL (required)")
else (OPENGL_FOUND)
  message (FATAL_ERROR "Please install OpenGL.")
endif (OPENGL_FOUND)
if (OPENGL_GLU_FOUND)
  message ("*** found GLU (required)")
else (OPENGL_GLU_FOUND)
  message (FATAL_ERROR "Please install GLU.")
endif (OPENGL_GLU_FOUND)

include (FindGLUT)
if (GLUT_FOUND)
  message ("*** found GLUT (required)")
else (GLUT_FOUND)
  message (FATAL_ERROR "Please install GLUT.")
endif (GLUT_FOUND)

##################################################

subdirs (sfl2)

include_directories (
  sfl2
  sfl2/3rdparty/fpplib/include
  sfl2/3rdparty/fpplib/yaml-cpp-0.2.7/include
  )

add_library (deis-plugin MODULE deis-plugin.cpp student-code.c)
target_link_libraries (deis-plugin npm2)

add_library (splinefollow-plugin MODULE splinefollow-plugin.cpp)
target_link_libraries (splinefollow-plugin npm2)
