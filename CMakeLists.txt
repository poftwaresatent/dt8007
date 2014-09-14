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
