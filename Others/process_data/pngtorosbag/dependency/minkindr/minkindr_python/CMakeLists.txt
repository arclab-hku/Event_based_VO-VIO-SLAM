cmake_minimum_required(VERSION 2.8.3)
project(minkindr_python)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set (CMAKE_CXX_FLAGS "-Wall  ${CMAKE_CXX_FLAGS}")

# Enable C++11 support.
if (CMAKE_VERSION VERSION_LESS "3.1")
  if (MSVC)
    message(FATAL_ERROR "CMake version 3.1 or later is required to compiler ${PROJECT_NAME} with Microsoft Visual C++")
  endif ()
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set (CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
  else ()
    set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
  endif ()
else ()
  set (CMAKE_CXX_STANDARD 11)
endif ()

# Set up the python exports.
SET(PY_PROJECT_NAME minkindr_python)
SET(PY_PACKAGE_DIR python/minkindr)

add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
  src/module.cpp
  src/export-rotation-quaternion.cc
  src/export-transformation.cc
)

cs_install()
cs_export()
