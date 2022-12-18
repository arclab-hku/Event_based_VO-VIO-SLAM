cmake_minimum_required(VERSION 2.8.3)
project(catkin_boost_python_buildtool_test)

find_package(catkin_simple REQUIRED)
catkin_simple()

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
SET(PY_PROJECT_NAME catkin_boost_python_test)
SET(PY_PACKAGE_DIR python/catkin_boost_python_test)

add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
  src/module.cpp
)

cs_install()
cs_export()

catkin_add_nosetests( 
  test/test.py
)
