cmake_minimum_required(VERSION 2.8.3)
project(cnpy_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(
  cnpy_src
  URL https://github.com/rogersce/cnpy/archive/4e8810b1a8637695171ed346ce68f6984e585ef4.zip
  URL_MD5 5d31154534fb738bd5721ac1604a1ecf
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND cd ../cnpy_src &&
     cmake . -G "Unix Makefiles"
       -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
  BUILD_COMMAND cd ../cnpy_src && make
  INSTALL_COMMAND cd ../cnpy_src && make install
)

cs_add_library(${PROJECT_NAME} src/dependency_tracker.cc)
add_dependencies(${PROJECT_NAME} cnpy_src)
target_link_libraries(${PROJECT_NAME} 
  ${CATKIN_DEVEL_PREFIX}/lib/libcnpy${CMAKE_SHARED_LIBRARY_SUFFIX})

cs_install()

cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
  CFG_EXTRAS cnpy-extras.cmake)
