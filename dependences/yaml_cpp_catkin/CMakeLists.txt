cmake_minimum_required(VERSION 2.8.3)
project(yaml_cpp_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

# Release versions currently don't use C++11 and create more compiler warnings.
set(YAML_CPP_TAG 11607eb5bf1258641d80f7051e7cf09e317b4746)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(yaml_cpp_src
  GIT_REPOSITORY  https://github.com/jbeder/yaml-cpp
  GIT_TAG ${YAML_CPP_TAG}
  UPDATE_COMMAND ""
  PATCH_COMMAND patch -p0 < ${CMAKE_SOURCE_DIR}/extra_version.patch
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX} -DBUILD_SHARED_LIBS=ON
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/yaml-cpp
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING PATTERN "libyaml-cpp*")
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
          LIBRARIES yaml-cpp0.5)
