cmake_minimum_required(VERSION 2.8.3)
project(ceres_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include_directories(${Eigen_INCLUDE_DIRS})

include(ExternalProject)

set(VERSION 1.14.0)

# The configure step fails at catkin_package() if this directory
# doesn't exist yet. Create it at configure time!
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

# Patch away the linking to the gomp OpenMP implementation, in case we use the Intel OpenMP implementation and Clang.
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set(GOMP_PATCH "")
else (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set(GOMP_PATCH "--dry-run")
endif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")

ExternalProject_Add(ceres_src
  GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver.git
  GIT_TAG ${VERSION}
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND cmake
    -DGFLAGS=ON
    -DGFLAGS_LIBRARY_DIR_HINTS=${gflags_catkin_LIBRARIES}
    -DGFLAGS_INCLUDE_DIR_HINTS=${gflags_catkin_INCLUDE_DIR}
    -DGLOG_INCLUDE_DIR_HINTS=${glog_catkin_INCLUDE_DIR}
    -DGLOG_LIBRARY_DIR_HINTS=${glog_catkin_LIBRARIES}
    -DBUILD_DOCUMENTATION=OFF
    -DSUITESPARSE_INCLUDE_DIR_HINTS=${suitesparse_PREFIX}/include/suitesparse
    -DSUITESPARSE_LIBRARY_DIR_HINTS=${suitesparse_PREFIX}/lib
    -DBUILD_EXAMPLES=OFF
    -DBUILD_TESTING=OFF
    -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX} -DBUILD_SHARED_LIBS=ON
    -DCMAKE_VERBOSE_MAKEFILE=ON
    -DCMAKE_BUILD_TYPE:STRING=Release
    -DCMAKE_CXX_FLAGS=-fPIC
    -DCMAKE_FIND_PACKAGE_NO_PACKAGE_REGISTRY=ON
    -DCMAKE_FIND_PACKAGE_NO_SYSTEM_PACKAGE_REGISTRY=ON
    ../ceres_src
  BUILD_COMMAND make -j8
  INSTALL_COMMAND make install -j8
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/ceres
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        FILES_MATCHING PATTERN "libceres*")
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
          LIBRARIES ceres)
