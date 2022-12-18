cmake_minimum_required(VERSION 2.8.3)
project(nanoflann_catkin)

find_package(catkin_simple REQUIRED)

catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

ExternalProject_Add(nanoflann_src
  GIT_REPOSITORY https://github.com/cfo/nanoflann.git
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND #cd ../nanoflann_src && cmake . -DCMAKE_CXX_FLAGS=-fPIC
  BUILD_COMMAND #cd ../nanoflann_src && make -j8
  INSTALL_COMMAND 
      #cp ../nanoflann_src/libgmock.a ${CATKIN_DEVEL_PREFIX}/lib/ &&
      #cp -rf ../nanoflann_src/include/gmock ${CATKIN_DEVEL_PREFIX}/include/ &&
      mkdir -p ${CATKIN_DEVEL_PREFIX}/include/nanoflann/ &&
      cp ../nanoflann_src/include/nanoflann.hpp ${CATKIN_DEVEL_PREFIX}/include/nanoflann/
)

cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)
