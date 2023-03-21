cmake_minimum_required(VERSION 2.8.3)
project(glog_catkin)

find_package(catkin_simple REQUIRED)
catkin_simple()

include(ExternalProject)

cmake_policy(SET CMP0053 OLD)
include(cmake/glog-catkin-extras.cmake.in)

# We can't check version numbers easily as the GLOG script doesn't export them.

# If set to OFF, will compile the given version of Glog.
# If set to ON, will use the system version of Glog.
# If set to AUTO, will try to use the syste version if there is one, and build otherwise.
set(USE_SYSTEM_GLOG "AUTO" CACHE INTERNAL "Whether to use the system version of Glog.")

# Check if the version is suitable, otherwise force downloading the newer one.
if(USE_SYSTEM_GLOG STREQUAL "AUTO" OR USE_SYSTEM_GLOG STREQUAL "ON")
  find_package(Glog QUIET)
  if(GLOG_FOUND)
    set(USE_SYSTEM_GLOG "ON")
    message(STATUS "Suitable Glog version found.")
  else()
    set(USE_SYSTEM_GLOG "OFF")
    message(STATUS "No suitable Glog version found.")
  endif()
endif()


if(USE_SYSTEM_GLOG STREQUAL "OFF")
  message(STATUS "Installing Glog from source!")

  file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

  set(VERSION 0.3.5)
  ExternalProject_Add(glog_src
    URL https://github.com/google/glog/archive/v${VERSION}.zip
    UPDATE_COMMAND ""
    PATCH_COMMAND patch -p0 < ${CMAKE_CURRENT_SOURCE_DIR}/fix-unused-typedef-warning.patch
    CONFIGURE_COMMAND cd ../glog_src/ && autoreconf -fi && ./configure --with-pic
      --with-gflags=${gflags_catkin_PREFIX}
      --prefix=${CATKIN_DEVEL_PREFIX}
    BUILD_COMMAND cd ../glog_src/ && make -j 8
    INSTALL_COMMAND cd ../glog_src/ && make install -j 8
  )

  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/glog
          DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
          FILES_MATCHING PATTERN "*.h")
  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
          DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          FILES_MATCHING PATTERN "libglog*")
  cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
            LIBRARIES glog)
else()
  message(STATUS "Using system Glog.")
  find_package(Glog REQUIRED)
  cs_install()
  cs_export(INCLUDE_DIRS  ${GLOG_INCLUDE_DIRS}
            LIBRARIES     ${GLOG_LIBRARIES})
endif()
