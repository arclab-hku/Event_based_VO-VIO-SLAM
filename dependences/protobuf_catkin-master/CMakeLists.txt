cmake_minimum_required(VERSION 2.8.3)
project(protobuf_catkin)

find_package(catkin_simple REQUIRED)
find_package(PythonInterp REQUIRED)
catkin_simple()

include(ExternalProject)

set(PROTOBUF_MINIMUM_VERSION 2.6.1)

# If set to OFF, will compile the given version of Protobuf.
# If set to ON, will use the system version of Protobuf.
# If set to GRPC, will use the version of protobuf that ships with catkin_grpc.
# If set to AUTO, will try to use the system version if there is one, and build otherwise.
set(USE_SYSTEM_PROTOBUF "AUTO" CACHE INTERNAL "Whether to use the system version of Protobuf.")

# Figure out what to do if it's auto.
# FindProtobuf cmake reference: https://github.com/Kitware/CMake/blob/master/Modules/FindProtobuf.cmake
if(USE_SYSTEM_PROTOBUF STREQUAL "AUTO")
  find_package(Protobuf QUIET)
  if(Protobuf_FOUND)
    if (DEFINED Protobuf_VERSION)
      if (${Protobuf_VERSION} VERSION_GREATER ${PROTOBUF_MINIMUM_VERSION})
        message(STATUS "Found a suitable version of system protobuf: ${Protobuf_VERSION}")
        set(USE_SYSTEM_PROTOBUF "ON")
      endif()
    endif()
    if(USE_SYSTEM_PROTOBUF STREQUAL "AUTO")
      message(STATUS "Could not find a recent enough version of system protobuf.")
      set(USE_SYSTEM_PROTOBUF "OFF")
    endif()
  else()
    message(STATUS "Could not find system protobuf.")
    set(USE_SYSTEM_PROTOBUF "OFF")
  endif()
endif()

# Ok now do the appropriate thing per option.
# We want to export the preferred protoc location as an extra variable.
if(USE_SYSTEM_PROTOBUF STREQUAL "ON")
  message(STATUS "Using system protobuf.")

  find_package(Protobuf REQUIRED)
  set(PREFERRED_PROTOC_EXECUTABLE ${Protobuf_PROTOC_EXECUTABLE})

  cs_install()
  cs_export(INCLUDE_DIRS ${Protobuf_INCLUDE_DIRS}
            LIBRARIES ${Protobuf_LIBRARIES}
            CFG_EXTRAS protobuf-generate-cpp.cmake)

elseif(USE_SYSTEM_PROTOBUF STREQUAL "GRPC")
  message(STATUS "Using protobuf from grpc_catkin.")

  # This is actually catkin_grpc: https://github.com/CogRob/catkin_grpc
  find_package(grpc REQUIRED)

  # TODO(helenol): this is kinda hacky, but catkin_grpc doesn't seem to export this.
  set(PREFERRED_PROTOC_EXECUTABLE "/opt/ros/$ENV{ROS_DISTRO}/lib/grpc/protobuf/protoc")

  cs_install()
  cs_export(INCLUDE_DIRS ${GRPC_INCLUDE_DIR}
            LIBRARIES ${ALL_PROTOBUF_LIBS}
            CFG_EXTRAS protobuf-generate-cpp.cmake)

elseif(USE_SYSTEM_PROTOBUF STREQUAL "OFF")
  file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

  ExternalProject_Add(protobuf_src
    URL https://github.com/google/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.gz
    UPDATE_COMMAND ""
    PATCH_COMMAND patch -p0 < ${CMAKE_SOURCE_DIR}/unused_parameter_warnings.patch &&
                  patch -p0 < ${CMAKE_SOURCE_DIR}/unused_parameter_warnings_wire_format.patch
    CONFIGURE_COMMAND cd ../protobuf_src && ./autogen.sh && ./configure --with-pic --prefix=${CATKIN_DEVEL_PREFIX}
    BUILD_COMMAND cd ../protobuf_src && make -j8 && cd python && ${PYTHON_EXECUTABLE} setup.py build --build-purelib build
    INSTALL_COMMAND cd ../protobuf_src && make install -j8 &&
                    cd python && ${PYTHON_EXECUTABLE} setup.py install --root ${CATKIN_DEVEL_PREFIX} --install-lib ${CATKIN_GLOBAL_PYTHON_DESTINATION} &&
                    cp build/google/__init__.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/google
  )

  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/google/protobuf
          DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/google
          FILES_MATCHING PATTERN "*.h")
  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
          DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          FILES_MATCHING PATTERN "libproto*")
  install(FILES ${CATKIN_DEVEL_PREFIX}/bin/protoc
          PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
          DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
  set(PREFERRED_PROTOC_EXECUTABLE ${CATKIN_DEVEL_PREFIX}/bin/protoc)
  cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
            LIBRARIES protobuf protobuf-lite protoc
            CFG_EXTRAS protobuf-generate-cpp.cmake)
endif()
