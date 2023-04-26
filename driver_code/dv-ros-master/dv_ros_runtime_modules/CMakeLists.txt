cmake_minimum_required(VERSION 3.14)
project(dv_ros_runtime_modules VERSION 1.2.0)

find_package(catkin REQUIRED COMPONENTS pluginlib roscpp roslib dv_ros_msgs
                                        dv_ros_messaging)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES
  ${PROJECT_NAME}
  DEPENDS
  roscpp
  roslib
  dv_ros_msgs
  dv_ros_messaging)

include(GNUInstallDirs)

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/dv
    /usr/${CMAKE_INSTALL_DATAROOTDIR}/dv
    ${CMAKE_INSTALL_PREFIX}/share/dv
    /usr/share/dv
    /usr/local/${CMAKE_INSTALL_DATAROOTDIR}/dv
    /usr/local/share/dv)

# Basic setup Looking for dv-runtime version 1.5+
find_package(dv 1.5.2)
if(NOT dv_FOUND)
  # Not found, falling back to legacy search
  find_package(dv-processing 1.4.0 REQUIRED)
  include(dv-modules)
  dv_module_setup()
  list(APPEND DV_LIBRARIES dv::processing)
else()
  set(DV_LIBRARIES dv::sdk)
endif()

include_directories(${catkin_INCLUDE_DIRS} ${DV_INCLUDE_DIRS})
# Set full RPATH, modules are libraries for DV
set(CMAKE_INSTALL_RPATH ${DV_MODULES_DIR})

add_library(ImageBridge SHARED src/RosImageBridge.cpp)
add_dependencies(ImageBridge ${catkin_EXPORTED_TARGETS})

set_target_properties(ImageBridge PROPERTIES PREFIX "ros_")
target_link_libraries(ImageBridge PRIVATE ${DV_LIBRARIES} ${catkin_LIBRARIES})

add_library(ImuBridge SHARED src/RosIMUBridge.cpp)
add_dependencies(ImuBridge ${catkin_EXPORTED_TARGETS})

set_target_properties(ImuBridge PROPERTIES PREFIX "ros_")

target_link_libraries(ImuBridge PRIVATE ${DV_LIBRARIES} ${catkin_LIBRARIES})

add_library(EventBridge SHARED src/RosEventBridge.cpp src/RosEventBridge.h)
add_dependencies(EventBridge ${catkin_EXPORTED_TARGETS})

set_target_properties(EventBridge PROPERTIES PREFIX "ros_")

target_link_libraries(EventBridge PRIVATE ${DV_LIBRARIES} ${catkin_LIBRARIES})

add_library(TriggerBridge SHARED src/RosTriggerBridge.cpp)
add_dependencies(TriggerBridge ${catkin_EXPORTED_TARGETS})

set_target_properties(TriggerBridge PROPERTIES PREFIX "ros_")

target_link_libraries(TriggerBridge PRIVATE ${DV_LIBRARIES} ${catkin_LIBRARIES})
