cmake_minimum_required(VERSION 3.14)
project(dv_ros_capture VERSION 1.2.0)

find_package(dv-processing 1.5.0 REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             std_msgs
             sensor_msgs
             geometry_msgs
             message_generation
             image_transport
             visualization_msgs
             dv_ros_msgs
             dv_ros_messaging
             geometry_msgs
             dynamic_reconfigure
             tf2_ros
             tf2_msgs
             message_generation)

add_message_files(FILES ImuInfo.msg CameraDiscovery.msg)

generate_dynamic_reconfigure_options(config/DVXplorer.cfg config/DAVIS.cfg
                                     config/Playback.cfg)

include_directories(include ${catkin_INCLUDE_DIRS})

add_service_files(FILES SetImuInfo.srv SetImuBiases.srv SynchronizeCamera.srv)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  roscpp
  std_msgs
  sensor_msgs
  dv_ros_msgs
  dv_ros_messaging
  geometry_msgs
  tf2_ros)

set(SOURCE src/parameters_loader.cpp src/capture_node.cpp src/reader.cpp
           src/main.cpp)

add_executable(capture_node ${SOURCE})

add_dependencies(capture_node ${catkin_EXPORTED_TARGETS}
                 ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(capture_node ${catkin_LIBRARIES} dv::processing)

catkin_install_python(PROGRAMS scripts/set_calibration DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})
