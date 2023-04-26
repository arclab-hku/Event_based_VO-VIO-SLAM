cmake_minimum_required(VERSION 3.14)
project(dv_ros_accumulation VERSION 1.2.0)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             std_msgs
             sensor_msgs
             image_transport
             visualization_msgs
             dv_ros_msgs
             dv_ros_messaging
             dynamic_reconfigure)

generate_dynamic_reconfigure_options(cfg/Accumulator.cfg cfg/EdgeMap.cfg)

find_package(dv-processing 1.4.0 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  roscpp
  std_msgs
  sensor_msgs
  dv_ros_msgs
  dv_ros_messaging)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(accumulator src/accumulator.cpp)

add_dependencies(accumulator ${catkin_EXPORTED_TARGETS}
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(accumulator ${catkin_LIBRARIES} dv::processing)

add_executable(edge_map src/edge_map.cpp)

add_dependencies(edge_map ${catkin_EXPORTED_TARGETS}
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(edge_map ${catkin_LIBRARIES} dv::processing)
