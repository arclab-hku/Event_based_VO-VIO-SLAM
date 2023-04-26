cmake_minimum_required(VERSION 3.14)
project(dv_ros_tracker VERSION 1.2.0)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             std_msgs
             sensor_msgs
             geometry_msgs
             message_generation
             dv_ros_msgs
             dv_ros_messaging
             dv_ros_capture)

find_package(dv-processing 1.4.0 REQUIRED)

add_message_files(FILES Depth.msg TimedKeypoint.msg TimedKeypointArray.msg)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  dv_ros_msgs
  dv_ros_messaging
  dv_ros_capture)

include_directories(include ${catkin_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_BINARY_DIR}/devel/include)

set(SOURCE src/tracker_node.cpp src/main.cpp)
add_executable(tracker_node ${SOURCE})

add_dependencies(tracker_node ${catkin_EXPORTED_TARGETS}
                 ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(tracker_node ${catkin_LIBRARIES} dv::processing)
