cmake_minimum_required(VERSION 3.14)
project(dv_ros_visualization VERSION 1.2.0)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             std_msgs
             sensor_msgs
             message_generation
             image_transport
             visualization_msgs
             dv_ros_msgs
             dv_ros_messaging)

find_package(dv-processing 1.4.0 REQUIRED)

catkin_package(CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs
               dv_ros_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(visualization_node src/visualization_node.cpp)

add_dependencies(visualization_node ${catkin_EXPORTED_TARGETS}
                 ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(visualization_node ${catkin_LIBRARIES} dv::processing)
