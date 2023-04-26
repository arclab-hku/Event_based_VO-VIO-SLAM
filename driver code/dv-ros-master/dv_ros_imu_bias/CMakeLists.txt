cmake_minimum_required(VERSION 3.14)
project(dv_ros_imu_bias VERSION 1.2.0)

find_package(dv-processing 1.4.0 REQUIRED)
find_package(
  catkin REQUIRED COMPONENTS roscpp std_msgs dv_ros_msgs dv_ros_messaging
                             dynamic_reconfigure dv_ros_capture)

generate_dynamic_reconfigure_options(config/biasesParams.cfg)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS roscpp std_msgs dv_ros_msgs dv_ros_messaging
               dv_ros_capture)

set(SOURCE src/main.cpp)
add_executable(bias_estimation_node ${SOURCE})

add_dependencies(bias_estimation_node ${catkin_EXPORTED_TARGETS}
                 ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(bias_estimation_node ${catkin_LIBRARIES} dv::processing)
