cmake_minimum_required(VERSION 3.14)
project(dv_ros_messaging VERSION 1.2.0)

find_package(dv-processing 1.4.0 REQUIRED)
find_package(Boost 1.77 REQUIRED COMPONENTS container)

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure
                                        camera_info_manager dv_ros_msgs)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  message_runtime
  roscpp
  std_msgs
  sensor_msgs
  dynamic_reconfigure
  camera_info_manager
  DEPENDS
  Boost)

include_directories(include ${catkin_INCLUDE_DIRS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_messaging tests/messaging_tests.cpp)
  target_link_libraries(test_messaging ${catkin_LIBRARIES} dv::processing
                        Boost::container)
endif()
