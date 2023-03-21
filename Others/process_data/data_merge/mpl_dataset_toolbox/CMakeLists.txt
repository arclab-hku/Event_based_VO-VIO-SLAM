cmake_minimum_required(VERSION 3.0.2)
project(mpl_dataset_toolbox)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  # prophesee_event_msgs
  pcl_conversions
  pcl_ros
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

add_message_files(
  FILES
  Event.msg
  EventArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  include
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_executable(bag2rawfile src/bag2rawfile.cpp)
add_executable(bag_merger src/bag_merger.cpp)
add_executable(bag_splitter src/bag_splitter.cpp)
add_executable(data_validator src/data_validator.cpp)
add_executable(event_visualizer src/event_visualizer.cpp)
add_executable(timeline_reconstructor src/timeline_reconstructor.cpp)

target_link_libraries(bag2rawfile ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(bag_merger ${catkin_LIBRARIES})
target_link_libraries(bag_splitter ${catkin_LIBRARIES})
target_link_libraries(data_validator ${catkin_LIBRARIES})
target_link_libraries(event_visualizer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(timeline_reconstructor ${catkin_LIBRARIES})
