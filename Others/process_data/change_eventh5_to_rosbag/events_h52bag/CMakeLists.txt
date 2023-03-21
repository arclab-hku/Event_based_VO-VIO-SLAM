set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_minimum_required(VERSION 2.8.3)

project(events_h52bag)
find_package(HDF5 COMPONENTS CXX REQUIRED)
find_package(rosbag REQUIRED std_msgs REQUIRED)

# modify this path to your ROS installation for dvs_mgs
set(ROS_INCLUDE_DIR /home/cpy/catkin_ws_dvs/devel/include)

include_directories(
        ${rosbag_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${HDF5_INCLUDE_DIR}
        ${ROS_INCLUDE_DIR}
)

add_executable(events_h52bag src/main.cpp)
target_link_libraries(events_h52bag
    ${HDF5_CXX_LIBRARIES}
    ${rosbag_LIBRARIES}
    ${std_msgs_LIBRARIES}
)

