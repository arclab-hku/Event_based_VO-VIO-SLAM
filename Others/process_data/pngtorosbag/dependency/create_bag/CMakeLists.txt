cmake_minimum_required(VERSION 3.0.2)
project(create_bag)
 
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
 
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# 寻找OpenCV库
find_package( OpenCV  REQUIRED )
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )
 
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rosbag
  roscpp
  rospy
  std_msgs
)
 
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imgtobag
#  CATKIN_DEPENDS cv_bridge rosbag roscpp rospy std_msgs
#  DEPENDS system_lib
)
 
###########
## Build ##
###########
 
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${rosbag_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
 
add_executable(node src/torosbag.cpp)
target_link_libraries(node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${rosbag_LIBRARIES}
  ${OpenCV_LIBS}
)
