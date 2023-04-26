cmake_minimum_required(VERSION 3.14)
project(dv_ros_msgs VERSION 1.2.0)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs
                                        message_generation)

add_message_files(FILES Trigger.msg Event.msg EventArray.msg)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs)

include_directories(${catkin_INCLUDE_DIRS})
