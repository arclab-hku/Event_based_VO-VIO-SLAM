# DV ROS Messaging

This project contains header files that serve as compatibility layer to use dv-processing library in ROS nodes. The
dv-processing library was designed as a modern C++20 library and heavily relies on C++20 features. Unfortunately, ROS
message code is not compatible with C++20 due to use of deprecated `std::allocator` features, which are not available in
C++20 anymore. This can be worked around by using different allocator instead of the one provided in `std`.

## Usage

To build a ROS node using dv-processing, add `dv_ros_messaging` and `dv_ros_msgs` as
a dependency in the find catkin call (also add find_package for dv-processing):
```cmake
find_package(catkin REQUIRED COMPONENTS
        ...
        dv_ros_msgs
        dv_ros_messaging
        )
find_package(dv-processing 1.3.0 REQUIRED)
```

Libraries and executables using dv-processing features needs to be linked against `dv::processing`:
```cmake
target_link_libraries(your_target
        ${catkin_LIBRARIES}
        dv::processing
        )
```

When writing a node, make sure to include the `dv_ros_messaging/messaging.hpp` header before any ROS headers
are included. Use the `DV_ROS_MSGS` macro when declaring types to avoid compilation issues with the std allocator.
E.g. if you use sensor_msgs::Image type, just wrap the type declaration with the macro:
```c++
// Variable declaration
DV_ROS_MSGS(sensor_msgs::Image) image;

// Or using an alias for the type
using ImageMessage = DV_ROS_MSGS(sensor_msgs::Image);
ImageMessage image;
```

The macro command will replace the `std::allocator` with `boost::container::allocator` instead, which has compatible
API and will allow the code to compile with C++20 features.

The header will also include modified headers for dynamic reconfiguration compatibility, so this feature can also be
used without any modifications.
