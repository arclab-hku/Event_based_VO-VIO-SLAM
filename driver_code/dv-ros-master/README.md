# DV ROS

ROS drivers and sample nodes for iniVation cameras and DV software infrastructure.

## Installation on Ubuntu OS

The code depends on DV software libraries, these libraries need to be installed for the ROS nodes to compile.
Enable the appropriate iniVation PPA depending on your Ubuntu distribution:

* For Ubuntu 18.04:
```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo add-apt-repository ppa:inivation-ppa/inivation-bionic
sudo apt update
sudo apt install dv-processing dv-runtime-dev gcc-10 g++-10
```

* For Ubuntu 20.04:
```
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt update
sudo apt install dv-processing dv-runtime-dev gcc-10 g++-10
```

Some extra ROS dependencies might also be needed:
```
# Example for ROS Noetic on Ubuntu 20.04
sudo apt install python3-catkin python3-catkin-tools ros-noetic-catkin ros-noetic-camera-info-manager
```

The project is build using catkin tools, run the following commands from your catkin workspace:
```
# Run in your catkin workspace root directory
cd src
git clone https://gitlab.com/inivation/dv/dv-ros.git
cd ..
catkin build --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10
```

## Verifying the build

After the build, source your environment to load the information about the new packages, connect your camera
to the computer and validate the build by running visualization sample:
```
source devel/setup.bash
roslaunch dv_ros_visualization event_visualization.launch
```

You should see a preview of events coming from the iniVation camera connected to your computer.

## Compatibility

The message types for events are designed to be compatible with the types available in
[rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros) repository, so the communication is possible with all nodes
developed using those event message types (Event and EventArray). The `capture_node` is designed to be a more general
node replacing the individual nodes for each type of iniVation camera (`davis_ros_driver`, `dvs_ros_driver`, and `dvxplorer_ros_driver`).

## Repository structure

The repository contains multiple projects:
* dv_ros_msgs - Basic data types for the cameras
* dv_ros_messaging - C++ headers required to use dv-processing in ROS nodes
* dv_ros_capture - Camera driver node (supports live camera data streaming and aedat4 file playback)
* dv_ros_accumulation - Event stream to frame accumulation
* dv_ros_aedat4 - Convert aedat4 files to rosbags
* dv_ros_runtime_modules - DV runtime modules for integration with ROS
* dv_ros_visualization - Simple visualization of events
* dv_ros_tracker - Lucas-Kanade feature trackers for event and image streams
