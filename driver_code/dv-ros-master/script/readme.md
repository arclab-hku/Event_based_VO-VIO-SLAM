# 配置记录

依赖安装连接：(Link)[https://gitlab.com/inivation/dv/dv-ros]

创建一个新的工作空间 catkin_dvs_ws，然后更新.bashrc文件如下：

```
source ~/catkin_dvs_ws/devel/setup.bash

alias dvsbuild='cd ~/catkin_dvs_ws && catkin build dv_ros_accumulation dv_ros_capture dv_ros_imu_bias dv_ros_messaging dv_ros_runtime_modules dv_ros_tracker dv_ros_visualization -DCMAKE_BUILD_TYPE=Release --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10'

alias dvsrun='cd ~/catkin_dvs_ws/src/Event_based_VO-VIO-SLAM/driver_code/dv-ros-master/script && sh run.sh'

```