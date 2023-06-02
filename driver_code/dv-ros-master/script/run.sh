# 开启roscore
gnome-terminal --tab -e 'bash -c "roscore;exec bash"'
sleep 1s

# # # davis346
gnome-terminal --tab -e 'bash -c "roslaunch dv_ros_visualization left_davis346.launch;exec bash"'
# sleep 10s
gnome-terminal --tab -e 'bash -c "roslaunch dv_ros_visualization right_davis346.launch;exec bash"'

# # # dvxplorer
gnome-terminal --tab -e 'bash -c "roslaunch dv_ros_visualization left_dvxplorer.launch;exec bash"'
# # # sleep 10s
gnome-terminal --tab -e 'bash -c "roslaunch dv_ros_visualization right_dvxplorer.launch;exec bash"'


# 可视化
gnome-terminal --tab -e 'bash -c "roslaunch dv_ros_visualization rqt_visualization.launch;exec bash"'

# 录制osbag
# roslaunch dv_ros_visualization event_record.launch 
# 同步机载电脑
# sudo ptpd -g -i eno1 -C


# gnome-terminal --tab -e 'bash -c "roslaunch dv_ros_visualization event_visualization.launch;exec bash"'
# gnome-terminal --window -e 'bash -c "rosbag play -r 1.0 --pause ~/road_data_bag_event_last.bag;exec bash"'
# gnome-terminal --window -e 'bash -c "rostopic echo /DAVIS346_left/image/header;exec bash"'
# gnome-terminal --window -e 'bash -c "rostopic echo /DAVIS346_right/image/header;exec bash"'

#bag record
sleep 6s
# gnome-terminal --window -e 'bash -c "roslaunch dv_ros_visualization event_record.launch ;exec bash"'