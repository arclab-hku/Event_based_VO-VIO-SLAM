# 开启roscore
gnome-terminal --tab -e 'bash -c "roscore;exec bash"'
sleep 1s

# davis346
gnome-terminal --window -e 'bash -c "roslaunch dv_ros_visualization left_davis346.launch;exec bash"'
# sleep 10s
# gnome-terminal --window -e 'bash -c "roslaunch dv_ros_visualization right_davis346.launch;exec bash"'

# # # dvxplorer
# gnome-terminal --window -e 'bash -c "roslaunch dv_ros_visualization left_dvxplorer.launch;exec bash"'
# # # sleep 10s
# gnome-terminal --window -e 'bash -c "roslaunch dv_ros_visualization right_dvxplorer.launch;exec bash"'


# 可视化
gnome-terminal --window -e 'bash -c "roslaunch dv_ros_visualization rqt_visualization.launch;exec bash"'
