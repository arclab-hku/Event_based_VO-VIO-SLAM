# 开启roscore
gnome-terminal --tab -e 'bash -c "roscore;exec bash"'
sleep 1s

# 可视化
gnome-terminal --window -e 'bash -c "roslaunch dv_ros_visualization left_event_visualization.launch;exec bash"'
sleep 10s
gnome-terminal --window -e 'bash -c "roslaunch dv_ros_visualization right_event_visualization.launch;exec bash"'