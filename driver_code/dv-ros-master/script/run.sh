# 开启roscore
gnome-terminal --tab -e 'bash -c "roscore;exec bash"'
# sleep 3s

# 可视化
gnome-terminal --window -e 'bash -c "roslaunch dv_ros_visualization event_visualization.launch;exec bash"'