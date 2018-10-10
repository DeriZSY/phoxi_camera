ip=$1
roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=$ip&
sleep 10s
python /home/bionicdl/catkin_ws/src/phoxi_camera/src/cal_dev/cal_main.py
# kill -15 %1