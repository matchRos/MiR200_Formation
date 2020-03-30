killall gzserver
killall gzclient
killall roscore

roslaunch multi_robot_launcher gazebo.launch paused:=true &
sleep 10
roslaunch multi_robot_launcher exe_demo.launch &
sleep 10
rosrun simulation_env system_handling_node -plan -reference &

killall roscore