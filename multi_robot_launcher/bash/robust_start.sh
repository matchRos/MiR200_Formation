killall gzserver
killall gzclient
killall rosmaster

roslaunch multi_robot_launcher gazebo.launch paused:=true &
sleep 10
roslaunch multi_robot_launcher exe_demo.launch 
killall rosmaster