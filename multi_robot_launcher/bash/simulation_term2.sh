function make_video()
{
	avconv -framerate 30 -i $1/camera$3/default_camera$3_link_camera$3\($3\)-%04d.jpg -c:v libx264 $2/camera$3.mp4
}


experiment_name='Test'
path='/home/ros_match/Documents/Simulations'

sleep 10 && rosrun simulation_env system_handling_node -plan -reference  

mkdir -p  $path/$experiment_name/Videos
cp /home/ros_match/catkin_ws/src/multi_robot_system/multi_robot_launcher/config/formation.yaml $path/$experiment_name
cp /home/ros_match/catkin_ws/src/multi_robot_system/multi_robot_launcher/bag/$experiment_name.bag $path/$experiment_name

make_video /tmp $path/$experiment_name/Videos $VARIABLE 

for VARIABLE in 1 2 3 4 5 6
do
	make_video /tmp $path/$experiment_name/Videos $VARIABLE 
done

killall rosmaster
sleep 10