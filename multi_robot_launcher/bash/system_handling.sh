#This skript sets up the necessary folder structure for the data acquise and runs the systsem handling node that initialises the simulation.
#Skript one has to be executed beforhand since it launchs the simualtion infrastructure.
#Yaml file and rosbags are copied into the proper folder and the movie is generated from the recoreded images. Afterwards the rosmaster is killed and therefore
#the simulation stops.
set -e
function make_video()
{
	# avconv -framerate 30 -i $1/camera$3/default_camera$3_link_camera$3\($3\)-%04d.jpg -c:v libx264 $2/camera$3.mp4
	# ffmpeg -r 30 -pattern_type glob -i $1/camera$3/*.jpg -c:v libx264 test3.mp4
	ffmpeg -r 30 -pattern_type glob -i $1/'camera'$3'/*.jpg' -c:v libx264 $2/camera$3.mp4

}

killall gzserver || true
killall gzclient || true
experiment_name=$1
data_source='/tmp'
config_source='/home/rosmatch/catkin_ws/src/multi_robot_system/multi_robot_launcher/config'
path_target='/media/rosmatch/3E0CFF1E0CFECFBF'

# # rm $data_source/*
sleep 3
roslaunch multi_robot_launcher gazebo.launch world_name:=/home/rosmatch/catkin_ws/src/multi_robot_system/multi_robot_launcher/worlds/empty_world_camera.world &
sleep 10
roslaunch multi_robot_launcher exe_record.launch filename:=$experiment_name &

#Check if gazebo has died
sleep 30
if ! pgrep -x "gzserver" > /dev/null
then
	echo "Problem with Gazebo!"
    exit 1
fi


#Execute the System handling node
rosrun simulation_env system_handling_node -plan -reference -start -camera		#Without object
# rosrun simulation_env system_handling_node -plan -reference -start -camera -link	#With Object but planned
rosnode kill -a
sleep 3
killall rosmaster

#Create directories to store the data in
mkdir -p  $path_target/$experiment_name/Videos

#Copy the foamtion parameter file
cp $config_source/formation.yaml $path_target/$experiment_name

# Copy the corresponding rosbags
for f in $data_source/$experiment_name*.bag; do 
	cp -v -- "$f" "$path_target/$experiment_name" 
done

#Create the videos from the corresponding images
for VARIABLE in 1 2 3 4 5 6; do
	make_video $data_source $path_target/$experiment_name/Videos $VARIABLE 
done



killall rosmaster
sleep 10
$SHELL