#This skript sets up the necessary folder structure for the data acquise and runs the systsem handling node that initialises the simulation.
#Skript one has to be executed beforhand since it launchs the simualtion infrastructure.
#Yaml file and rosbags are copied into the proper folder and the movie is generated from the recoreded images. Afterwards the rosmaster is killed and therefore
#the simulation stops.

set -e
function make_video()
{
	avconv -framerate 30 -i $1/camera$3/default_camera$3_link_camera$3\($3\)-%04d.jpg -c:v libx264 $2/camera$3.mp4
}


experiment_name=$1
path_target='/home/ros_match/Documents/Simulations'
path_source="/home/ros_match/catkin_ws/src/multi_robot_system/multi_robot_launcher"

#Check if gazebo has died
sleep 10 
if ! pgrep -x "gzserver" > /dev/null
then
	echo "Problem with Gazebo!"
    exit 1
fi


#Execute the System handling node
rosrun simulation_env system_handling_node -plan -reference -start -camera		#Without object
# rosrun simulation_env system_handling_node -plan -reference -start -camera -link	#With Object but planned


#Create directories to store the data in
mkdir -p  $path_target/$experiment_name/Videos

#Copy the foamtion parameter file
cp $path_source/config/formation.yaml $path_target/$experiment_name

#Copy the corresponding rosbags
for f in /tmp/$experiment_name*.bag; do 
	cp -v -- "$f" "$path_target/$experiment_name" 
done

#Create the videos from the corresponding images
for VARIABLE in 1 2 3 4 5 6; do
	make_video /tmp $path_target/$experiment_name/Videos $VARIABLE 
done



killall rosmaster
sleep 10
$SHELL