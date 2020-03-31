#Clear temp images
rm -r /tmp/camera*


#Setting up simulation structure.
roslaunch multi_robot_launcher exe_record.launch filename:=$1          #Without object
roslaunch multi_robot_launcher exe_record.launch filename:=$1          #Without object
# roslaunch multi_robot_launcher exe_record_transport.launch filename:=$1     #With object but planned