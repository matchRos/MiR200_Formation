number=5

terminator -e "roslaunch multi_robot_launcher simulation.launch"&
sleep 10
terminator -e "roslaunch multi_robot_controller master.launch robot_name:=master robot_x:=-1.0 robot_y:=0.0 :=0.0"&

for k in `seq 1 $number`
do
    sleep 2
    name="robot"$k
    terminator -e "roslaunch multi_robot_controller slave.launch robot_name:=$name  robot_x:=$k robot_y:=0.0 robot_yaw:=0.0"&
done


