# multi_robot_system

## Usage:


## Troubleshooting:

1. if gazebo crashes it may has to be killed by "killall gzserver"


## Important nodes:

| Nodename       			 | Short description            | Documnetation |
| ------------- 			 |:-------------:		| :-------------:		|
| robot_state_publisher     		 | calculates the forward kinematics of a mobile base platform 		| 
| robot_slave     			 | calculaties motion of a slave      		| 
| robot_master 			 	 | manages motion of the master and comunication with the slaves     	   	|
|ekf_navigation_node			 |ekf_localization_node is an implementation of an extended Kalman filter. It uses an omnidirectional motion model to project the state forward in time, and corrects that projected estimate using perceived sensor data.| http://docs.ros.org/melodic/api/robot_localization/html/state_estimation_nodes.html#ekf-localization-node

