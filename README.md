# Definition of launchfiles:
Launchfiles for launching the implemented functionalities are given in /multi_robot_launcher/launch
1. exe_demo.launch:
Executes a demo environment within basic funcitonalities of the robot formation can be shown.
2. exe_navigation.launch:
Executes an environment within the robot formation can be used in combination with the navigation_stack.
3. exe_transport.launch:
Executes an environment within the robot formation is carrying a transport object.
4. formation.launch:
This is one of the most important launchfiles for using mobile robot formation. It launchs the robots for the formation and does the nescessary remaping for member parameters.
5. gazebo. launch:
Just launches the gazeob environment.
6. robot.launch:
This launches the hole robot stuff. Eg. The robot state publisher the localisation node for extended kalman filter and loads parameter for these modules. This file does NOT load the Foramtion controller since this is located within the controller package!
7. simulation.launch:
Prepares simulations from skript files (see bash).
8. spawn_transport_object.launch
A launch files that is used to spawn generic transport object

# How to start a simple formation:
1. Execute exe_demo.launch
2. Execute the system handling node 
`roslaunch multi_robot_launcher exe_demo.launch`
wait til everything is setup
`rosrun simulation_env system_handling_node -plan -reference`
if necessary click on simulation pasued within gazebo.
