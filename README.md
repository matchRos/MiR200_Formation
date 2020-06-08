# Formation control with multi_robot_system package
Since there are some large advantages of using multi robot system in case of e.g. object transportation this package implements functionalities for this purpose. Therfore it contains ros nodes, configuration files and launch files for using multiple mobile robots (differential driven) as a formation. The package contains two different control laws, implemented within the multi_robot_controller. For using this the multi_robot_launcher gives some configurations and launch files for simulating formations of robots within the gazebo simualtion environment. Further a multi_robot_simulation package occures which contains helper functions for the initialisation of controllers and planning som test trajectories. A detailed desscription and a quick start for the multi_robot_system package follows.
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
1. Execute exe_demo.launch `$roslaunch multi_robot_launcher exe_demo.launch`
2. Wait until everything is setup
3. Execute the system handling node `$rosrun simulation_env system_handling_node -plan -reference`

# How to modifie formation parameters:
1. Goto foramtion.yaml in the multi_robot_launcher package
2. Change the parameter within the file
2a) If you change the number or names of formation memebers make sure that you do the same in the formation.launch file in mutli_robot_launcher package
