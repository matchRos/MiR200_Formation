# Formation control with multi_robot_system package
Since there are some large advantages of using multi robot system in case of e.g. object transportation this package implements functionalities for this purpose. Therfore it contains ros nodes, configuration files and launch files for using multiple mobile robots (differential driven) as a formation. The package contains two different control laws, implemented within the multi_robot_controller. For using this the multi_robot_launcher gives some configurations and launch files for simulating formations of robots within the gazebo simualtion environment. Further a multi_robot_simulation package occures which contains helper functions for the initialisation of controllers and for planning some test trajectories. A detailed description and a quick start for the multi_robot_system package can be found in the following.
# How to install
Since this package depends on some submodules (see [submodules](../submodules)) the buidl by source is a little different from other repositories.
1. Clone the repository in the common way (substitute X by the gazebo version you use):
  * `$ git clone -b gazeboX  https://github.com/matchRos/MiR200_Formation.git`
2. This is just used if you want to get the content form the submodules
  2a. Get all submodules: 
    * `git submodule init`
    * `git submodule update`\
  2b. Get just the gazebo_ros_link_attacher:
    * `git submodule init`
    * `git submodule update submodules/gazebo_ros_link_attacher`\
3. Build with your build tools (e.g catkin_make)

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
<<<<<<< HEAD
3. Execute the system handling node `$rosrun multi_robot_simualtion system_handling_node -plan -reference`
=======
3. Execute the system handling node `$rosrun simulation_env system_handling_node -plan -reference`
>>>>>>> 79b34a8474db407a148318c21493512015706137

# How to modifie formation parameters:
1. Goto foramtion.yaml in the multi_robot_launcher package
2. Change the parameter within the file
2a) If you change the number or names of formation memebers make sure that you do the same in the formation.launch file in mutli_robot_launcher package
