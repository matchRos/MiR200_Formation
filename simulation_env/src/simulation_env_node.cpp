#include <simulation_env/planner.h>
#include <ros/ros.h>


int main(int argc,char **argv)
{
    ros::init(argc,argv,"Simulation");
    ros::NodeHandle nh;
    CirclePlanner circle(nh);
    circle.set_parameter(1.5,0.0,1.5,0.1);
    circle.start();   
    ros::spin(); 
}