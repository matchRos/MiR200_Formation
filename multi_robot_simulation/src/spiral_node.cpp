#include <multi_robot_simulation/planner.h>


int main (int argc,char**argv)
{
    ros::init(argc,argv,"SpiralPlanner");
    ros::NodeHandle nh;    
    Spiralplanner euler(nh);
    euler.set_parameter(3.0,1.0/(2*M_PI),0.1);
    ros::spin(); 
}
