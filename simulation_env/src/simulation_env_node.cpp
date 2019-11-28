#include <simulation_env/planner.h>
#include <ros/ros.h>


int main(int argc,char **argv)
{
    ros::init(argc,argv,"Simulation");
    ros::NodeHandle nh;
    tf::Pose ref;
    ref.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion quat;
    quat.setRPY(0,0,0);
    ref.setRotation(quat);
    // CirclePlanner circle(nh);
    // circle.set_parameter(1.5,0.0,1.5,0.1);
    // circle.set_start_pose(ref);
    LissajousPlanner lissa(nh);
    lissa.set_parameter(0.2,0.0,2,3.0,3.0);
    lissa.set_start_pose(ref);    
    ros::spin(); 
}