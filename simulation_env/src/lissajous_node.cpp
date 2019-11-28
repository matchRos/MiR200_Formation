#include <simulation_env/planner.h>
#include <ros/ros.h>


int main(int argc,char **argv)
{
    ros::init(argc,argv,"LissajousPlanner");
    ros::NodeHandle nh;
    
    tf::Pose ref;
    ref.setOrigin(tf::Vector3(0,0,0));
    
    tf::Quaternion quat;
    quat.setRPY(0,0,0);
    ref.setRotation(quat);

    LissajousPlanner lissa(nh);
    lissa.set_parameter(atof(argv[1]));
    lissa.set_start_pose(ref);
    lissa.start();
    ros::spin(); 
}