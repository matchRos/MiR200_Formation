#include <simulation_env/planner.h>
#include <ros/ros.h>


int main(int argc,char **argv)
{
    ros::init(argc,argv,"LissajousPlanner");
    ros::NodeHandle nh("~");
    
    tf::Pose ref;
    if(argc>=3)
    {
        ref=tf::Pose(   tf::createQuaternionFromRPY(0,0,atof(argv[3])),
                        tf::Vector3(atof(argv[1]),atof(argv[2]),0));
    }
    else
    {
        ref=tf::Pose(   tf::createQuaternionFromRPY(0,0,0),
                        tf::Vector3(0,0,0));
    }

    LissajousPlanner lissa(nh);
    lissa.set_start_pose(ref);
    lissa.load();
   
    ros::spin(); 
}