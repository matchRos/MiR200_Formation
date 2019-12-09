#include <simulation_env/planner.h>
#include <ros/ros.h>


int main(int argc,char **argv)
{
    ros::init(argc,argv,"LissajousPlanner");
    ros::NodeHandle nh("~");
    
    tf::Pose ref;
    ref.setOrigin(tf::Vector3(0,0,0));
    
    tf::Quaternion quat;
    quat.setRPY(0,0,0);
    ref.setRotation(quat);

    LissajousPlanner lissa(nh);
    lissa.load();
    lissa.set_start_pose(ref);
    if(argc>1)
    {
        if(strcmp(argv[1],"start"))
        {
            lissa.start();
        }
    }  
   
    ros::spin(); 
}