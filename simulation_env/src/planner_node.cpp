#include<ros/ros.h>
#include<simulation_env/planner.h>

int main(int argc,char**argv)
{
    tf::Pose ref;
    ref.setOrigin(tf::Vector3(0,0,0));
    
    tf::Quaternion quat;
    quat.setRPY(0,0,0);
    ref.setRotation(quat);


    Planner* planner;
    if(strcmp(argv[1],"-circle"))
    {
        ros::init(argc,argv,"CirclePlanner");
        ros::NodeHandle nh;
        planner=new CirclePlanner(nh);
    }
    else if(strcmp(argv[1],"-lissa"))
    {
        ros::init(argc,argv,"LissajousPlanner");
        ros::NodeHandle nh;
        planner=new LissajousPlanner(nh);
    }

    planner->set_start_pose(ref);
    ros::spin();
}