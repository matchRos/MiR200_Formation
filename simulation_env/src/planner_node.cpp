#include<ros/ros.h>
#include<simulation_env/planner.h>

int main(int argc,char**argv)
{
    //Just zero initialisation for pose
    tf::Pose ref;
    ref.setOrigin(tf::Vector3(0,0,0));
    
    tf::Quaternion quat;
    quat.setRPY(0,0,0);
    ref.setRotation(quat);



    //Choose planner from cammand line
    Planner* planner;
    if(!strcmp(argv[1],"-circle"))
    {
        ROS_INFO("Starting Circle Planner!");
        ros::init(argc,argv,"CirclePlanner");
        ros::NodeHandle nh("~");
        planner=new CirclePlanner(nh);
    }
    else if(!strcmp(argv[1],"-lissa"))
    {
        ROS_INFO("Starting LissajousPlanner!");
        ros::init(argc,argv,"LissajousPlanner");
        ros::NodeHandle nh("~");
        planner=new LissajousPlanner(nh);
    }
    planner->load();
    planner->set_start_pose(ref);
    planner->start();
    ros::spin();
    delete planner;
}