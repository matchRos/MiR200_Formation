#include<ros/ros.h>
#include<multi_robot_simulation/planner.h>

enum ControllerTypes{
    lissajous,
    circle,
    spiral
};

int main(int argc,char**argv)
{
    //Choose planner
    tf::Pose reference(tf::createIdentityQuaternion(),tf::Vector3(0,0,0));

    ros::NodeHandle nh_planner("planner");
    Planner* planner;
    int type;    
    nh_planner.getParam("/planner/type",type);
    ROS_INFO("Got parameter /planner/type: %i",type);
    switch(type)
    {
        case lissajous:
        {
            ROS_INFO("Starting LissajousPlanner!"); 
            planner=new LissajousPlanner(nh_planner);
            break;
        }
        case circle:
        {
             ROS_INFO("Starting Circle Planner!");  
            planner=new CirclePlanner(nh_planner);
            break;
        }
        case spiral:
        {
            ROS_INFO("Starting Spiral Planner");
            planner=new Spiralplanner(nh_planner);
        }
              

    }
   
    planner->setStartPose(reference);
    planner->load();
    
    ros::Duration(2).sleep();
    bool pause;
    nh_planner.getParam("planner/pause",pause);
    ROS_INFO("Got Parameter /planner/pause: %d",pause);
    if(!pause)
    {
        planner->start();
    }    
    ros::spin();
    delete planner;
}