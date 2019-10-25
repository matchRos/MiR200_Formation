#include <ros/ros.h>
#include <tf/transform_listener.h>



int main(int argc, char **argv)
{
    ros::init(argc,argv,"planner");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    ros::Rate rate(1);
    while(nh.ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("/robot_master/base_link","map", ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        rate.sleep();
    }


}