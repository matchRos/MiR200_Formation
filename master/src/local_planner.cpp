#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <stdio.h>

#include <geometry_msgs/Twist.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"local_planner");
    ros::NodeHandle nh("/robot_master");
    ros::Rate rate(10);
    ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/key_vel",10);
    
    //tf::TransformListener tf(ros::Duration(10));
    //costmap_2d::Costmap2DROS costmap("my_costmap", tf);
    //base_local_planner::TrajectoryPlannerROS tp;
   

    ros::param::set("holonomic_robot",true);

    //tp.initialize("my_trajectory_planner", &tf, &costmap);
    
    while(ros::ok())
    {
        geometry_msgs::Twist cmd_vel;
        //tp.computeVelocityCommands(cmd_vel);
        pub.publish(cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }



}