#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <controller/controller.h>



#include <math.h>
#include <stdio.h>

#define PARAM_MASTER_ODOM "topic_master_odometry"

     
class Slave:public Controller{
    public:
        Slave(ros::NodeHandle &nh);        
        void scope();  
        ///Links the slave to the master complete odometry descripten for accessing its velocity and pose
        ///param topic_name :name of the topic
        void link_master_odom(std::string topic_name);      

    private:
        ros::Subscriber master_odom;

        ///This implements a least sqaures determination of control vector [v,omega] [control.v control.omega] 
        ///from the given cartesian velocity state d/dt[x,y,phi] (cart_vel)
        void optimal_control();
        void target_state_callback(geometry_msgs::PoseStamped msg) ; 
        void master_odom_callback(nav_msgs::Odometry msg);

        tf::Pose master_pose;
        tf::Vector3 master_lin_vel;
        tf::Vector3 master_ang_vel;

};