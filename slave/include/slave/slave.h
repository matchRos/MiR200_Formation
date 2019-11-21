#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <controller/controller.h>



#include <math.h>
#include <stdio.h>

class Slave:public Controller{
    public:
        Slave(ros::NodeHandle &nh);        
        void scope();  

    private:
        ros::Subscriber master_odom;

        ///This implements a least sqaures determination of control vector [v,omega] [control.v control.omega] 
        ///from the given cartesian velocity state d/dt[x,y,phi] (cart_vel)
        void optimal_control();
        void target_state_callback(geometry_msgs::PoseStamped msg) ; 
        tf::Pose master_pose;
 

};