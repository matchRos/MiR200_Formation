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

        ///link Controller to it's masters odom input topic
        ///'topic_name'Name of the topic the Controller reads its oddom from                       
        void link_master_odom(std::string topic_name); 

        void scope();        

    private:
        /// Callback for master odometry message. Is executed everytima a Odometry input is incoming. Writes data to input current_pose
        void master_odom_callback(nav_msgs::Odometry msg);
        ///This implements a least sqaures determination of control vector [v,omega] [control.v control.omega] 
        ///from the given cartesian velocity state d/dt[x,y,phi] (cart_vel)
        void optimal_control();
       
       ros::Subscriber master_odom;
};