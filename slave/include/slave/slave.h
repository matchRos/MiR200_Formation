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
         enum controllerType{
            pseudo_inverse=1,
            lypanov=2,
        };
        void set_type(Slave::controllerType);

    private:
        ///This implements a least sqaures determination of control vector [v,omega] [control.v control.omega] 
        ///from the given cartesian velocity state d/dt[x,y,phi] (cart_vel)
        void optimal_control();
        controllerType type;
};