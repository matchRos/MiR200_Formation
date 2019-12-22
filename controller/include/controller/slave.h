#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <controller/controller.h>



#include <math.h>
#include <stdio.h>

/**
 * @brief Class that implements a slave robot for multi robot formation control
 * 
 */
class Slave:public Controller{
    public:
        /**
         * @brief Construct a new Slave object
         * 
         * @param nh NodeHandle
         */
        Slave(ros::NodeHandle &nh);        
        
    private:
        /**
         * @brief Calculates a Control vector by pseudo inverting the Jacobian of the Robot
         * 
         * @return ControlVector Calculated Control Vector
         */
        ControlVector optimal_control();
        /**
         * @brief Overloads the target odometry callback and expands it with transformation needed for slave robots.
         * 
         * @param msg  Incoming message at the target odometry topic
         */
        void targetOdomCallback(nav_msgs::Odometry msg);
        

};