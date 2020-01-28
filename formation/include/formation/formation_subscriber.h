#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <formation/formation.h>

class FormationSubscriber{
    public:
        FormationSubscriber(ros::NodeHandle &nh,Formation* formation,std::vector<std::string> topics);
        void callback_odometry(const nav_msgs::OdometryConstPtr& msg,int number);
        void callback_laserscanner(const sensor_msgs::LaserScanConstPtr &msg, int number);
        
    private:
        Formation* formation_;
        tf::TransformListener listener_;
        std::list<ros::Subscriber> odom_subscribers_;
        std::list<ros::Subscriber> laser_subscribers_;
        
};

