#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <vector>

     
class Slave{
    public:       
        struct state{
            double x=0.0;
            double y=0.0;
            double z=0.0;
        };
     
        void set_name(std::string);
        void set_orientation(double x,double y,double z);
        void set_position(double x,double y,double z);
        void set_velocity(double x,double y,double z);
        void set_master_name(std::string);
        
        void run();

        void input_callback(msg);
    
    private:
        ros::NodeHandle nh;
        ros::Publisher output;
        ros::Subscriber input;
        std::string name;
        std::string master_name;
        state pos_slave;
        state ori_slave;
        state vel_slave;       
   
};