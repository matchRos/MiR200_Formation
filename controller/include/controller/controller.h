#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>



#include <math.h>
#include <stdio.h>


     
class Controller{
    public:
     Controller(ros::NodeHandle &nh);    
      
        ///setting the name of the Controller and its node
        void set_name(std::string);  

        void set_frequenzy(double frequenzy);

        void set_reference(double x,double y,double z);                        


        //Methods for linking the Controller with important topics as input output an odom

        ///link Controller to it's input topic
        ///'topic_name': Name of tjhe topic the Controller gets input from
        void link_input_velocity(std::string topic_name);   
        ///link Controller to it's odom input topic
        ///'topic_name'Name of the topic the Controller reads its oddom from                       
        void link_input_odom(std::string topic_name); 
        ///link Controller to it's input topic
        ///'topic_name'Name of the topic the Controller writes its output to                       
        void link_output_velocity(std::string topic_name);        
        
        
        virtual void scope();



        //Callbacks

        void input_velocities_callback(geometry_msgs::Twist msg);          //Callback routine for incomung data: Writes data to input state

        void input_odom_callback(nav_msgs::Odometry msg);

        
    protected:
        ros::NodeHandle nh;                                     //Node Handle

        ros::Publisher output;                                  //publisher object for output topic
        ros::Subscriber input;                                  //Subscirber object for input topic
        ros::Subscriber odom;                                   //Subscriber object for odometry
    
     

        std::string name;                                       //Name of the node and Controller

        nav_msgs::Odometry msg_odom;                            //Odometry message 


        geometry_msgs::Twist msg_velocities_in;                 //Input velocities
        geometry_msgs::Twist msg_velocities_out;                //output velocities
        geometry_msgs::Twist msg_velocities_ideal;              //Ideal velocity state
        
        
        geometry_msgs::Pose reference;                          //Reference position
        geometry_msgs::Pose current_pose;                       //Pose of Controller at the moment

       


        ///This implements a least sqaures determination of control vector [v,omega] [control.v control.omega] 
        ///from the given cartesian velocity state d/dt[x,y,phi] (cart_vel)
        void optimal_control();
};