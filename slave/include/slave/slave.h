#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>



#include <math.h>
#include <stdio.h>


     
class Slave{
    public:
        Slave(ros::NodeHandle &nh);    
        struct cart_state{
            double x;
            double y;
            double z;
        };
        struct state{
            double v;
            double omega;
        };
        ///setting the name of the slave and its node
        void set_name(std::string);                          
        ///setting the name of corresponding master        
        void set_master_name(std::string);                 
        
        ///setting the orientation of the slave in global sense
        ///x,y,z position in cartesians space
        void set_orientation(double x,double y,double z);    
        ///setting a state state in global/cartesian sense
        ///x,y,z position in cartesians space
        ///ori: reference to cart_state that should be changed
        void set_cart_state(double x,double y,double z,Slave::cart_state &ori);  
        
        ///setting the position of the slave in global sense
        void set_position(double x,double y,double z);                                   
        void set_position(double x,double y,double z,Slave::cart_state &pos); 
        //setting the reference position of the slave to the master 
        void set_reference(double x,double y,double z);  

        ///setting the cartesian linear velocitities
        void set_translation(double x,double y,double z);          
        void set_translation(double x,double y,double z,Slave::cart_state &trans);

        ///setting the rotational velocities
        void set_rotation(double x,double y,double z);      
        void set_rotation(double x,double y,double z,Slave::cart_state &rot);



        //Methods for linking the slave with important topics as input output an odom

        ///link slave to it's input topic
        ///'topic_name': Name of tjhe topic the slave gets input from
        void link_input(std::string topic_name);   
        ///link slave to it's input topic
        ///'topic_name'Name of the topic the slave writes its output to                       
        void link_output(std::string topic_name); 
        ///link slave to it's transformation
        ///'topic_name1'Name of the reference frame    
        ///'topic_name2'Name of the target frame                
        void link_transform(std::string transform_name1,std::string transform_name2); 
        
        
        


        
        void set_frequenzy(double frequenzy);
        
        void run();

        virtual void input_callback(geometry_msgs::Twist msg);          //Callback routine for incomung data: Writes data to input state
        void odom_callback(nav_msgs::Odometry msg);

        
    protected:
        ros::NodeHandle nh;                                     //Node Handle
        ros::Publisher output;                                  //publisher object for output topic
        ros::Subscriber input;                                  //Subscirber object for input topic
        
        tf::TransformListener tf;                               //Listener object for the slave base transformation
        std::string reference;                                  //refernce frame from wich the transform is listened
        std::string target;                                     //Frame to wich the transform is listened

        std::string name;                                       //Name of the node and slave
        std::string master_name;                                //Name of the corresponding master

        cart_state cart_pos;                                   //cartesian position stat
        cart_state cart_ori;                                   //cartesian orientation state
        cart_state ref_pos;

        cart_state cart_vel_in;                                 //Target velocity state from master
        cart_state cart_rot_in;                                 //Target rotational state from master
        

        cart_state cart_vel;                                    //cartesian velocity state
        cart_state cart_rot;                                    //cartesian rotation state

        state control;                                          //local state of slave (control state)

       

        double frequenzy;                                       //Control frequenzy
        double step;


        ///This implements a least sqaures determination of control vector [v,omega] [control.v control.omega] 
        ///from the given cartesian velocity state d/dt[x,y,phi] (cart_vel)
        void optimal_control();

        void forward_propagation();
        void global_from_local();   
};