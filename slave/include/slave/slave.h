#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <vector>


     
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
        ///setting the orientation of the slave in global sense
        ///x,y,z position in cartesians space
        ///ori: reference to cart_state that should be changed
        void set_orientation(double x,double y,double z,Slave::cart_state &ori);  
        
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

        ///link slave to it's input topic
        ///'topic_name': Name of tjhe topic the slave gets input from
        void link_input(std::string topic_name);   
        ///link slave to it's input topic
        ///'topic_name'Name of the topic the slave writes its output to                       
        void link_output(std::string topic_name);    
        ///
        void set_frequenzy(double frequenzy);
        
        void run();

        virtual void input_callback(geometry_msgs::Twist msg);          //Callback routine for incomung data: Writes data to input state
    

        
    protected:
        ros::NodeHandle nh;                                     //Node Handle
        ros::Publisher output;                                  //publisher object for output topic
        ros::Subscriber input;                                  //Subscirber object for input topic

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

        void optimal_control();
        void forward_propagation();
        void global_from_local();   
};