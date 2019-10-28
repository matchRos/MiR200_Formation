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
     
        void set_name(std::string);                             //setting the name of the slave and its node
        void set_master_name(std::string);                      //setting the name of corresponding master
        
        void set_orientation(double x,double y,double z);       //setting the orientation of the slave in global sense
        void set_position(double x,double y,double z);          //setting the position of the slave in global sense
        void set_translation(double x,double y,double z);       //setting the translation velocity of the slave in global sense
        void set_rotation(double x,double y,double z);

        void link_input(std::string topic_name);                //Linking input topic to the given topic name
        void link_output(std::string topic_name);               //Linking output topic to the given topic name
        void set_frequenzy(double frequenzy);
        
        void run();

        void input_callback(geometry_msgs::Twist msg);          //Callback routine for incomung data: Writes data to input state
    

        
    private:
        ros::NodeHandle nh;                                     //Node Handle
        ros::Publisher output;                                  //publisher object for output topic
        ros::Subscriber input;                                  //Subscirber object for input topic

        std::string name;                                       //Name of the node and slave
        std::string master_name;                                //Name of the corresponding master

        cart_state cart_pos;                                   //cartesian position stat
        cart_state cart_ori;                                   //cartesian orientation state
        cart_state cart_vel;                                   //cartesian velocity state
        cart_state cart_rot;
        state control;                                      //local state of slave (control state)

        double frequenzy;                                       //Control frequenzy
        double step;

        void optimal_control();
        void forward_propagation();
        void global_from_local();   
};