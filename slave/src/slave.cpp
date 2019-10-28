#include <slave/slave.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <stdio.h>


Slave::Slave(ros::NodeHandle &nh):nh(nh)
{

    set_name("my_slave");
    set_master_name("my_master");
    set_orientation(0.0,0.0,0.0);
    set_position(0.0,0.0,0.0);
    set_translation(0.0,0.0,0.0);
    set_rotation(0.0,0.0,0.0);

    this->output=this->nh.advertise<geometry_msgs::Twist>("out",10);
    this->input=this->nh.subscribe("in",10,&Slave::input_callback,this);

    //Parameters that have to be implented more dynamic later
    this->frequenzy=10;
    this->step=1/frequenzy;


    
} 


void Slave::set_name(std::string name)
{
    this->name=name;
    this->nh.resolveName(this->name);
}
    
void Slave::set_master_name(std::string name)
{
    this->master_name=name;
}

void Slave::set_orientation(double x,double y,double z)
{
    this->cart_ori.x=x;
    this->cart_ori.y=y;
    this->cart_ori.z=z;
}

void Slave::set_position(double x,double y,double z)
{
    this->cart_pos.x=x;
    this->cart_pos.y=y;
    this->cart_pos.z=z;
}

void Slave::set_translation(double x,double y,double z)
{
     this->cart_vel.x=x;
     this->cart_vel.y=y;
     this->cart_vel.z=z;
}

void Slave::set_rotation(double x,double y,double z)
{
    this->cart_rot.x=x;
    this->cart_rot.y=y;
    this->cart_rot.z=z;
}

void Slave::link_input(std::string topic_name)
{
    this->input.shutdown();
    this->input=this->nh.subscribe(topic_name,10,&Slave::input_callback,this);
}

void Slave::link_output(std::string topic_name)
{
    this->output.shutdown();
    this->output=this->nh.advertise<geometry_msgs::Twist>(topic_name,10);
}

void Slave::input_callback(geometry_msgs::Twist msg)
{
    ROS_INFO("IN");
    this->set_translation(msg.linear.x,msg.linear.y,msg.linear.z);
    this->set_rotation(msg.angular.x,msg.angular.y,msg.angular.z);

    this->optimal_control();
    this->global_from_local();
    this->forward_propagation();
}

void Slave::optimal_control()
{
     this->control.v=cos(this->cart_ori.z)*this->cart_vel.x +sin(this->cart_ori.z);
     this->control.omega=(this->cart_rot.z);
     this->global_from_local();
}

void Slave::global_from_local()
{
     this->cart_vel.x=cos(this->cart_ori.z)*this->control.v;
     this->cart_vel.y=sin(this->cart_ori.z)*this->control.v;
     this->cart_rot.z=this->control.omega;
}

void Slave::forward_propagation()
{
    this->cart_pos.x=this->cart_vel.x*this->step;
    this->cart_pos.y=this->cart_vel.y*this->step;
    this->cart_ori.z=this->cart_rot.z*this->step;
}

void Slave::run()
{
    ros::Rate rate(this->frequenzy);
    ROS_INFO("START");
    while(ros::ok())
    {
        //publish
        geometry_msgs::Twist msg;
        msg.linear.x=this->cart_vel.x;
        msg.linear.y=this->cart_vel.y;
        msg.linear.z=this->cart_vel.z;
        msg.angular.x=this->cart_rot.x;
        msg.angular.y=this->cart_rot.y;
        msg.angular.z=this->cart_rot.z;
        this->output.publish(msg);
        ros::spinOnce();

        rate.sleep();
    }

}