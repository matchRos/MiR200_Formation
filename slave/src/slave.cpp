#include <slave/slave.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


Slave::Slave(ros::NodeHandle &nh):nh(nh)
{
    this->output=this->nh.advertise<geometry_msgs::Twist>(this->name+"_out",10);
    this->input=this->nh.subscribe(this->name+"_in",10,&Slave::input_callback,this);
} 


void Slave::set_name(std::string name)
{
    this->name=name;
    this->nh.resolveName(this->name);
}
    
void Slave::set_orientation(double x,double y,double z)
{
    this->ori_slave.x=x;
    this->ori_slave.y=y;
    this->ori_slave.z=z;
}


void Slave::set_position(double x,double y,double z)
{
    this->pos_slave.x=x;
    this->pos_slave.y=y;
    this->pos_slave.z=z;
}
void Slave::set_master_name(std::string name)
{
    this->master_name=name;
}

void Slave::set_velocity(double x,double y,double z)
{
     this->vel_slave.x=x;
     this->vel_slave.y=y;
     this->vel_slave.z=z;
}

void Slave::run()
{

}