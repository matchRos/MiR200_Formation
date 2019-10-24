#include <slave/slave.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void Slave::Slave(ros::NodeHandle &nh):nh(nh)
{
    self->output=self->nh.advertise<geometry_msgs::Twist>(self->name+"_out",10);
    self->input=self->nh.subscribe(self->name+"_in",10,self->input_callback);
} 



void Slave::set_name(std::string name)
{
    this->name=name;
    this->nh.resolveName(this->name);
}
    
void Slave::set_orientation(double x,double y,double z)
{
    this->ori_slave.x=x;
    this->ori_slsave.y=y;
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
    this->maste_name=name;
}

void Slave::set_velocity(double x,double y,double z)
{
     this->vel_slave.x=x;
     this->vel_slave.y=y;
    t his->vel_slave.z=z;
}

void Slave::run()
{

}