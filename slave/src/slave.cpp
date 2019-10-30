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
    this->set_orientation(x,y,z,this->cart_ori);
}

void Slave::set_orientation(double x,double y,double z,Slave::cart_state &ori)
{
    ori.x=x;
    ori.y=y;
    ori.z=z;
}

void Slave::set_position(double x,double y,double z)
{
    this->set_position(x,y,z,this->cart_pos);
}

void Slave::set_position(double x,double y,double z,Slave::cart_state &pos)
{
    pos.x=x;
    pos.y=y;
    pos.z=z;
}

void Slave::set_reference(double x,double y,double z)
{
    this->set_position(x,y,z,this->ref_pos);
}



void Slave::set_translation(double x,double y,double z)
{
    this->set_translation(x,y,z,this->cart_vel_in);
}

void Slave::set_translation(double x,double y,double z,Slave::cart_state &trans)
{
     trans.x=x;
     trans.y=y;
     trans.z=z;
}



void Slave::set_rotation(double x,double y,double z)
{
    this->set_rotation(x,y,z,this->cart_rot_in);
}

void Slave::set_rotation(double x,double y,double z,Slave::cart_state &rot)
{
    rot.x=x;
    rot.y=y;
    rot.z=z;
}



void Slave::link_input(std::string topic_name)
{
    this->input.shutdown();
    ROS_INFO("Linking input %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->input=this->nh.subscribe(topic_name,10,&Slave::input_callback,this);
}

void Slave::link_output(std::string topic_name)
{
    this->output.shutdown();
    ROS_INFO("Linking output %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->output=this->nh.advertise<geometry_msgs::Twist>(topic_name,10);
}




void Slave::input_callback(geometry_msgs::Twist msg)
{
    this->set_translation(msg.linear.x,msg.linear.y,msg.linear.z);
    this->set_rotation(msg.angular.x,msg.angular.y,msg.angular.z);

    this->optimal_control();
    this->forward_propagation();
}

///<summary>
///This implements a least sqaures determination of control vector [v,omega] [control.v control.omega] 
///from the given cartesian velocity state d/dt[x,y,phi] (cart_vel)
///</summary>
void Slave::optimal_control()
{
    //calculate the ideal velocity state
    this->cart_rot.z=this->cart_rot_in.z;
    this->cart_vel.x=this->cart_vel_in.x-this->cart_rot_in.z*this->ref_pos.y;
    this->cart_vel.y=this->cart_vel_in.y+this->cart_rot_in.z*this->ref_pos.x;    


    //calculate least sqared state to ideal state
    this->control.v=cos(this->cart_ori.z)*this->cart_vel.x +sin(this->cart_ori.z)*this->cart_vel.y;
    this->control.omega=this->cart_rot_in.z;

    //calculate reached state
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
    ROS_INFO("Started controller with name: %s",this->name.c_str());
    while(ros::ok())
    {
        //publish
        geometry_msgs::Twist msg;
        msg.linear.x=this->control.v;
        msg.linear.y=0.0;
        msg.linear.z=0.0;;
        msg.angular.x=0.0;
        msg.angular.y=0.0;
        msg.angular.z=this->control.omega;
        this->output.publish(msg);
        ros::spinOnce();

        rate.sleep();
    }

}