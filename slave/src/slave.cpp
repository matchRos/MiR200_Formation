#include <slave/slave.h>

Slave::Slave(ros::NodeHandle &nh):Controller(nh)
{
     this->master_odom=this->nh.subscribe("/robot_master/mobile_base_controller/odom",10,&Slave::master_odom_callback,this);
} 

void Slave::link_master_odom(std::string topic_name)
{
    this->master_odom.shutdown();
    ROS_INFO("Linking input currnet odometry of %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->master_odom=this->nh.subscribe(topic_name,10,&Slave::master_odom_callback,this);
}


void Slave::master_odom_callback(nav_msgs::Odometry msg)
{
    tf::Transform trafo;
    tf::poseMsgToTF(msg.pose.pose,trafo);
    //this->target_pose=this->reference_pose;
}



void Slave::optimal_control()
{      
    tf::Vector3 ideal_trans;
    ideal_trans=this->lin_vel_in+this->ang_vel_in.cross(this->reference_pose.getOrigin());
    tf::Vector3 ideal_rot;
    ideal_rot=this->ang_vel_in;

    //Calculate controlvector
    double phi;
    phi=tf::getYaw(this->current_pose.getRotation());
    this->lin_vel_out.setX(cos(phi)*ideal_trans.x()+sin(phi)*ideal_trans.y());
    this->ang_vel_out=ideal_rot;
}


void Slave::scope()
{   
    switch(this->type)
    {
        case pseudo_inverse: break;
        case lypanov: this->calc_Lyapunov(this->kx,this->ky,this->kphi,this->vd,this->omegad);break;
        default: break;
    }
    
}