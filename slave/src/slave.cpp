#include <slave/slave.h>

Slave::Slave(ros::NodeHandle &nh):Controller(nh)
{
} 


void Slave::target_state_callback(geometry_msgs::PoseStamped msg)
{
    tf::Transform trafo;  
    tf::poseMsgToTF(msg.pose,trafo);
    this->target_pose.setOrigin(trafo*this->reference_pose.getOrigin());
    this->target_pose.setRotation(trafo*this->reference_pose.getRotation());
    
}

void Slave::link_master_odom(std::string topic_name)
{
     this->master_odom.shutdown();
    ROS_INFO("Linking input currnet odometry of %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->master_odom=this->nh.subscribe(topic_name,10,&Slave::master_odom_callback,this);
}
void Slave::master_odom_callback(nav_msgs::Odometry msg)
{
    tf::poseMsgToTF(msg.pose.pose,this->master_pose);
    tf::vector3MsgToTF(msg.twist.twist.linear,this->master_lin_vel);
    tf::vector3MsgToTF(msg.twist.twist.angular,this->master_ang_vel);
}


void Slave::optimal_control()
{      
    tf::Vector3 ideal_trans;
    ideal_trans=this->lin_vel_in+this->ang_vel_in.cross(this->world2odom*this->reference_pose.getOrigin());
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
        case pseudo_inverse: this->optimal_control();break;
        case lypanov: this->calc_Lyapunov(this->kx,this->ky,this->kphi,this->vd,this->omegad);break;
        default: break;
    }
    
}