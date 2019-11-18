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