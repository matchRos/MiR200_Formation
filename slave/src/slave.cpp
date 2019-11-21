#include <slave/slave.h>

Slave::Slave(ros::NodeHandle &nh):Controller(nh)
{
} 


void Slave::target_state_callback(geometry_msgs::PoseStamped msg)
{
    tf::Transform trafo;  
    tf::poseMsgToTF(msg.pose,trafo);
    this->target_pose=trafo*this->reference_pose;    
}





void Slave::optimal_control()
{ 
    tf::StampedTransform trafo_master;   
    this->listener->lookupTransform(this->world_frame,"/robot_master/base_footprint",ros::Time(0),trafo_master);
    trafo_master.setOrigin(tf::Vector3(0,0,0));
    this->lin_vel_in=trafo_master*this->lin_vel_in;
    

    tf::Vector3 ideal_trans=this->lin_vel_in+this->ang_vel_in.cross(trafo_master*this->reference_pose.getOrigin());
   

    //Calculate controlvector
    double phi=tf::getYaw(this->current_pose.getRotation());
    this->lin_vel_out.setX(cos(phi)*ideal_trans.x()+sin(phi)*ideal_trans.y());
    this->ang_vel_out=this->ang_vel_in;
}


void Slave::scope()
{   
    switch(this->type)
    {
        case pseudo_inverse: this->optimal_control();break;
        case lypanov: this->calc_Lyapunov(this->kx,this->ky,this->kphi,this->vd,this->omegad);break;
        case angle_distance: this->calc_angle_distance(this->kr,this->kang);break;
        default: break;
    }
    
}