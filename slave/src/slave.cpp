#include <slave/slave.h>

Slave::Slave(ros::NodeHandle &nh):Controller(nh)
{
} 


void Slave::optimal_control()
{   
    //Calcualte ideal velocities
    // geometry_msgs::Vector3 linear;
    // geometry_msgs::Vector3 angular;
    
    // linear=this->msg_velocities_in.linear;
    // angular=this->msg_velocities_in.angular;

    // this->msg_velocities_ideal.angular.x=angular.x;
    // this->msg_velocities_ideal.angular.y=angular.y;
    // this->msg_velocities_ideal.angular.z=angular.z;
    
    tf::Vector3 ideal_trans;
    ideal_trans=this->lin_vel_in+this->ang_vel_in.cross(this->reference_pose.getOrigin());
    tf::Vector3 ideal_rot;
    ideal_rot=this->ang_vel_in;

    //Calculate controlvector
    double phi;
    phi=this->current_pose.getRotation().getAngle(); 
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