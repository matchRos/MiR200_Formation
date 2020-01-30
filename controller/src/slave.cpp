#include <controller/slave.h>

Slave::Slave(ros::NodeHandle &nh):Controller(nh)
{
} 

void Slave::targetOdomCallback(nav_msgs::Odometry msg)
{
    //Transform msg to tf
    tf::Vector3 ang;
    tf::vector3MsgToTF(msg.twist.twist.angular,ang);
    tf::Vector3 lin;
    tf::vector3MsgToTF(msg.twist.twist.linear,lin);

   
    
    //Get necessary transformations
    tf::Transform trafo;  
    tf::poseMsgToTF(msg.pose.pose,trafo);
    tf::Transform rot;
    rot.setRotation(trafo.getRotation());

    
    //Calculate linear velocities
    tf::Vector3 rotational;
    rotational=ang.cross(rot*this->world2reference_.getOrigin());
    this->target_state_.velocity=rot*lin+rotational;
    this->target_state_.angular_velocity=ang.z();
   
    if(std::abs(this->target_state_.velocity.x())<0.01){this->target_state_.velocity.setX(0.0);}
    if(std::abs(this->target_state_.velocity.y())<0.01){this->target_state_.velocity.setY(0.0);}
    // //Calculate position 
    this->target_state_.pose=trafo*this->world2reference_;
    
    
        // //Calculate orientation from velocity constrain
    double phi;
    phi=atan2(this->target_state_.velocity.y(),this->target_state_.velocity.x());
    this->target_state_.pose.setRotation(tf::createQuaternionFromRPY(0.0,0.0,phi));
}


Controller::ControlVector Slave::optimal_control()
{   
    //Calculate controlvector
    double phi=tf::getYaw(this->current_state_.pose.getRotation());
    this->control_.v=(cos(phi)*this->target_state_.velocity.x()+sin(phi)*this->target_state_.velocity.y());
    this->control_.omega=this->target_state_.angular_velocity;
}