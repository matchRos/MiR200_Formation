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
 
    //Get the current velocity
    VelocityEulerian vel_eul;
    //Project to x axis since sometimes odometry velocity is given in parent framen not in child
    vel_eul.v=std::sqrt(std::pow(msg.twist.twist.linear.x,2)+std::pow(msg.twist.twist.linear.y,2)+std::pow(msg.twist.twist.linear.z,2));    //Nessescarry since ros is not consistent with odom msg. Sometimes its lagrangian sometimes eulerian
    VelocityCartesian vel;
    lin=tf::Vector3(vel_eul.v,0.0,0.0);
    
    //Filtering
   
    if(std::abs(lin.x())<0.05){lin.setX(0.0);}
    if(std::abs(lin.y())<0.05){lin.setY(0.0);}
    if(std::abs(ang.z())<0.05){ang.setZ(0.0);}
    
    switch(this->type)
    {
        case ControllerType::lypanov:
        case ControllerType::pseudo_inverse:
        case ControllerType::lyapunov_bidirectional:
        {
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

            
            //Calculate position 
            this->target_state_.pose=trafo*this->world2reference_;
            break;
        }
        
        case ControllerType::angle_distance:
        {            
            tf::poseMsgToTF(msg.pose.pose,this->target_state_.pose);
            this->target_state_.velocity=lin;
            this->target_state_.angular_velocity=ang.z();
            break;
        }
       
    }
    
}


Controller::ControlVector Slave::calcOptimalControl()
{   
    //Calculate controlvector
    ControlVector ret;
    double phi=tf::getYaw(this->current_state_.pose.getRotation());
    ret.v=cos(phi)*this->target_state_.velocity.x()+sin(phi)*this->target_state_.velocity.y();
    ret.omega=this->target_state_.angular_velocity;
    return ret;
}