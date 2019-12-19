#include <controller/slave.h>

Slave::Slave(ros::NodeHandle &nh):Controller(nh)
{
} 


void Slave::target_state_callback(geometry_msgs::PoseStamped msg)
{
    tf::Transform trafo;  
    tf::poseMsgToTF(msg.pose,trafo);
    this->target_pose=trafo*this->reference_pose;    
}

void Slave::target_velocities_callback(geometry_msgs::Twist msg)
{
    tf::Vector3 ang;
    tf::vector3MsgToTF(msg.angular,ang);
    tf::Vector3 lin;
    tf::vector3MsgToTF(msg.linear,lin);

    tf::StampedTransform trafo_master;   
    this->listener->lookupTransform(this->world_frame,"/robot_master/base_footprint",ros::Time(0),trafo_master);
    trafo_master.setOrigin(tf::Vector3(0,0,0));

    this->lin_vel_in=trafo_master*lin+ang.cross(trafo_master*this->reference_pose.getOrigin());
    this->ang_vel_in=ang;

}


void Slave::target_odometry_callback(nav_msgs::Odometry msg)
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
    this->lin_vel_in=rot*lin+ang.cross(trafo*this->reference_pose.getOrigin());
    
    //Calculate position 
    this->target_pose=trafo*this->reference_pose;
    // tf::Vector3 rel;
    // rel=rot*this->reference_pose.getOrigin();

    // //Calculate orientation from velocity constrain
    //this->target_pose.setRotation(tf::createQuaternionFromYaw(atan2(lin_vel_in.y(),lin_vel_in.x())));

    // //Calculate angular velocity
    tf::Vector3 pos;
    pos=this->target_pose.getOrigin();
    this->ang_vel_in.setZ((pos.x()*lin_vel_in.y()-pos.y()*lin_vel_in.x())/(pow(pos.x(),2)+pow(pos.y(),2)));
}


void Slave::optimal_control()
{ 
   
    //Calculate controlvector
    double phi=tf::getYaw(this->current_pose.getRotation());
    this->lin_vel_out.setX(cos(phi)*this->lin_vel_in.x()+sin(phi)*this->lin_vel_in.y());
    this->ang_vel_out=this->ang_vel_in;
}


void Slave::scope()
{   
  
    switch(this->type)
    {
        case pseudo_inverse: this->optimal_control();break;
        case lypanov:
            this->calc_Lyapunov(    this->lyapunov_parameter.kx,
                                    this->lyapunov_parameter.ky,
                                    this->lyapunov_parameter.ktheta,
                                    sqrt(lin_vel_in.x()*lin_vel_in.x()+lin_vel_in.y()*lin_vel_in.y()),
                                    this->ang_vel_in.z());
            break;
        case angle_distance: this->calc_angle_distance(this->kr,this->kang);break;
        default: break;
    }
    
}