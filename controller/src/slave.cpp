#include <controller/slave.h>

Slave::Slave(   std::string name,
                ros::NodeHandle nh,
                ros::NodeHandle nh_topics,
                ros::NodeHandle nh_parameters):Controller(name,nh,nh_topics,nh_parameters)
{
    this->srv_set_reference_=this->controller_nh.advertiseService("set__master_reference",&Slave::srvSetMasterReference,this);
} 

void Slave::setMasterReference(tf::Pose pose)
{
    this->master_reference_=pose;
    this->target_state_.pose.setOrigin(this->current_state_.pose.getOrigin()-this->master_reference_.getOrigin());
    ROS_INFO("Set coordiantes of: %s relative to master to: %lf %lf %lf",   this->name.c_str(), 
                                                                            this->master_reference_.getOrigin().x(),
                                                                            this->master_reference_.getOrigin().y(),
                                                                            this->master_reference_.getOrigin().z());  
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
    

    //Get the transformation for frames 
    tf::StampedTransform trafo1;
    try{
        this->listener->lookupTransform(this->world_frame,msg.header.frame_id,ros::Time(0),trafo1);
    }
    catch(ros::Exception &ex)
    {
        ROS_WARN("%s",ex.what());
    }
    
    //Filtering   
    if(std::abs(lin.x())<this->thresh_.x()){lin.setX(0.0);}
    if(std::abs(lin.y())<this->thresh_.y()){lin.setY(0.0);}
    if(std::abs(ang.z())<this->thresh_.z()){ang.setZ(0.0);}

    //Get necessary transformations
    tf::Transform trafo;  
    tf::poseMsgToTF(msg.pose.pose,trafo);
    trafo=trafo*trafo1;
    tf::Transform rot;
    rot.setRotation(trafo.getRotation());
    
    switch(this->type)
    {
        case ControllerType::lypanov:
        case ControllerType::pseudo_inverse:
        {
            
            
            
            //Calculate linear velocities
            tf::Vector3 rotational;
            rotational=ang.cross(rot*this->master_reference_.getOrigin());
            this->target_state_.velocity=rot*lin+rotational;
            
            tf::Vector3 acc;
            acc=(this->target_state_.velocity-this->target_state_old_.velocity)/(msg.header.stamp.toSec()-this->time_old_);

            tf::Vector3 vel=this->target_state_.velocity;            
            this->target_state_.angular_velocity=(vel.x()*acc.y()-vel.y()*acc.x())/vel.length2();
            this->target_state_.pose=trafo*this->master_reference_;
            break;
        }
        
        case ControllerType::angle_distance:
        {            
            tf::poseMsgToTF(msg.pose.pose,this->target_state_.pose);
            this->target_state_.velocity=lin;
            this->target_state_.angular_velocity=ang.z();
            // ROS_INFO("odom: %lf, %lf",
            //                                     target_state_.pose.getOrigin().x(),
            //                                     target_state_.pose.getOrigin().y());
            break;
        }       
    }
    this->target_state_old_=this->target_state_;
    this->time_old_=msg.header.stamp.toSec();
    // this->time_buffer_.push_back(msg.header.stamp.toSec());
    // this->velocity_buffer_.push_back(target_state_.velocity);
}



Controller::ControlVector Slave::calcAngleDistance(AngleDistanceParameter parameter,ControlState target, ControlState current)
{
    float l12d=this->master_reference_.getOrigin().length();
    float psi12d=atan2(master_reference_.getOrigin().y(),master_reference_.getOrigin().x());
    if(psi12d<0.0){psi12d+=2*M_PI;}
    float theta1=tf::getYaw(target.pose.getRotation());
    if(theta1<0.0){theta1+=2*M_PI;}
    float theta2=tf::getYaw(target.pose.getRotation());
    if(theta2<0.0){theta2+=2*M_PI;}

   

    tf::Vector3 r=current.pose.getOrigin()-target.pose.getOrigin();   
    float l12=r.length();

    float psi12=atan2(r.y(),r.x());
    if(psi12<0.0){psi12+=2*M_PI;}
    // ROS_INFO("target: %lf, %lf ,%lf,%lf",   master_reference_.getOrigin().x(),
    //                                         master_reference_.getOrigin().y(),
    //                                         target.pose.getOrigin().x(),
    //                                         target.pose.getOrigin().y());

    // ROS_WARN("current: %lf,%lf,%lf,%lf",    r.x(),
    //                                         r.y(),
    //                                         current.pose.getOrigin().x(),
    //                                         current.pose.getOrigin().y());

    float omega1=target.angular_velocity;
    float omega2=current.angular_velocity;
    float v1=target.velocity.length();
    float v2=current.velocity.length();
    

    float gamma1=theta1+psi12-theta2;
    float roh12=(parameter.linear_gain*(l12d-l12)+v1*cos(psi12))/cos(gamma1);

    ROS_INFO("diff: %lf,%lf,%lf,%lf",l12d,psi12d,l12,psi12);
    ControlVector u;
    u.omega=cos(gamma1)/parameter.d*(parameter.angular_gain*l12*(psi12d-psi12)-v1*sin(psi12)+l12*omega1+roh12*sin(gamma1));
    u.v=roh12-parameter.d*omega2*tan(gamma1);

    return u;
}

bool Slave::srvSetMasterReference(multi_robot_msgs::SetInitialPoseRequest &req,multi_robot_msgs::SetInitialPoseResponse &res )
{
    tf::Pose pose;
    tf::poseMsgToTF(req.initial_pose,pose);
    this->setMasterReference(pose);
    res.succeded=true;   
    return true;
}