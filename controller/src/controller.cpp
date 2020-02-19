#include <controller/controller.h>

Controller::Controller( std::string name,
                        ros::NodeHandle nh,
                        ros::NodeHandle nh_topics,
                        ros::NodeHandle nh_parameters
                        )                                                        
{
    this->controller_nh=nh_parameters;
    this->robot_nh_=nh_topics;
    //Setting up a nescessary transform lsitener                                    
    this->listener=new tf::TransformListener(nh_topics);

    //Setting up timer for execution;
    this->time_scope_ = nh.createTimer(ros::Duration(0.005),&Controller::execute,this);
    //Setting of controller name
    this->setName(name);
    //Loading parameter from parameter server
    this->load();
    //Initialize control difference
    control_dif_=tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0,0,0));
    this->srv_set_pose=this->controller_nh.advertiseService("set_reference",&Controller::srvSetInitial,this);
} 

Controller::~Controller()
{
    delete this->listener;
}

void Controller::reset()
{  
    this->load();    
}


void Controller::controlState2controlStateMsg(Controller::ControlState &state,multi_robot_msgs::ControlState &msg)
{
    tf::poseTFToMsg(state.pose,msg.pose);
    msg.angle=tf::getYaw(state.pose.getRotation());
    msg.angular_velocity=state.angular_velocity;
    tf::vector3TFToMsg(state.velocity,msg.velocity);
}
void Controller::controlDifference2controlDifferenceMsg(ControlDifference &difference,multi_robot_msgs::ControlDifference &msg)
{
    msg.angle=tf::getYaw(difference.getRotation());
    tf::vector3TFToMsg(difference.getOrigin(),msg.translation);
}     
void Controller::controlVector2controlVectorMsg(ControlVector &control,multi_robot_msgs::ControlVector &msg)         
{
    msg.angular=control.omega;
    msg.linear=control.v;
}


//##############################################################################################################################################
//Setters#######################################################################################################################################
void Controller::load()
{
    //Load the world frame name
    std::string param;
    if(this->controller_nh.getParam(PARAM_WORLD_FRAME,param))
    {
        this->setWorldFrame(param);     
    }
    else
    {
        ROS_WARN("Could not load %s",this->controller_nh.resolveName(PARAM_WORLD_FRAME).c_str());
    }
    
    //Load the reference of the controller input data
    std::vector<float> ref;
    if(this->controller_nh.getParam(PARAM_REFERENCE_POSE,ref))
    {
        if(ref.size()!=6)
        {
            throw std::invalid_argument("Wrong number of Pose parameter!");
        }
        tf::Pose pose(tf::createQuaternionFromRPY(ref[3],ref[4],ref[5]),tf::Vector3(ref[0],ref[1],ref[2]));
        this->setReference(pose);
    }
    else
    {
        ROS_WARN("Could not load %s",this->controller_nh.resolveName(PARAM_REFERENCE_POSE).c_str());
    }

    //Load current odometry topic
    if(this->controller_nh.getParam(PARAM_CURRENT_ODOM,param))
    {
        this->linkCurrentOdom(param);
    }
    else
    {
       ROS_WARN("Could not load %s",this->controller_nh.resolveName(PARAM_CURRENT_ODOM).c_str());
    }


    //Load Target odometry topic
    if(this->controller_nh.getParam(PARAM_TARGET_ODOM,param))
    {
        this->linkTargetOdom(param);
    }
    else
    {
        ROS_WARN("Could not load %s",this->controller_nh.resolveName(PARAM_TARGET_ODOM).c_str());
    }

    
    //Load Type of controller
    int i;
    if(this->controller_nh.getParam(PARAM_TYPE,i))
    {
        this->setType(static_cast<Controller::ControllerType>(i));
    }
    else
    {
        ROS_WARN("Could not load %s",this->controller_nh.resolveName(PARAM_TYPE).c_str());
    }

    //Load parameter if tf should be published
    if(!this->controller_nh.getParam(PARAM_PUBISH_TF,this->publish_tf_))
    {
        ROS_WARN("Could not load %s",this->controller_nh.resolveName(PARAM_PUBISH_TF).c_str());
    }
    
    //Load lyapunov parameter
    std::vector<float> lyapunov;
    if(this->controller_nh.getParam(PARAM_LYAPUNOV,lyapunov))
    {
        this->setControlParameter(LyapunovParameter(lyapunov));
    } 
    else
    {
       ROS_WARN("Could not load %s",this->controller_nh.resolveName(PARAM_LYAPUNOV).c_str());
    }

    
    //Load angle distance parameter
    std::vector<float> angle_distance;
    if(this->controller_nh.getParam(PARAM_ANG_DIST,angle_distance))
    {
        this->setControlParameter(AngleDistanceParameter(angle_distance));
    }
    else
    {
        ROS_WARN("Could not load %s",this->controller_nh.resolveName(PARAM_ANG_DIST).c_str());
    }   
  
}


void Controller::setName(std::string name)
{
    this->name=name;

    this->linkOutputVelocity("mobile_base_controller/cmd_vel");
    this->linkOutputControlData("control_data");   
}


void Controller::setReference(tf::Pose pose)
{
    this->world2reference_=pose;
    this->current_state_.pose=this->world2reference_;
    this->target_state_.pose=this->world2reference_;
    ROS_INFO("Set coordiantes of: %s to: %lf %lf %lf",this->name.c_str(),   this->world2reference_.getOrigin().x(),
                                                                            this->world2reference_.getOrigin().y(),
                                                                            this->world2reference_.getOrigin().z());
    this->publishReference();   
}


void Controller::setType(Controller::ControllerType type)
{  
    this->type=type;
    ROS_INFO("Setting controller type of %s to: %i",this->name.c_str(),this->type); 
}

void Controller::setWorldFrame(std::string frame)
{
    this->world_frame=frame;
    ROS_INFO("Setting world frame of %s to: %s",this->name.c_str(),this->world_frame.c_str());
}

void Controller::setControlParameter(Controller::LyapunovParameter param)
{
    this->lyapunov_parameter_=param;
    ROS_INFO("Set lyapunov parameter to: X gain: %f, Y gain: %f, Phi gain: %f",param.kx,param.ky,param.kphi);
}

void Controller::setControlParameter(Controller::AngleDistanceParameter param)
{
    this->angle_distance_parameter_=param;
    ROS_INFO("Set angular distance parameter to: Angular gain: %f, Linear gain %f, Coliision difference %f",param.angular_gain,param.linear_gain,param.d);
}




 /*Linking topics #################################################################################################################################
##################################################################################################################################################*/
        
//INPUTS
void Controller::linkCurrentOdom(std::string topic_name)
{
    this->sub_odom_current.shutdown();
    ROS_INFO("Linking input currnet odometry of %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->sub_odom_current=this->nh.subscribe(topic_name,10,&Controller::currentOdomCallback,this);
}

void Controller::linkTargetOdom(std::string topic_name)
{
    this->sub_odom_target.shutdown();
    ROS_INFO("Linking input target odometry of %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->sub_odom_target=this->nh.subscribe(topic_name,10,&Controller::targetOdomCallback,this);
}

///OUTPUTS
void Controller::linkOutputVelocity(std::string topic_name)
{
    this->pub_cmd_vel.shutdown();
    ROS_INFO("Linking output velocity of %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->pub_cmd_vel=this->nh.advertise<geometry_msgs::Twist>(topic_name,10);
}

void Controller::linkOutputControlData(std::string topic_name)
{
    this->pub_control_data.shutdown();
    ROS_INFO("Linking control data of %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->pub_control_data=this->nh.advertise<multi_robot_msgs::ControlData>(topic_name,10);
}





 /*Callbacks########################################################################################################################################
##################################################################################################################################################*/
void Controller::currentOdomCallback(nav_msgs::Odometry msg)
{
    //Get the current pose defined in header frame
    tf::Pose pose;
    tf::poseMsgToTF(msg.pose.pose,pose);
    
    //Get the current velocity
    VelocityEulerian vel_eul;
    //Project to x axis since sometimes odometry velocity is given in parent framen not in child
    vel_eul.v=std::sqrt(std::pow(msg.twist.twist.linear.x,2)+std::pow(msg.twist.twist.linear.y,2)+std::pow(msg.twist.twist.linear.z,2));    //Nessescarry since ros is not consistent with odom msg. Sometimes its lagrangian sometimes eulerian
    VelocityCartesian vel;
    vel=tf::Vector3(vel_eul.v,0.0,0.0);

    //Get the transformation for frames 
    tf::StampedTransform trafo;
    try{
        this->listener->lookupTransform(this->world_frame,msg.header.frame_id,ros::Time(0),trafo);
    }
    catch(ros::Exception &ex)
    {
        ROS_WARN("%s",ex.what());
    }
    //Get the rotation part
    tf::Transform rot;
    rot.setRotation(trafo.getRotation());
    //Transfor pose and velocity
    pose=trafo*pose;
    vel=tf::Transform(pose.getRotation(),tf::Vector3(0.0,0.0,0.0))*vel;
           

    //Write to member
    this->current_state_.pose=pose;
    this->current_state_.velocity=vel;
    this->current_state_.angular_velocity=msg.twist.twist.angular.z;
}

void Controller::targetOdomCallback(nav_msgs::Odometry msg)
{
    //Get the current pose
    tf::Pose pose;
    tf::poseMsgToTF(msg.pose.pose,pose);
    
    //Get the current cartesian velocity
    VelocityCartesian vel;
    tf::vector3MsgToTF(msg.twist.twist.linear,vel);

    //Get the transformation for frames 
    tf::StampedTransform trafo;
    try{
        this->listener->lookupTransform(this->world_frame,msg.header.frame_id,ros::Time(0),trafo);
    }
    catch(ros::Exception &ex)
    {
        ROS_WARN("%s",ex.what());
    }

    //Get the rotation part
    tf::Transform rot(trafo.getRotation(),tf::Vector3(0,0,0));
    
    //Transform pose and velocity
    pose=trafo*pose;
    vel=rot*vel;

    //Write to member
    this->target_state_.pose=pose;
    this->target_state_.velocity=vel;
    this->target_state_.angular_velocity=msg.twist.twist.angular.z;
}
/*Service routines ####################################################################################################################
##################################################################################################################################################*/
       

bool Controller::srvReset(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    this->reset();
    return true;
}

bool Controller::srvSetInitial(multi_robot_msgs::SetInitialPoseRequest &req,multi_robot_msgs::SetInitialPoseResponse &res )
{
    tf::Pose pose;
    tf::poseMsgToTF(req.initial_pose,pose);
    this->setReference(pose);
    res.succeded=true;   
    return true;
}

/*Publishsing procedures####################################################################################################################
##################################################################################################################################################*/

void Controller::publish()
{
    this->publishVelocityCommand();
    this->publishControlMetaData();
    this->publishReference();     
    if(this->publish_tf_){this->publishBaseLink();}   
}

void Controller::publishReference()
{
    //Publish reference link
    geometry_msgs::TransformStamped msg2;
    msg2.header.stamp = ros::Time::now();
    msg2.header.frame_id =this->world_frame ;
    msg2.child_frame_id=this->robot_nh_.resolveName("reference");
    tf::transformTFToMsg(this->world2reference_,msg2.transform);
    this->broadcaster_.sendTransform(msg2); 
}

void Controller::publishControlMetaData()
{
    //Publish Controller metadata
    multi_robot_msgs::ControlData msg;
    msg.header.frame_id=this->world_frame;
    msg.header.stamp=ros::Time::now();
    controlDifference2controlDifferenceMsg(this->control_dif_,msg.difference);
    controlState2controlStateMsg(this->current_state_,msg.current);
    controlState2controlStateMsg(this->target_state_,msg.target);
    controlVector2controlVectorMsg(this->control_,msg.control);
    this->pub_control_data.publish(msg);
}

void Controller::publishVelocityCommand()
{
    geometry_msgs::Twist msg_vel;
    msg_vel.linear.x=this->control_.v;
    msg_vel.angular.z=this->control_.omega;
    this->pub_cmd_vel.publish(msg_vel);
}

void Controller::publishBaseLink()
{
    //Publish base_link
    tf::StampedTransform base_link( this->world2reference_.inverseTimes(this->current_state_.pose),
                                    ros::Time::now(),
                                    robot_nh_.resolveName("reference"),
                                    robot_nh_.resolveName("base_footprint"));
    this->broadcaster_.sendTransform(base_link);
}



/*Calculations executions and scopes####################################################################################################################
##################################################################################################################################################*/
Controller::ControlVector Controller::calcLyapunov(LyapunovParameter parameter,ControlState target, ControlState current)
{
    double omega=target.angular_velocity;
    double v=sqrt(pow(target.velocity.getX(),2)+pow(target.velocity.getY(),2)); 
    
    tf::Transform relative=current.pose.inverseTimes(target.pose);
    double x=relative.getOrigin().getX();
    double y=relative.getOrigin().getY();
    double phid;
    if(this->target_state_.velocity.length()>0.01)
    {
        phid=std::atan2(this->target_state_.velocity.y(),this->target_state_.velocity.x());
    }
    else
    {
       phid=tf::getYaw(this->target_state_.pose.getRotation());
    }
    
    
    double phi=tf::getYaw(this->current_state_.pose.getRotation());
    phi=phid-phi;
   
    ControlVector output;
    output.v=parameter.kx*x+v*cos(phi);
    output.omega=parameter.kphi*sin(phi)+parameter.ky*v*y+omega;

    return output;
}



Controller::ControlVector Controller::calcAngleDistance(AngleDistanceParameter parameter,ControlState target, ControlState current)
{
    float l12d=this->world2reference_.getOrigin().length();
    float psi12d=atan2(world2reference_.getOrigin().y(),world2reference_.getOrigin().x());
    if(psi12d<0.0){psi12d+=2*M_PI;}
    float theta1=tf::getYaw(target_state_.pose.getRotation());
    if(theta1<0.0){theta1+=2*M_PI;}
    float theta2=tf::getYaw(current_state_.pose.getRotation());
    if(theta2<0.0){theta2+=2*M_PI;}
    
    tf::Vector3 r=target.pose.inverseTimes(current.pose).getOrigin();   
    float l12=r.length();

    float psi12=atan2(r.y(),r.x());
    if(psi12<0.0){psi12+=2*M_PI;}

    float omega1=target.angular_velocity;
    float omega2=current.angular_velocity;
    float v1=target.velocity.length();
    float v2=current.velocity.length();

    float gamma1=theta1+psi12-theta2;
    float roh12=(parameter.linear_gain*(l12d-l12)+v1*cos(psi12))/cos(gamma1);

    ControlVector u;
    u.omega=cos(gamma1)/parameter.d*(parameter.angular_gain*l12*(psi12d-psi12)-v1*sin(psi12)+l12*omega1+roh12*sin(gamma1));
    u.v=roh12-parameter.d*omega2*tan(gamma1);
    return u;
}

Controller::ControlVector Controller::calcOptimalControl()
{
    ControlVector ret;
    ret.v=this->target_state_.velocity.length();
    ret.omega=this->target_state_.angular_velocity;
    return ret;
}


void Controller::execute(const ros::TimerEvent &ev)
{
    VelocityEulerian desired;
    this->control_dif_=this->current_state_.pose.inverseTimes(this->target_state_.pose);    
        
    switch(this->type)
    {
        case ControllerType::disable:
            this->publishReference();     
            if(this->publish_tf_){this->publishBaseLink();}   
            break;    
        case ControllerType::pseudo_inverse: 
            this->control_=calcOptimalControl();
            this->publish();    
            break;
        case ControllerType::lypanov:                   
            this->control_=calcLyapunov(this->lyapunov_parameter_,this->target_state_,this->current_state_);
            this->publish();    
            break;
        case ControllerType::angle_distance:           
            this->control_=calcAngleDistance(this->angle_distance_parameter_,this->target_state_,this->current_state_);
            this->publish();    
            break;            
        default: 
            break;
    }
    
}