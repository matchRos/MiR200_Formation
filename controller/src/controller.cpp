#include <controller/controller.h>

Controller::Controller(ros::NodeHandle &nh):    nh(nh),
                                                control_dif_(tf::createIdentityQuaternion(),
                                                            tf::Vector3(0,0,0))
{
                                           
    this->listener=new tf::TransformListener(nh);
    this->name=nh.getNamespace();

    //Publishers
    this->pub_cmd_vel=this->nh.advertise<geometry_msgs::Twist>("/out",10);
    this->pub_control_data=this->nh.advertise<multi_robot_msgs::ControlData>("/control_data",10);
    
    //Subscribers
    this->sub_odom_current=this->nh.subscribe("/odom_current",10,&Controller::currentOdomCallback,this);   
    this->sub_odom_target=this->nh.subscribe("/odom_target",10,&Controller::targetOdomCallback,this);
    
    //Services
    this->srv_reset=nh.advertiseService("reset",&Controller::srvReset,this);
    this->srv_set_initial=nh.advertiseService("set_pose",&Controller::srvSetInitial,this);


    //Timers
    this->time_scope_ = nh.createTimer(ros::Duration(0.005),&Controller::execute,this);

    //Flags
    this->loaded_parameter=false;

    this->target_state_.pose=tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0,0,0));
    this->target_state_.angular_velocity=0;
    this->target_state_.velocity=VelocityCartesian(0,0,0);


    this->current_state_.pose=tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0,0,0));
    this->current_state_.angular_velocity=0;
    this->current_state_.velocity=VelocityCartesian(0,0,0);

} 

Controller::~Controller()
{
    delete this->listener;
}

void Controller::reset()
{  
    if(loaded_parameter)
    {
        this->load();    
    }
    else
    {
        this->publishReference();
    }
    
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
void Controller::setName(std::string name)
{
    this->name=name;
    this->nh.resolveName(name);

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

void Controller::setReference(std::vector<double> coord,double angle)
{
    this->setReference(coord[0],coord[1],coord[2],angle);
}

void Controller::setReference(double x,double y,double z,double angle)
{
    setReference(tf::Pose(tf::createQuaternionFromRPY(0,0,angle),tf::Vector3(x,y,z)));  
    
}

void Controller::setType(Controller::ControllerType type)
{
    ROS_INFO("Setting controller type of %s to: %i",this->name.c_str(),this->type);
    this->type=type;
}

void Controller::setWorldFrame(std::string frame)
{
    this->world_frame=frame;
    ROS_INFO("Setting world frame of %s to: %s",this->name.c_str(),this->world_frame.c_str());
}

void Controller::setLyapunov(Controller::LyapunovParameter param)
{
    this->lyapunov_parameter=param;
}
void Controller::setLyapunov(std::vector<float> param)
{
    Controller::LyapunovParameter parameter;
    parameter.kx=param[0];
    parameter.ky=param[1];
    parameter.kphi=param[2];
    this->setLyapunov(parameter);
}

void Controller::load()
{
    std::string param;
    ros::NodeHandle param_nh(this->nh.resolveName("controller"));

    //Load the world frame name
    if(param_nh.getParam(PARAM_WORLD_FRAME,param))
    {
        ROS_INFO("Loading %s",PARAM_WORLD_FRAME);
        this->setWorldFrame(param);     
    }
    else
    {
        ROS_INFO("Could not load %s for %s",PARAM_WORLD_FRAME,this->name.c_str());
    }
   
    //Load current odometry topic
    if(param_nh.getParam(PARAM_CURRENT_ODOM,param))
    {
        ROS_INFO("Loading %s",PARAM_CURRENT_ODOM);
        this->linkCurrentOdom(param);
    }
    else
    {
        ROS_INFO("Could not load %s for %s",PARAM_CURRENT_ODOM,this->name.c_str());
    }

    //Load Target odometry topic
    if(param_nh.getParam(PARAM_TARGET_ODOM,param))
    {
        ROS_INFO("Loading %s",PARAM_TARGET_ODOM);
        this->linkTargetOdom(param);
    }
    else
    {
        ROS_INFO("Could not load %s for %s",PARAM_TARGET_ODOM,this->name.c_str());
    }
    
    //Load Type of controller
    int i;
    if(param_nh.getParam(PARAM_TYPE,i))
    {
        ROS_INFO("Loading %s ",PARAM_TYPE);
        this->setType(static_cast<Controller::ControllerType>(i));
    }
    
    //Load lyapunov parameter
    std::vector<float> lyapunov;
    if( param_nh.getParam(PARAM_LYAPUNOV,lyapunov))
    {
        this->setLyapunov(lyapunov);
    } 
    
    //Load parameter for publishing tf
    if(!param_nh.getParam(PARAM_PUBISH_TF,this->publish_tf_))
    {
        publish_tf_=false;
    }    
    loadParameter();
    this->loaded_parameter=true;

}

void Controller::loadParameter()
{
    return;
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
    this->publishReference();     
    if(this->publish_tf_){this->publishBaseLink();}   
}

void Controller::publishReference()
{
    //Publish reference link
    geometry_msgs::TransformStamped msg2;
    msg2.header.stamp = ros::Time::now();
    msg2.header.frame_id =this->world_frame ;
    msg2.child_frame_id=this->nh.resolveName("reference");
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
                                    nh.resolveName("reference"),
                                    nh.resolveName("base_footprint"));
    this->broadcaster_.sendTransform(base_link);
}



/*Calculations executions and scopes####################################################################################################################
##################################################################################################################################################*/
Controller::ControlVector Controller::calcLyapunov(LyapunovParameter parameter,VelocityEulerian desired,tf::Transform relative)
{
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
    output.v=parameter.kx*x+desired.v*cos(phi);
    output.omega=parameter.kphi*sin(phi)+parameter.ky*desired.v*y+desired.omega;

    return output;
}

Controller::ControlVector Controller::calcAngleDistance(AngleDistanceParameter parameter,ControlState target, ControlState current)
{
    float l12d=this->world2reference_.getOrigin().length();
    float psi12d=atan2(-world2reference_.getOrigin().y(),-world2reference_.getOrigin().x());
    if(psi12d<0.0){psi12d+=2*M_PI;}
    float theta1=tf::getYaw(target_state_.pose.getRotation());
    if(theta1<0.0){theta1+=2*M_PI;}
    float theta2=tf::getYaw(current_state_.pose.getRotation());
    if(theta2<0.0){theta2+=2*M_PI;}
    tf::Vector3 r=current.pose.inverseTimes(target.pose).getOrigin();
    
    
    float l12=r.length();
    float psi12=atan2(r.y(),r.x());
    if(psi12<0.0){psi12+=2*M_PI;}
    float omega1=current.angular_velocity;
    float omega2=target.angular_velocity;
    float v1=current.velocity.length();
    float v2=target.velocity.length();

    float gamma1=theta1+psi12-theta2;
    float roh12=(parameter.linear_gain*(l12d-l12)+v1*cos(psi12))/cos(gamma1);


    ROS_WARN("x %f, y%f ,l12d: %f psi12d: %f theta1: %f theta2: %f psi12: %f l12: %f gamma1: %f roh12: %f",
                r.x(),r.y(),l12d,psi12d,theta1,theta2,psi12,l12,gamma1,roh12);
    
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
        case disable:
            break;    
        case pseudo_inverse: 
            this->control_=calcOptimalControl();
            this->publish();    
            break;
        case lypanov:
            desired.omega=this->target_state_.angular_velocity;
            desired.v=sqrt(pow(this->target_state_.velocity.getX(),2)+pow(this->target_state_.velocity.getY(),2));           
            this->control_=calcLyapunov(this->lyapunov_parameter,desired,control_dif_);
            this->publish();    
            break;
        case angle_distance:
            AngleDistanceParameter param;
            param.angular_gain=0.05;
            param.linear_gain=0.12;
            param.d=0.3;
            this->control_=calcAngleDistance(param,this->target_state_,this->current_state_);
            this->publish();    
            break;
        default: 
            break;
    }
    
}