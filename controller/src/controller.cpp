#include <controller/controller.h>

Controller::Controller(ros::NodeHandle &nh):    nh(nh),
                                                lin_vel_in{0,0,0},
                                                ang_vel_in{0,0,0}
{
    this->listener=new tf::TransformListener(nh);
    this->name=nh.getNamespace();

    this->pub_vel_out=this->nh.advertise<geometry_msgs::Twist>("/out",10);
    this->pub_state_out=this->nh.advertise<multi_robot_msgs::State>("/state_out",10);
    this->pub_control_data=this->nh.advertise<multi_robot_msgs::ControlData>("/control_data",10);
    
    this->sub_vel_target=this->nh.subscribe("/in",10,&Controller::target_velocities_callback,this);
    this->sub_state_target=this->nh.subscribe("/state_target",10,&Controller::target_state_callback,this);
    this->sub_odom_current=this->nh.subscribe("/odom_current",10,&Controller::current_odom_callback,this);   

    this->reset_service=nh.advertiseService("reset",&Controller::srv_reset,this);

    
    this->loaded_parameter=false;
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
        this->add_map();
    }
    
}



//################################################################################################
//Setters
void Controller::set_name(std::string name)
{
    this->name=name;
    this->nh.resolveName(name);

    this->link_output_velocity("mobile_base_controller/cmd_vel");
    this->link_output_state("state");
    this->link_output_control_data("control_data");
    this->link_target_state("state_in");   
}



void Controller::set_reference(double x,double y,double z,double angle)
{
    
    this->reference_pose=tf::Pose(tf::createQuaternionFromYaw(angle),tf::Vector3(x,y,z));
    this->current_pose=this->reference_pose;
    this->target_pose=this->current_pose;
    ROS_INFO("Set coordiantes of: %s to: %lf %lf %lf",this->name.c_str(),   this->reference_pose.getOrigin().x(),
                                                                            this->reference_pose.getOrigin().y(),
                                                                            this->reference_pose.getOrigin().z());
    this->add_map();   
}

void Controller::set_reference(std::vector<double> coord,double angle)
{
    this->set_reference(coord[0],coord[1],coord[2],angle);
}


void Controller::add_map()
{
    //Publish the trasnformation of a single controller instance to its refernece coordinate system
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    
    tf::Transform trafo(this->reference_pose);

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id =this->world_frame ;
    static_transformStamped.child_frame_id = this->name+"/odom_comb";
    tf::transformTFToMsg(trafo,static_transformStamped.transform);
    static_broadcaster.sendTransform(static_transformStamped);
}


void Controller::set_type(Controller::controllerType type)
{
    ROS_INFO("Setting controller type of %s to: %i",this->name.c_str(),this->type);
    this->type=type;
}

void Controller::set_world_frame(std::string frame)
{
    this->world_frame=frame;
    ROS_INFO("Setting world frame of %s to: %s",this->name.c_str(),this->world_frame.c_str());
}

void Controller::set_lyapunov(Controller::lyapunov param)
{
    this->lyapunov_parameter.kx=param.kx;
    this->lyapunov_parameter.ky=param.ky;
    this->lyapunov_parameter.ktheta=param.ktheta;

}
void Controller::set_lyapunov(std::vector<float> param)
{
    Controller::lyapunov parameter{param[0],param[1],param[2]};
    this->set_lyapunov(parameter);
}

void Controller::load()
{
    std::string param;

    if(ros::param::get(PARAM_WORLD_FRAME,param))
    {
        ROS_INFO("Loading %s",PARAM_WORLD_FRAME);
        this->set_world_frame(param);     
    }
    else
    {
        ROS_INFO("Could not load %s for %s",PARAM_WORLD_FRAME,this->name.c_str());
    }
   
    if(ros::param::get(PARAM_CURRENT_ODOM,param))
    {
        ROS_INFO("Loading %s",PARAM_CURRENT_ODOM);
        this->link_current_odom(param);
    }
    else
    {
        ROS_INFO("Could not load %s for %s",PARAM_CURRENT_ODOM,this->name.c_str());
    }

    if(ros::param::get(PARAM_TARGET_ODOM,param))
    {
        ROS_INFO("Loading %s",PARAM_TARGET_ODOM);
        this->link_target_odometry(param);
    }
    else
    {
        ROS_INFO("Could not load %s for %s",PARAM_TARGET_ODOM,this->name.c_str());
    }
    if(ros::param::get(PARAM_TARGET_VEL,param))
    {
        ROS_INFO("Loading %s ",PARAM_TARGET_VEL);
        this->link_target_velocity(param);
    }
    else
    {
        ROS_INFO("Could not load %s for %s",PARAM_TARGET_VEL,this->name.c_str());
    }
    if(ros::param::get(PARAM_TARGET_STATE,param))
    {
        ROS_INFO("Loading %s ",PARAM_TARGET_STATE);
        this->link_target_state(param);
    }
    else
    {
        ROS_INFO("Could not load %s for %s",PARAM_TARGET_STATE,this->name.c_str());
    }

    int i;
    if(ros::param::get(PARAM_TYPE,i))
    {
        ROS_INFO("Loading %s ",PARAM_TYPE);
        this->set_type(static_cast<Controller::controllerType>(i));
    }

   
    std::vector<float> lyapunov;
    if( ros::param::get(PARAM_LYAPUNOV,lyapunov))
    {
        this->set_lyapunov(lyapunov);
        ROS_INFO("LOADED %s  PARAM: %lf %lf %lf %lf %lf",PARAM_LYAPUNOV ,this->kx,this->ky,this->kphi,this->vd,this->omegad);
    }   

    std::vector<double> ang_dist;
    if( ros::param::get(PARAM_ANG_DIST,ang_dist))
    {
       this->kr=ang_dist[0]; 
       this->kang=ang_dist[1];
       ROS_INFO("LOADED %s PARAM: %lf %lf ",PARAM_ANG_DIST, this->kr,this->kang);

    }   
    
    load_parameter();
    this->loaded_parameter=true;

}

void Controller::load_parameter()
{
    return;
}


 /*Linking topics #################################################################################################################################
##################################################################################################################################################*/
        
//INPUTS
void Controller::link_current_odom(std::string topic_name)
{
    this->sub_odom_current.shutdown();
    ROS_INFO("Linking input currnet odometry of %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->sub_odom_current=this->nh.subscribe(topic_name,10,&Controller::current_odom_callback,this);
}

void Controller::link_target_odometry(std::string topic_name)
{
    this->sub_target_odometry.shutdown();
    ROS_INFO("Linking input target odometry of %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->sub_target_odometry=this->nh.subscribe(topic_name,10,&Controller::target_odometry_callback,this);
}

void Controller::link_target_state(std::string topic_name)
{
    this->sub_state_target.shutdown();
    ROS_INFO("Linking target state %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->sub_state_target=this->nh.subscribe(topic_name,10,&Controller::target_state_callback,this);
}

void Controller::link_target_velocity(std::string topic_name)
{
    this->sub_vel_target.shutdown();
    ROS_INFO("Linking input velocity %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->sub_vel_target=this->nh.subscribe(topic_name,10,&Controller::target_velocities_callback,this);
}


///OUTPUTS
void Controller::link_output_velocity(std::string topic_name)
{
    this->pub_vel_out.shutdown();
    ROS_INFO("Linking output velocity of %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->pub_vel_out=this->nh.advertise<geometry_msgs::Twist>(topic_name,10);
}

void Controller::link_output_state(std::string topic_name)
{
    this->pub_state_out.shutdown();
    ROS_INFO("Linking output state of %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->pub_state_out=this->nh.advertise<multi_robot_msgs::State>(topic_name,10);
}

void Controller::link_output_control_data(std::string topic_name)
{
    this->pub_control_data.shutdown();
    ROS_INFO("Linking control data of %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->pub_control_data=this->nh.advertise<multi_robot_msgs::ControlData>(topic_name,10);
}





 /*Callbacks########################################################################################################################################
##################################################################################################################################################*/
void Controller::current_odom_callback(nav_msgs::Odometry msg)
{
    tf::Pose pose;
    tf::poseMsgToTF(msg.pose.pose,pose); 
    tf::StampedTransform trafo;
    this->listener->lookupTransform(this->world_frame,msg.header.frame_id,ros::Time(0),trafo);
    this->current_pose=trafo*pose;
}

void Controller::target_velocities_callback(geometry_msgs::Twist msg)
{
    tf::vector3MsgToTF(msg.linear,this->lin_vel_in);
    tf::vector3MsgToTF(msg.angular,this->ang_vel_in);
}

void Controller::target_state_callback(geometry_msgs::PoseStamped msg)
{
    tf::poseMsgToTF(msg.pose,this->target_pose);
}

void Controller::target_odometry_callback(nav_msgs::Odometry msg)
{
    tf::poseMsgToTF(msg.pose.pose,this->target_pose);
    tf::vector3MsgToTF(msg.twist.twist.linear,this->lin_vel_in);
    tf::vector3MsgToTF(msg.twist.twist.angular,this->ang_vel_in);
}

 bool Controller::srv_reset(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
 {
     this->reset();
     return true;
 }




/*Calculations and executions ####################################################################################################################
##################################################################################################################################################*/
void Controller::getTransformation()
{
    try
    {
        this->listener->lookupTransform(this->world_frame,this->name+"/base_footprint",ros::Time(0),this->world2robot);

        this->listener->lookupTransform(this->world_frame,this->name+"/odom_comb",ros::Time(0),this->world2odom);
    }
    catch (tf::TransformException ex)
    {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
    }
}

void Controller::publish()
{
    //publish output velocities 
    geometry_msgs::Twist msg_vel;
    tf::vector3TFToMsg(this->lin_vel_out,msg_vel.linear);
    tf::vector3TFToMsg(this->ang_vel_out,msg_vel.angular);
    this->pub_vel_out.publish(msg_vel);

    //publish output pose state
    multi_robot_msgs::State msg_state;    
    msg_state.pose.x=this->current_pose.getOrigin().x();
    msg_state.pose.y=this->current_pose.getOrigin().y();
    msg_state.pose.theta=tf::getYaw(this->current_pose.getRotation());
    this->pub_state_out.publish(msg_state);   

    //publish control data
    multi_robot_msgs::ControlData msg_data;
    msg_data.header.stamp=ros::Time::now();
    msg_data.header.frame_id=this->world_frame;

    //INPUTS    
    tf::vector3TFToMsg(this->target_pose.getOrigin(),msg_data.position_in);
    msg_data.angular_velocity_in=this->ang_vel_in.z();
    tf::vector3TFToMsg(this->lin_vel_in,msg_data.velocity_in); 
    msg_data.angle_in=tf::getYaw(this->target_pose.getRotation());
    

    msg_data.linear_velocity_out=this->lin_vel_out.x();
    msg_data.angular_velocity_out=this->ang_vel_out.z();

    tf::vector3TFToMsg(this->current_pose.getOrigin()-this->target_pose.getOrigin(),msg_data.control_difference_cart);
    msg_data.control_difference_angular=tf::getYaw(this->control_dif.getRotation());

    this->pub_control_data.publish(msg_data);

}

void Controller::calc_Lyapunov(double kx, double ky, double kphi,double vd,double omegad)
{
    tf::Pose relative;
    relative=this->current_pose.inverseTimes(this->target_pose);
    this->control_dif=relative;

    double x=relative.getOrigin().getX();
    double y=relative.getOrigin().getY();
    double phi=tf::getYaw(relative.getRotation());

    

    this->lin_vel_out.setX(kx*x+vd*cos(phi));
    
    this->ang_vel_out.setZ(kphi*sin(phi)+ky*vd*y+omegad); 
}

void Controller::calc_angle_distance(double kr,double kphi)
{
    tf::Pose relative;
    relative=this->current_pose.inverseTimes(this->target_pose);
    this->control_dif=relative;

    if(abs(atan2(relative.getOrigin().y(),relative.getOrigin().x()))>M_PI/2)
    {
        kr*=-1;
    }
    this->lin_vel_out.setX(kr*relative.getOrigin().length());
    this->ang_vel_out.setZ(kphi*tf::getYaw(relative.getRotation())); 
      
    
}

void Controller::execute()
{
    this->getTransformation();
    this->scope();
    this->publish();
     ros::spinOnce();
}