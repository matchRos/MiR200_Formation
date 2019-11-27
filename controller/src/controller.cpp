#include <controller/controller.h>

Controller::Controller(ros::NodeHandle &nh):    nh(nh),
                                                lin_vel_in{0,0,0},
                                                ang_vel_in{0,0,0}
{
    this->listener=new tf::TransformListener(nh);
    
    this->name="my_slave";
 
    this->pub_vel_out=this->nh.advertise<geometry_msgs::Twist>("/out",10);
    this->pub_state_out=this->nh.advertise<geometry_msgs::PoseStamped>("/state_out",10);
    this->pub_control_difference=this->nh.advertise<geometry_msgs::TransformStamped>("/control_difference",10);
    
    this->sub_vel_target=this->nh.subscribe("/in",10,&Controller::target_velocities_callback,this);
    this->sub_state_target=this->nh.subscribe("/state_target",10,&Controller::target_state_callback,this);
    this->sub_odom_current=this->nh.subscribe("/odom_current",10,&Controller::current_odom_callback,this);

    this->current_pose.setOrigin(tf::Vector3(0,0,0));
    this->current_pose.setRotation(tf::Quaternion(0,0,0,1));
    this->target_pose.setOrigin(tf::Vector3(0,0,0));   
    this->target_pose.setRotation(tf::Quaternion(0,0,0,1));
    this->control_dif.setOrigin(tf::Vector3(0,0,0));
    this->control_dif.setRotation(tf::Quaternion(0,0,0,1));

    this->loaded_parameter=false;
} 

Controller::~Controller()
{
    delete this->listener;
}





//################################################################################################
//Setters
void Controller::set_name(std::string name)
{
    this->name=name;
    this->nh.resolveName(name);

    this->link_output_velocity("mobile_base_controller/cmd_vel");
    this->link_output_state("state");
    this->link_output_ctrldiff("control_dif");
    this->link_target_state("state_in");

   
}

void Controller::set_reference(double x,double y,double z,double angle)
{

    this->reference_pose=tf::Pose();
    this->reference_pose.setOrigin(tf::Vector3(x,y,z));
    this->reference_pose.setRotation(tf::Quaternion(angle,0,0));

    this->current_pose=this->reference_pose;
    this->target_pose=this->current_pose;    

    ROS_INFO("Set coordiantes of: %s to: %lf %lf %lf",this->name.c_str(),   this->reference_pose.getOrigin().x(),
                                                                            this->reference_pose.getOrigin().y(),
                                                                            this->reference_pose.getOrigin().z());

    this->add_map();

   
   
}

void Controller::add_map()
{
    //Publis the trasnformation of a single controller instance to its refernece coordinate system
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    
    tf::Transform trafo(this->reference_pose);

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id =this->world_frame ;
    static_transformStamped.child_frame_id = this->name+"/odom_comb";
    tf::transformTFToMsg(trafo,static_transformStamped.transform);
    static_broadcaster.sendTransform(static_transformStamped);
}


void Controller::set_reference(std::vector<double> coord,double angle)
{
    this->set_reference(coord[0],coord[1],coord[2],angle);
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


void Controller::load()
{
    std::string param;

    if(ros::param::get(PARAM_WORLD_FRAME,param));
    {
        ROS_INFO("Loading %s",PARAM_WORLD_FRAME);
        this->set_world_frame(param);     
    }

   
    if(ros::param::get(PARAM_CURRENT_ODOM,param))
    {
        ROS_INFO("Loading %s",PARAM_CURRENT_ODOM);
        this->link_current_odom(param);
    }

    if(ros::param::get(PARAM_TARGET_VEL,param))
    {
        ROS_INFO("Loading %s ",PARAM_TARGET_VEL);
        this->link_target_velocity(param);
    } 

    if(ros::param::get(PARAM_TARGET_STATE,param))
    {
        ROS_INFO("Loading %s ",PARAM_TARGET_STATE);
        this->link_target_state(param);
    }


    int i;
    if(ros::param::get(PARAM_TYPE,i))
    {
        ROS_INFO("Loading %s ",PARAM_TYPE);
        this->set_type(static_cast<Controller::controllerType>(i));
    }

   
    std::vector<double> lyapunov;
    if( ros::param::get(PARAM_LYAPUNOV,lyapunov))
    {
        this->kx=lyapunov[0];
        this->ky=lyapunov[1];
        this->kphi=lyapunov[2];
        this->vd=lyapunov[3];
        this->omegad=lyapunov[4];
        ROS_INFO("LOADED LYAPUNOV PARAM: %lf %lf %lf %lf %lf", this->kx,this->ky,this->kphi,this->vd,this->omegad);

    }   

    std::vector<double> ang_dist;
    if( ros::param::get(PARAM_ANG_DIST,ang_dist))
    {
       this->kr=ang_dist[0]; 
       this->kang=ang_dist[1];
       ROS_INFO("LOADED ANG_DIST PARAM: %lf %lf ", this->kr,this->kang);

    }   
    
    load_parameter();
    this->loaded_parameter=true;

}

void Controller::load_parameter()
{
    return;
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

 /*Linking topics #################################################################################################################################
##################################################################################################################################################*/
        
//INPUTS
void Controller::link_current_odom(std::string topic_name)
{
    this->sub_odom_current.shutdown();
    ROS_INFO("Linking input currnet odometry of %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->sub_odom_current=this->nh.subscribe(topic_name,10,&Controller::current_odom_callback,this);
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
    ROS_INFO("Linking output velocity %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->pub_vel_out=this->nh.advertise<geometry_msgs::Twist>(topic_name,10);
}


void Controller::link_output_state(std::string topic_name)
{
    this->pub_state_out.shutdown();
    ROS_INFO("Linking output state %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->pub_state_out=this->nh.advertise<geometry_msgs::PoseStamped>(topic_name,10);
}

void Controller::link_output_ctrldiff(std::string topic_name)
{
    this->pub_control_difference.shutdown();
    ROS_INFO("Linking control difference %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->pub_control_difference=this->nh.advertise<geometry_msgs::TransformStamped>(topic_name,10);
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
    geometry_msgs::PoseStamped msg_pose;    
    msg_pose.header.frame_id=this->world_frame;
    msg_pose.header.stamp=ros::Time::now();    
    tf::poseTFToMsg(this->current_pose,msg_pose.pose);
    this->pub_state_out.publish(msg_pose);   

    //publish control difference
    geometry_msgs::TransformStamped trafo;
    tf::transformTFToMsg(this->control_dif,trafo.transform);
    trafo.child_frame_id="target_frame";
    trafo.header.stamp=ros::Time::now();
    this->pub_control_difference.publish(trafo);
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