#include <controller/controller.h>

Controller::Controller(ros::NodeHandle &nh):    nh(nh),
                                                lin_vel_in{0,0,0},
                                                ang_vel_in{0,0,0}
{
    this->listener=new tf::TransformListener(nh);
    
    this->name="my_slave";
 
    this->vel_out=this->nh.advertise<geometry_msgs::Twist>("/out",10);
    this->state_out=this->nh.advertise<geometry_msgs::PoseStamped>("/state_out",10);
    this->control_difference=this->nh.advertise<geometry_msgs::Transform>("/control_difference",10);
    
    this->vel_target=this->nh.subscribe("/in",10,&Controller::target_velocities_callback,this);
    this->state_target=this->nh.subscribe("/state_target",10,&Controller::target_state_callback,this);
     this->state_current=this->nh.subscribe("/state_current",10,&Controller::current_state_callback,this);
    this->odom_current=this->nh.subscribe("/odom_current",10,&Controller::current_odom_callback,this);

    this->current_pose.setOrigin(tf::Vector3(0,0,0));
    this->current_pose.setRotation(tf::Quaternion(0,0,0,1));
    this->target_pose.setOrigin(tf::Vector3(0,0,0));   
    this->target_pose.setRotation(tf::Quaternion(0,0,0,1));
    this->control_dif.setOrigin(tf::Vector3(0,0,0));
    this->control_dif.setRotation(tf::Quaternion(0,0,0,1));
} 

Controller::~Controller()
{
    delete this->listener;
}





//################################################################################################
//Setter

void Controller::set_name(std::string name)
{
    this->name=name;
    this->nh.resolveName(name);

    this->link_output_velocity("mobile_base_controller/cmd_vel");
    this->link_output_state("state");
    this->link_output_ctrldiff("control_dif");
    this->link_target_state("state_in");

   
}

void Controller::set_reference(double x,double y,double z)
{

    this->reference_pose=tf::Pose();
    this->reference_pose.setOrigin(tf::Vector3(x,y,z));
    this->reference_pose.setRotation(tf::Quaternion(0,0,0,1));

    this->current_pose=this->reference_pose;
    this->target_pose=this->current_pose;
    ROS_INFO("CURRENT: %lf %lf %lf",this->current_pose.getOrigin().getX(),this->current_pose.getOrigin().getY(),this->current_pose.getOrigin().getZ());
    


    //Publis the trasnformation of a single controller instance to its refernece coordinate system
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    


    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id =this->world_frame ;
    static_transformStamped.child_frame_id = this->name+"/odom_comb";
    static_transformStamped.transform.translation.x=x;
    static_transformStamped.transform.translation.y=y;
    static_transformStamped.transform.translation.z=z;
    

    static_transformStamped.transform.rotation.x=0;
    static_transformStamped.transform.rotation.y=0;
    static_transformStamped.transform.rotation.z=0;
    static_transformStamped.transform.rotation.w=1;
    static_broadcaster.sendTransform(static_transformStamped);

    ROS_INFO("Set coordiantes of: %s to: %lf %lf %lf",this->name.c_str(),x,y,z);

}

void Controller::set_reference(std::vector<double> coord)
{
    this->set_reference(coord[0],coord[1],coord[2]);
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

    ros::param::get(PARAM_WORLD_FRAME,param);
    ROS_INFO("Loading %s",PARAM_WORLD_FRAME);
    this->set_world_frame(param); 

    ros::param::get(PARAM_CURRENT_STATE,param);
    ROS_INFO("Loading %s",PARAM_CURRENT_STATE);
    this->link_current_state(param);  


    ros::param::get(PARAM_CURRENT_ODOM,param);
    ROS_INFO("Loading %s",PARAM_CURRENT_ODOM);
    this->link_current_odom(param);


    ros::param::get(PARAM_TARGET_VEL,param);
    ROS_INFO("Loading %s ",PARAM_TARGET_VEL);
    this->link_target_velocity(param);

    ros::param::get(PARAM_TARGET_STATE,param);
    ROS_INFO("Loading %s ",PARAM_TARGET_STATE);
    this->link_target_state(param);



    int i;
    ros::param::get(PARAM_TYPE,i);
    ROS_INFO("Loading %s ",PARAM_TYPE);
    this->set_type(static_cast<Controller::controllerType>(i));

    std::vector<double> coord;
    ros::param::get(PARAM_COORD,coord);
    ROS_INFO("Loading %s ",PARAM_COORD);
    this->set_reference(coord);


    std::vector<double> temp;
    ros::param::get(PARAM_LYAPUNOV,temp);
    this->kx=temp[0];
    this->ky=temp[1];
    this->kphi=temp[2];
    this->vd=temp[3];
    this->omegad=temp[4];

    load_parameter();
}


void Controller::load_parameter()
{
    
}






 /*Linking topics #################################################################################################################################
##################################################################################################################################################*/
        
//INPUTS

void Controller::link_current_odom(std::string topic_name)
{
    this->odom_current.shutdown();
    ROS_INFO("Linking input currnet odometry of %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->odom_current=this->nh.subscribe(topic_name,10,&Controller::current_odom_callback,this);
}

void Controller::link_current_state(std::string topic_name)
{
    this->state_current.shutdown();
    ROS_INFO("Linking current state of %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->state_current=this->nh.subscribe(topic_name,10,&Controller::current_state_callback,this);
}


void Controller::link_target_state(std::string topic_name)
{
    this->state_target.shutdown();
    ROS_INFO("Linking target state %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->state_target=this->nh.subscribe(topic_name,10,&Controller::target_state_callback,this);
}

void Controller::link_target_velocity(std::string topic_name)
{
    this->vel_target.shutdown();
    ROS_INFO("Linking input velocity %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->vel_target=this->nh.subscribe(topic_name,10,&Controller::target_velocities_callback,this);
}


///OUTPUTS
void Controller::link_output_velocity(std::string topic_name)
{
    this->vel_out.shutdown();
    ROS_INFO("Linking output velocity %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->vel_out=this->nh.advertise<geometry_msgs::Twist>(topic_name,10);
}


void Controller::link_output_state(std::string topic_name)
{
    this->state_out.shutdown();
    ROS_INFO("Linking output state %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->state_out=this->nh.advertise<geometry_msgs::PoseStamped>(topic_name,10);
}

void Controller::link_output_ctrldiff(std::string topic_name)
{
    this->control_difference.shutdown();
    ROS_INFO("Linking control difference %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->control_difference=this->nh.advertise<geometry_msgs::Transform>(topic_name,10);
}





 /*Callbacks########################################################################################################################################
##################################################################################################################################################*/
        

void Controller::target_velocities_callback(geometry_msgs::Twist msg)
{
    tf::vector3MsgToTF(msg.linear,this->lin_vel_in);
    tf::vector3MsgToTF(msg.angular,this->ang_vel_in);
}

void Controller::current_odom_callback(nav_msgs::Odometry msg)
{
    tf::Point point;
    tf::pointMsgToTF(msg.pose.pose.position,point);
    tf::Quaternion quat;
    quat.normalize();
    tf::quaternionMsgToTF(msg.pose.pose.orientation,quat);
    quat.normalize();
    this->current_pose.setOrigin(this->world2odom.getOrigin()+point);
    this->current_pose.setRotation(this->world2odom.getRotation()*quat);
    ROS_INFO("ODOM: %lf %lf %lf",this->current_pose.getOrigin().getX(),this->current_pose.getOrigin().getY(),this->current_pose.getOrigin().getZ());
    
    
}

void Controller::target_state_callback(geometry_msgs::PoseStamped msg)
{
    tf::poseMsgToTF(msg.pose,this->target_pose);
}

void Controller::current_state_callback(geometry_msgs::PoseStamped msg)
{
    tf::poseMsgToTF(msg.pose,this->current_pose);
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
    this->vel_out.publish(msg_vel);

    //publish output pose state
    geometry_msgs::PoseStamped msg_pose;    
    msg_pose.header.frame_id=this->world_frame;
    msg_pose.header.stamp=ros::Time::now();    
    tf::poseTFToMsg(this->current_pose,msg_pose.pose);
    this->state_out.publish(msg_pose);   

    //publish control difference
    geometry_msgs::Transform trafo;
    tf::transformTFToMsg(this->control_dif,trafo);
    this->control_difference.publish(trafo);
}

void Controller::calc_Lyapunov(double kx, double ky, double kphi,double vd,double omegad)
{
   
    tf::Pose relative;
    relative=this->current_pose.inverseTimes(this->target_pose);
    this->control_dif=relative;

    double x=relative.getOrigin().getX();
    double y=relative.getOrigin().getY();
    double phi=tf::getYaw(relative.getRotation());

    ROS_INFO("Controller: x: %lf  y: %lf  phi: %lf",x,y,phi);
    this->lin_vel_out.setX(kx*x+vd*cos(phi));
    this->ang_vel_out.setZ(kphi*sin(phi)+ky*vd*y+omegad);

 
}




void Controller::scope()
{
    switch(this->type)
    {
        case pseudo_inverse: break;
        case lypanov:this->calc_Lyapunov(this->kx, this->ky, this->kphi,this->vd,this->omegad);break;
        default: break;
    }
}

void Controller::execute()
{
    this->getTransformation();
    this->scope();
    this->publish();
    ros::spinOnce();   
}