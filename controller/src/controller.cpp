#include <controller/controller.h>

Controller::Controller(ros::NodeHandle &nh):    nh(nh),
                                                lin_vel_in{0,0,0},
                                                ang_vel_in{0,0,0}
{
    this->listener=new tf::TransformListener(nh);
    
    set_name("my_slave");
 
    this->vel_out=this->nh.advertise<geometry_msgs::Twist>("/out",10);
    this->state_out=this->nh.advertise<geometry_msgs::PoseStamped>("/state_out",10);
    this->control_difference=this->nh.advertise<geometry_msgs::Transform>("/control_difference",10);

    this->vel_in=this->nh.subscribe("/in",10,&Controller::input_velocities_callback,this);
    //this->odom=this->nh.subscribe("/odom",10,&Controller::input_odom_callback,this);
    this->state_in=this->nh.subscribe("/state_in",10,&Controller::input_state_callback,this);

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


void Controller::set_reference(double x,double y,double z)
{

    this->reference_pose=tf::Pose();
    this->reference_pose.setOrigin(tf::Vector3(x,y,z));
    this->reference_pose.setRotation(tf::Quaternion(0,0,0,1));

    this->current_pose=this->reference_pose;



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


}

void Controller::set_name(std::string name)
{
    this->name=name;
    this->nh.resolveName(name);

    this->link_output_velocity("mobile_base_controller/cmd_vel");
    this->link_output_state("state");
    this->link_control_difference("control_dif");
    this->link_input_state("state_in");

   
}

void Controller::set_type(Controller::controllerType type)
{
    int i;

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

   
    ROS_INFO("Loading %s ",PARAM_IN_ODOM);
    ros::param::get(PARAM_IN_ODOM,param);
    this->link_input_odom(param);
    
    // ros::param::get(PARAM_OUT_STATE,param);
    // ROS_INFO("Loading %s ",PARAM_OUT_STATE);
    // this->link_output_state(param);

    ros::param::get(PARAM_IN_VEL,param);
    ROS_INFO("Loading %s ",PARAM_IN_VEL);
    this->link_input_velocity(param);

    // ros::param::get(PARAM_OUT_VEL,param);
    // ROS_INFO("Loading %s ",PARAM_OUT_VEL);
    // this->link_output_velocity(param);

    // ros::param::get(PARAM_CONTROL_DIFF,param);
    // ROS_INFO("Loading %s ",PARAM_CONTROL_DIFF);
    // this->link_(param);

    int i;
    ros::param::get(PARAM_TYPE,i);
    ROS_INFO("Loading %s ",PARAM_TYPE);
    this->set_type(static_cast<Controller::controllerType>(i));

    double x;
    double y;
    ros::param::get(PARAM_X,x);
    ros::param::get(PARAM_Y,y);
    ROS_INFO("Loading coordinates %s : %lf , %lf ",this->name.c_str(),x,y);
    this->set_reference(x,y,0.0);


    // ros::param::get(PARAM_CONTROLLER_KX,this->kx);
    // ros::param::get(PARAM_CONTROLLER_KPHI,this->kphi);
    // ros::param::get(PARAM_CONTROLLER_OMEGAD,this->kx);
    // ros::param::get(PARAM_CONTROLLER_VD,this->kx);
    


    load_parameter();
}


void Controller::load_parameter()
{
    
}

//################################################################################################
//Linking important topics/transformations

//INPUTS
void Controller::link_input_velocity(std::string topic_name)
{
    this->vel_in.shutdown();
    ROS_INFO("Linking input velocity %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->vel_in=this->nh.subscribe(topic_name,10,&Controller::input_velocities_callback,this);
}

void Controller::link_input_odom(std::string topic_name)
{
    this->odom.shutdown();
    ROS_INFO("Linking input odom %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->odom=this->nh.subscribe(topic_name,10,&Controller::input_odom_callback,this);
}


void Controller::link_input_state(std::string topic_name)
{
    this->state_in.shutdown();
    ROS_INFO("Linking input state %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->state_in=this->nh.subscribe(topic_name,10,&Controller::input_state_callback,this);
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

void Controller::link_control_difference(std::string topic_name)
{
    this->control_difference.shutdown();
    ROS_INFO("Linking control difference %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->control_difference=this->nh.advertise<geometry_msgs::Transform>(topic_name,10);
}





//################################################################################################
//callback methods

void Controller::input_velocities_callback(geometry_msgs::Twist msg)
{
    tf::vector3MsgToTF(msg.linear,this->lin_vel_in);
    tf::vector3MsgToTF(msg.angular,this->ang_vel_in);
}

void Controller::input_odom_callback(nav_msgs::Odometry msg)
{
    tf::Point point;
    tf::pointMsgToTF(msg.pose.pose.position,point);
    tf::Quaternion quat;
    quat.normalize();
    tf::quaternionMsgToTF(msg.pose.pose.orientation,quat);
    this->current_pose.setOrigin(this->world2odom.getOrigin()+point);
    this->current_pose.setRotation(this->world2odom.getRotation()*quat);
    
}

void Controller::input_state_callback(nav_msgs::Odometry msg)
{
    tf::Point point;
    tf::pointMsgToTF(msg.pose.pose.position,point);
    tf::Quaternion quat;
    quat.normalize();  
    tf::quaternionMsgToTF(msg.pose.pose.orientation,quat);      
    this->target_pose.setOrigin(point);
    this->target_pose.setRotation(quat);   
}





//################################################################################################
//calculations and scope functions

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

void Controller::calc_Lyapunov(double kx, double kphi,double vd,double omegad)
{
    //calculate control differences
    tf::Transform world2target;
    world2target.setOrigin(this->target_pose.getOrigin());
    world2target.setRotation(this->target_pose.getRotation());

    this->control_dif.setOrigin(this->world2robot.inverse()*(target_pose.getOrigin()-current_pose.getOrigin()));
    this->control_dif.setRotation(this->world2robot.inverse()*(target_pose.getRotation()-current_pose.getRotation()));

    tf::Vector3 posr=this->control_dif.getOrigin();
    double rot=this->target_pose.getRotation().getAngle()-this->current_pose.getRotation().getAngle();
    ROS_INFO("dx: %lf dy: %lf",posr.x(),posr.y());
    ROS_INFO("Angle: %lf",rot);


    this->lin_vel_out.setX(kx*posr.getX()+vd*cos(rot));
    this->ang_vel_out.setZ(kphi*sin(rot)+vd*posr.getY()+omegad);
}




void Controller::scope()
{
    switch(this->type)
    {
        case pseudo_inverse: break;
        case lypanov:this->calc_Lyapunov(0.1,0.1,1.0,1.0);break;
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