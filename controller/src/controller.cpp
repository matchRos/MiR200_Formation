#include <controller/controller.h>

Controller::Controller(ros::NodeHandle &nh):nh(nh)
{

    set_name("my_slave");
 
    this->output=this->nh.advertise<geometry_msgs::Twist>("/out",10);
    this->state_out=this->nh.advertise<geometry_msgs::PoseStamped>("/state_out",10);

    this->input=this->nh.subscribe("/in",10,&Controller::input_velocities_callback,this);
    this->odom=this->nh.subscribe("/odom",10,&Controller::input_odom_callback,this);
    this->state_in=this->nh.subscribe("/state_in",10,&Controller::input_state_callback,this);

    this->current_pose=tf::Pose();
} 
//################################################################################################
//Setter


void Controller::set_reference(double x,double y,double z)
{
    this->reference.position.x=x;
    this->reference.position.y=y;
    this->reference.position.z=z;

    this->current_pose=tf::Pose();
    this->current_pose.setOrigin(tf::Vector3(x,y,z));
     

    tf2::Quaternion quat;
    quat.setRPY(0,0,0);
   
    
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id =this->world_frame ;
    static_transformStamped.child_frame_id = this->name+"/odom_comb";
    static_transformStamped.transform.translation.x=x;
    static_transformStamped.transform.translation.y=y;
    static_transformStamped.transform.translation.z=z;
    static_transformStamped.transform.rotation.x=quat.getX();
    static_transformStamped.transform.rotation.y=quat.getY();
    static_transformStamped.transform.rotation.z=quat.getZ();
    static_transformStamped.transform.rotation.w=quat.getW();
    static_broadcaster.sendTransform(static_transformStamped);




}

void Controller::set_name(std::string name)
{
    this->name=name;
    this->nh.resolveName(name);
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
    
    ros::param::get(PARAM_OUT_STATE,param);
    ROS_INFO("Loading %s ",PARAM_OUT_STATE);
    this->link_output_state(param);

    ros::param::get(PARAM_IN_VEL,param);
    ROS_INFO("Loading %s ",PARAM_IN_VEL);
    this->link_input_velocity(param);

    ros::param::get(PARAM_OUT_VEL,param);
    ROS_INFO("Loading %s ",PARAM_OUT_VEL);
    this->link_output_velocity(param);

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




    load_parameter();
}


void Controller::load_parameter()
{
    
}

//################################################################################################
//Linking important topics/transformations

void Controller::link_input_velocity(std::string topic_name)
{
    this->input.shutdown();
    ROS_INFO("Linking input velocity %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->input=this->nh.subscribe(topic_name,10,&Controller::input_velocities_callback,this);
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

void Controller::link_output_velocity(std::string topic_name)
{
    this->output.shutdown();
    ROS_INFO("Linking output velocity %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->output=this->nh.advertise<geometry_msgs::Twist>(topic_name,10);
}


void Controller::link_output_state(std::string topic_name)
{
    this->state_out.shutdown();
    ROS_INFO("Linking output state %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->state_out=this->nh.advertise<geometry_msgs::PoseStamped>(topic_name,10);
}






//################################################################################################
//callback methods

void Controller::input_velocities_callback(geometry_msgs::Twist msg)
{
    this->msg_velocities_in=msg;
}

void Controller::input_odom_callback(nav_msgs::Odometry msg)
{
    tf::Point point;
    tf::pointMsgToTF(msg.pose.pose.position,point);
    tf::Quaternion quat;
    quat.normalize();
    tf::quaternionMsgToTF(msg.pose.pose.orientation,quat);
    this->current_pose.setOrigin(this->robot2world.getOrigin()+point);
    this->current_pose.setRotation(this->robot2world.getRotation()*quat);
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
    tf::TransformListener listener;      
    try{
        listener.waitForTransform(this->world_frame,this->name+"/odom_comb",ros::Time(0),ros::Duration(0.05));
        listener.lookupTransform(this->world_frame,this->name+"/odom_comb", 
                               ros::Time(0), this->robot2world);
        
    }
    catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
    }
}

void Controller::publish()
{
    //publish 
    this->output.publish(this->msg_velocities_out);
    
    geometry_msgs::PoseStamped msg; 
   
    tf::Quaternion quat;
    quat=this->current_pose.getRotation();
    quat.normalize();
    tf::quaternionTFToMsg(quat,msg.pose.orientation);
    tf::pointTFToMsg(this->current_pose.getOrigin(), msg.pose.position);

    
    msg.header.frame_id=this->world_frame;
    msg.header.stamp=ros::Time::now();
   
    this->state_out.publish(msg);   

}

void Controller::calc_Lyapunov(double kx, double kphi,double vd,double omegad)
{
    tf::Vector3 pos_temp;   //Control differences in postion
    pos_temp=this->target_pose.getOrigin()-this->current_pose.getOrigin();
    tf::Quaternion quatr;   //Control difference in orientation;
    quatr=this->target_pose.getRotation()-this->current_pose.getRotation();
    tf::Vector3 posr;
    double phi =this->current_pose.getRotation().getAngle();
    posr.setX(pos_temp.getX()*cos(phi)+pos_temp.getX()*sin(phi));
    posr.setY(-pos_temp.getY()*sin(phi)+pos_temp.getY()*cos(phi));

    
    
    
    this->msg_velocities_out.linear.x=kx*posr.getX()+vd*cos(quatr.getAngle());
    this->msg_velocities_out.angular.z=kphi*sin(quatr.getAngle()+vd*posr.getY())+omegad;
}




void Controller::scope()
{
    switch(this->type)
    {
        case pseudo_inverse: break;
        case lypanov:this->calc_Lyapunov(0.7,0.7,0.3,0.3);break;
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