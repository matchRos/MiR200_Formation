#include <controller/controller.h>

Controller::Controller(ros::NodeHandle &nh):nh(nh)
{

    set_name("my_slave");
 
    this->output=this->nh.advertise<geometry_msgs::Twist>("out",10);

    this->input=this->nh.subscribe("in",10,&Controller::input_velocities_callback,this);
    this->odom=this->nh.subscribe("mobile_base_controller/odom",10,&Controller::input_odom_callback,this);

    this->world_frame="/map";
   
} 

void Controller::set_reference(double x,double y,double z)
{
    this->reference.position.x;
    this->reference.position.y;
    this->reference.position.z;

    tf2::Quaternion quat;
    quat.setRPY(0,0,0);

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id ="/map" ;
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

//################################################################################################
//Linking important topics/transformations

void Controller::link_input_velocity(std::string topic_name)
{
    this->input.shutdown();
    ROS_INFO("Linking input %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->input=this->nh.subscribe(topic_name,10,&Controller::input_velocities_callback,this);
}

void Controller::link_input_odom(std::string topic_name)
{
    this->odom.shutdown();
    ROS_INFO("Linking odom %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->odom=this->nh.subscribe(topic_name,10,&Controller::input_odom_callback,this);
}

void Controller::link_output_velocity(std::string topic_name)
{
    this->output.shutdown();
    ROS_INFO("Linking output %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->output=this->nh.advertise<geometry_msgs::Twist>(topic_name,10);
}




//################################################################################################
//callback methods

void Controller::input_velocities_callback(geometry_msgs::Twist msg)
{
    this->msg_velocities_in=msg;
}

void Controller::input_odom_callback(nav_msgs::Odometry msg)
{
    tf::TransformListener listener;
    tf::StampedTransform trafo;
    listener.lookupTransform(this->name+"/odom_comb","/map",ros::Time(0),trafo);
    this->current_pose.setOrigin(trafo.getOrigin());
    this->current_pose.setRotation(trafo.getRotation());
}


void Controller::scope()
{
    //publish
    geometry_msgs::Twist msg;  

    this->output.publish(this->msg_velocities_out);

    ros::spinOnce();
}