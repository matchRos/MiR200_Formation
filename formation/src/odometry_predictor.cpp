#include <formation/odometry_predictor.h>


OdometryPredictor::OdometryPredictor(): topic_name_("odom"),
                                        nh_("odometry_predictor"),
                                        initial_trafo_(tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0.0,0.0,0.0)))
{
    ROS_INFO("Odometry predictor: Subscribing to: %s",this->nh_.resolveName(topic_name_).c_str());
    this->odom_sub_=this->nh_.subscribe<nav_msgs::Odometry>(topic_name_,1,boost::bind(&OdometryPredictor::callbackOdometry,this,_1));
}


OdometryPredictor::OdometryPredictor(ros::NodeHandle &nh):  topic_name_("odometry/filtered"),
                                                            nh_(nh),
                                                            initial_trafo_(tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0.0,0.0,0.0)))
{
    ROS_INFO("Odometry predictor: Subscribing to: %s",this->nh_.resolveName(topic_name_).c_str());
    this->odom_sub_=this->nh_.subscribe<nav_msgs::Odometry>(topic_name_,1,boost::bind(&OdometryPredictor::callbackOdometry,this,_1));
}


OdometryPredictor::OdometryPredictor(ros::NodeHandle &nh,std::string topic_name,tf::Transform initial_trafo):
                                                            nh_(nh),
                                                            topic_name_(topic_name),
                                                            initial_trafo_(initial_trafo)
{
    ROS_INFO("Odemetry predictor: Initial Transformation: x:%lf y:%lf z:%lf ; qx: %lf qy:%lf qz:%lf qw:%lf",
                this->initial_trafo_.getOrigin().x(),
                this->initial_trafo_.getOrigin().y(),
                this->initial_trafo_.getOrigin().z(),
                this->initial_trafo_.getRotation().x(),
                this->initial_trafo_.getRotation().y(),
                this->initial_trafo_.getRotation().z(),
                this->initial_trafo_.getRotation().w());
    ROS_INFO("Odometry predictor: Subscribing to: %s",this->nh_.resolveName(topic_name_).c_str());
    this->odom_sub_=this->nh_.subscribe<nav_msgs::Odometry>(topic_name_,1,boost::bind(&OdometryPredictor::callbackOdometry,this,_1));  
}



void OdometryPredictor::callbackOdometry(const nav_msgs::OdometryConstPtr msg)
{
    tf::poseMsgToTF(msg->pose.pose,this->pose_);
    pose_=this->initial_trafo_*pose_;
}    

tf::Pose OdometryPredictor::getPose()
{
    return this->pose_;
}