#include <formation/odometry_predictor.h>


OdometryPredictor::OdometryPredictor(): topic_name_("odom"),
                                        nh_("odometry_predictor"),
                                        initial_trafo_(tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0.0,0.0,0.0)))
{
    this->odom_sub_=this->nh_.subscribe<nav_msgs::Odometry>(topic_name_,10,boost::bind(&OdometryPredictor::callbackOdometry,this,_1));
}


OdometryPredictor::OdometryPredictor(ros::NodeHandle &nh):  topic_name_("odometry/filtered"),
                                                            nh_(nh),
                                                            initial_trafo_(tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0.0,0.0,0.0)))
{
    this->odom_sub_=this->nh_.subscribe<nav_msgs::Odometry>(topic_name_,10,boost::bind(&OdometryPredictor::callbackOdometry,this,_1));
}


OdometryPredictor::OdometryPredictor(ros::NodeHandle &nh,std::string topic_name,tf::Transform initial_trafo):
                                                            nh_(nh),
                                                            topic_name_(topic_name),
                                                            initial_trafo_(initial_trafo)
{
    this->odom_sub_=this->nh_.subscribe<nav_msgs::Odometry>(topic_name_,10,boost::bind(&OdometryPredictor::callbackOdometry,this,_1));  
}



void OdometryPredictor::callbackOdometry(const nav_msgs::OdometryConstPtr msg)
{
    tf::poseMsgToTF(msg->pose.pose,this->pose_);
    pose_*=this->initial_trafo_;
}    

tf::Pose OdometryPredictor::getPose()
{
    return this->pose_;
}