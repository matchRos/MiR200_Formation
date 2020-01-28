#include <formation/formation_subscriber.h>

FormationSubscriber::FormationSubscriber(ros::NodeHandle &nh,Formation* formation,std::vector<std::string> topics):formation_(formation)
{

}

void FormationSubscriber::callback_odometry(const nav_msgs::OdometryConstPtr& msg,int number)
{   

}

