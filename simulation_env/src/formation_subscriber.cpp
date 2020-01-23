#include <simulation_env/formation_subscriber.h>

FormationSubscriber::FormationSubscriber(ros::NodeHandle &nh,Formation* formation,std::vector<std::string> topics):formation_(formation)
{
    for(int i=0; i<this->formation_->size();i++)
    {   
        std::string str;
        if(topics.size()>0)
        {
            str=this->formation_->getName(i)+"/"+topics.at(0);
            ROS_INFO("Subscribing to: %s",nh.resolveName(str).c_str());
            this->odom_subscribers_.push_back(nh.subscribe<nav_msgs::Odometry>(str,10,boost::bind(&FormationSubscriber::callback_odometry,this,_1,i)));
        }
    }
}

void FormationSubscriber::callback_odometry(const nav_msgs::OdometryConstPtr& msg,int number)
{   
    tf::Pose pose;
    tf::StampedTransform trafo;
    tf::poseMsgToTF(msg->pose.pose,pose);
    try{
        this->listener_.lookupTransform(this->formation_->getReferenceFrame(),msg->header.frame_id,ros::Time(0),trafo);
        pose=trafo*pose;
    }
    catch(ros::Exception &e)
    {
        ROS_WARN("%s",e.what());
    }
    this->formation_->modifiePose(number,pose);
}

