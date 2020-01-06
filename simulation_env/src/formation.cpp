#include <simulation_env/formation.h>


Formation::Formation()
{

}

void Formation::addRobot(tf::Pose pose)
{
    this->formation_.push_back(pose);
}

int Formation::size()
{
    return this->formation_.size();
}


std::vector<tf::Pose> Formation::getPoses()
{
    return this->formation_;   
}

std::vector<tf::Transform> operator-(Formation &c1,Formation &c2)
{
    std::vector<tf::Pose> poses_l=c1.getPoses();
    std::vector<tf::Pose> poses_r=c2.getPoses();   
    std::vector<tf::Transform> trafos; 
    for(int i=0;i<poses_l.size();i++)
    {
        trafos.push_back(poses_l.at(i).inverseTimes(poses_r.at(i)));
    }
    return trafos;
}

static void Formation2Msg(std::vector<tf::Pose>formation,multi_robot_msgs::Formation &msg)
{    
    for(int i=0;i<formation.size();i++)
    {
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(formation.at(i),pose);
        msg.formation.push_back(pose);
    }
}


FormationPublisher::FormationPublisher(ros::NodeHandle nh,std::string topic):   nh_(nh),
                                                                                topic_name_(topic)

{
    this->publisher_=this->nh_.advertise<multi_robot_msgs::Formation>(this->topic_name_,10);
}