#include <simulation_env/formation_publisher.h>

FormationPublisher::FormationPublisher()
{

}
FormationPublisher::FormationPublisher(ros::NodeHandle nh,std::string topic):   nh_(nh),
                                                                                topic_name_(topic)

{
    this->pub_form_=this->nh_.advertise<multi_robot_msgs::Formation>("formation",10);
}


void FormationPublisher::publish(Formation formation)
{
    if(formation.empty())
    {
        std::string str;
        throw std::invalid_argument("Cannot publish empty Formation!");
    }
    else
    {
        multi_robot_msgs::Formation msg;
        FormationPublisher::Formation2Msg(formation,msg);
        this->pub_form_.publish(msg);              
    }
}



void FormationPublisher::Formation2Msg(std::vector<tf::Pose>formation,multi_robot_msgs::Formation &msg)
{    
    for(int i=0;i<formation.size();i++)
    {
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(formation.at(i),pose);
        msg.formation.push_back(pose);
    }
}

void FormationPublisher::Formation2Msg(Formation formation,multi_robot_msgs::Formation &msg)
{    
    Formation2Msg(formation.getPoses(),msg);
    std::vector<std::vector<double> > adjacency=formation.getAdjacency();
    multi_robot_msgs::ArrayOfArrays matrix;
   
    for(int i=0;i<adjacency.size();i++)
    {
        multi_robot_msgs::Array row;
        for (int k=0;k<adjacency.at(i).size();k++)
        {
            row.array.push_back(adjacency.at(i).at(k));
        }
        matrix.array_of_arrays.push_back(row);
    }
    msg.adjacency=matrix;
}
