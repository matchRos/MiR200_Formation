#include <simulation_env/formation.h>


Formation::Formation()
{

}

void Formation::addRobot(tf::Pose pose,std::string name,std::vector<int> neighbours)
{
    if(neighbours.empty())
    {
        this->formation_.push_back(pose);
        this->names_.push_back(name);
    }
    else
    {
        this->formation_.push_back(pose);
        this->names_.push_back(name);
        int max=*std::max_element(neighbours.begin(),neighbours.end())+1;
        if(formation_.size()>max)
        {
            max=formation_.size();
        }       
        this->adjacency_.resize(max, std::vector<double>(max,0.0));
        this->connectivity_.resize(max, std::vector<bool>(max,false));
        for(int i=0;i<neighbours.size();i++)
        {
            int k=neighbours.at(i);
            this->connectivity_.at(i).at(k)=true;
            this->connectivity_.at(k).at(i)=true;
        }
    }
}
Formation Formation::transform(Formation formation,tf::Transform trafo)
{
    for(int i=0;i<formation.size();i++)
    {
        formation.formation_.at(i)=trafo*formation.formation_.at(i);
    }
    return formation;
}
int Formation::size()
{
    return this->formation_.size();
}


std::vector<tf::Pose> Formation::getPoses()
{
    return this->formation_;   
}


bool Formation::empty()
{
    return this->formation_.size()==0;
}

void Formation::modifiePose(int i,tf::Pose pose)
{
    if(i<formation_.size())
    {   
        this->formation_.at(i)=pose;
    }
    else
    {
        throw std::invalid_argument("Pose to modified does not belong to this formation!");
    }
    
    
}
std::vector<std::vector<double> > Formation::getAdjacency()
{
    std::vector<std::vector<double> > adjacency;
    adjacency.resize(this->connectivity_.size(),std::vector<double>(this->connectivity_.size(),0.0));
    
    for(int i=0;i<adjacency.size();i++)
    {
        for(int k=0;k<adjacency.at(i).size();k++)
        {
            if(i>=this->formation_.size() || k>=this->formation_.size())
            {
                std::stringstream ss;
                ss<<"Adjacecny matrix wants to connect "<<i<<"with "<<k<<" while formation size is just "<<formation_.size();
                throw std::invalid_argument(ss.str());
            }
            else
            {
                 tf::Vector3 distance=this->formation_[i].getOrigin()-this->formation_[k].getOrigin();
                 adjacency.at(i).at(k)=distance.length();
            }           
        }
    }
    return adjacency;
}

Formation::Transformation Formation::operator-(Formation &target)
{
    Formation res;
    if(this->size()!=target.size())
    {
        throw std::invalid_argument("Dimentions of formation dont fetch to each other!");
    }
    else
    {
        if(!adjacency_.empty())
        {
            res.adjacency_.resize(target.size(),std::vector<double>(target.size(),0.0));
            for(int i=0;i<target.adjacency_.size();i++)
            {
                for(int k=0;k<target.adjacency_.at(i).size();k++)
                {
                    res.adjacency_.at(i).at(k)=target.adjacency_.at(i).at(k)-this->adjacency_.at(i).at(k);
                }            
            }
        }

        res.formation_.resize(target.formation_.size());
        for(int i=0;i<target.formation_.size();i++)
        {
            res.formation_.at(i)=this->formation_.at(i).inverseTimes(target.formation_.at(i));
        }
    }    
    return res;
}

std::string Formation::getName(int i)
{
    if(i>this->names_.size())
    {
        throw std::invalid_argument(std::string("Index %i is out of Formation range",i));
    }
    return this->names_[i];
}
void Formation::setReferenceFrame(std::string frame_name)
{
    this->refrence_frame=frame_name;
}
std::string Formation::getReferenceFrame()
{
    return this->refrence_frame;
}

//##############################################################################################################################################################

//##############################################################################################################################################################
FormationPublisher::FormationPublisher()
{

}
FormationPublisher::FormationPublisher(ros::NodeHandle nh,std::string topic):   nh_(nh),
                                                                                topic_name_(topic)

{
    this->publisher_=this->nh_.advertise<multi_robot_msgs::Formation>(this->topic_name_,10);
}


void FormationPublisher::publish(Formation formation)
{
    if(formation.empty())
    {
        std::string str;
        throw std::invalid_argument("Cannot convert empty Formation to message type");
    }
    else
    {
        multi_robot_msgs::Formation msg;
        FormationPublisher::Formation2Msg(formation,msg);
        this->publisher_.publish(msg);
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





FormationSubscriber::FormationSubscriber(ros::NodeHandle &nh,Formation* formation,std::string topic):formation_(formation)
{
    for(int i=0; i<this->formation_->size();i++)
    {        
        std::string str=this->formation_->getName(i)+"/"+topic;
        ROS_INFO("Subscribing to: %s",nh.resolveName(str).c_str());
        this->odom_subscribers_.push_back(nh.subscribe<nav_msgs::Odometry>(str,10,boost::bind(&FormationSubscriber::callback_subscription,this,_1,i)));
    }
}

void FormationSubscriber::callback_subscription(const nav_msgs::OdometryConstPtr& msg,int number)
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
