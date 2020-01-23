#include <simulation_env/formation.h>


Formation::Formation()
{

}

Formation::Formation(ros::NodeHandle &nh):nh_(nh)
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
        determineConnectivity(neighbours);
        this->adjacency_.resize(connectivity_.size(), std::vector<double>(connectivity_.size(),0.0));        
    }
}

void Formation::addRobot(Formation::RobotProperties robot_properties)
{
    if( robot_properties.name!="" &&
        robot_properties.laser_frames.back!="" &&
        robot_properties.laser_frames.base!="" &&
        robot_properties.laser_frames.front!=""&&
        robot_properties.laser_topics.back!="" &&
        robot_properties.laser_topics.front!=""&&
        !robot_properties.neighbours.empty()       )
    {
        Robot robot;
        robot.pose=robot_properties.pose;
        robot.predictor=LaserPointer(new LaserPredictor(this->nh_,robot_properties.laser_frames,robot_properties.laser_topics));
        robot.predictor->startClustering(10);

    }
    else
    {
        throw std::invalid_argument("Robot to add to formation contains errors! See if everything is initalised properly!");
    }
    
}


int Formation::size()
{
    return this->formation_.size();
}
bool Formation::empty()
{
    return this->size()==0;
}

std::vector<tf::Pose> Formation::getPoses()
{
    return this->formation_;   
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





Formation::Matrix<double> Formation::getAdjacency()
{
    this->determineAdjacency();
    return this->adjacency_;
}


LaserPredictor::Cloud Formation::getScannerPrediction(std::string name)
{
    this->formation_map_[name].predictor->getClusteredPoints();
}



void Formation::determineAdjacency()
{ 
    this->adjacency_.resize(this->connectivity_.size(),std::vector<double>(this->connectivity_.size(),0.0));
    
    for(int i=0;i<this->adjacency_.size();i++)
    {
        for(int k=0;k<this->adjacency_.at(i).size();k++)
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
                 this->adjacency_.at(i).at(k)=distance.length();
            }           
        }
    }
}

void Formation::determineConnectivity(std::vector<int> neighbours)
{
    int max=*std::max_element(neighbours.begin(),neighbours.end())+1;
    if(formation_.size()>max)
    {
        max=formation_.size();
    }      
    this->connectivity_.resize(max, std::vector<bool>(max,false));
    for(int i=0;i<neighbours.size();i++)
    {
        int k=neighbours.at(i);
        this->connectivity_.at(i).at(k)=true;
        this->connectivity_.at(k).at(i)=true;
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