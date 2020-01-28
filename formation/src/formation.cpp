#include <formation/formation.h>


Formation::Formation(): number_of_robots_(0),
                        reference_frame_("map")

{

}

void Formation::addRobot(Formation::RobotProperties robot_properties)
{
    if( robot_properties.name==""               ||
        robot_properties.laser_frames.back==""  ||
        robot_properties.laser_frames.base==""  ||
        robot_properties.laser_frames.front=="" ||
        robot_properties.laser_topics.back==""  ||
        robot_properties.laser_topics.front=="" ||
        robot_properties.neighbours.empty()       )
    { 
        throw std::invalid_argument("Robot to add to formation contains errors! See if everything is initalised properly!");
    }
    
    Robot robot;
    robot.pose=robot_properties.pose;
    ros::NodeHandle nh(robot_properties.name);
    robot.neighbours=robot_properties.neighbours;
    robot.laser=LaserPointer(new LaserPredictor(nh,robot_properties.laser_frames,robot_properties.laser_topics));
    robot.odom=OdomPointer(new OdometryPredictor(nh,robot_properties.odom_topic,robot_properties.pose));
    this->formation_map_.insert(std::pair<std::string,Formation::Robot>(robot_properties.name,robot));
    this->index_map_.insert(std::pair<std::string,unsigned int>(robot_properties.name,this->number_of_robots_));
    this->number_of_robots_++;
}

int Formation::size()
{
    return this->number_of_robots_;
}

bool Formation::empty()
{
    return this->size()==0;
}

std::string Formation::getReferenceFrame()
{
    return this->reference_frame_;
}

void Formation::modifiePose(std::string name,tf::Pose pose)
{
    try
    {
        this->formation_map_.at(name).pose=pose;
    }
    catch(std::out_of_range &e)
    {
        std::stringstream ss;
        ss<<"Formation: "<<e.what();
        throw std::out_of_range(ss.str());
    }   
}


Formation::Poses Formation::getPose()
{
    if(this->formation_map_.empty())
    {
        throw std::out_of_range("No Poses in Formation since the formation is empty!");
    }

    Poses poses;
    for(auto robot:this->formation_map_)
    {
        poses.push_back(robot.second.pose);
    }
    return poses;
}

tf::Pose Formation::getPose(std::string name)
{
    return this->formation_map_.at(name).pose;
}

Formation::Poses Formation::getScannedPose()
{
    bool first=true;
    Poses poses;
    Robot start=this->formation_map_.begin()->second;
    poses.push_back(start.pose);
    for(auto robot:this->formation_map_)
    {
        if(first)
        {
            first=false;
        }
        else
        {
            poses.push_back(poses.at(0)*this->transformBetweenRobots(start,robot.second));
            poses.at(poses.size()-1).setRotation(tf::createIdentityQuaternion());   
        }    
    }
    return poses;
}



Formation::Poses Formation::getScannedPose(std::string name)
{
    return this->formation_map_.at(name).laser->getPoses();
}


Formation::Matrix<double> Formation::getAdjacency()
{  
    return  this->determineAdjacency();
}

Formation::Matrix<bool> Formation::getConnectivity()
{
    return this->determineConnectivity();
}


Formation::Cloud Formation::getClusteredScan()
{
    if(this->empty())
    {
        throw std::out_of_range("Cannot get scanner data from an empty formation!");
    }
    Formation::Cloud cloud;
    for(auto robot: formation_map_)
    {
        Formation::Cloud local;
        local=robot.second.laser->getClusteredPoints();        
        local.header.frame_id=this->reference_frame_;
        LaserPredictor::transformCloud(local,robot.second.pose);
        cloud=LaserPredictor::combineData(local,cloud);
    }
    return cloud;
}

Formation::Cloud Formation::getClusteredScan(std::string name)
{
    return this->formation_map_.at(name).laser->getClusteredPoints();
}

Formation::Cloud Formation::getScan()
{
    if(this->empty())
    {
        throw std::out_of_range("Cannot get scanner data from an empty formation!");
    }   
    Formation::Cloud cloud;
    for(auto robot: formation_map_)
    {
        Formation::Cloud local;
        local=robot.second.laser->getRegisteredPoints();        
        local.header.frame_id=this->reference_frame_;
        LaserPredictor::transformCloud(local,robot.second.pose);
        cloud=LaserPredictor::combineData(local,cloud);
    }
    return cloud;
       
}

Formation::Cloud Formation::getScan(std::string name)
{
    return this->formation_map_.at(name).laser->getRegisteredPoints();
}

std::vector<std::string> Formation::getNames()
{
    std::vector<std::string> names;
    for(auto robot: this->formation_map_)
    {
        names.push_back(robot.first);
    }
    return names;
}


void Formation::startPrediction(double frequenzy)
{    
    for(auto robot : this->formation_map_)
    {
        Poses relative_poses;
        for(auto neighbour: robot.second.neighbours)
        {
            relative_poses.push_back(robot.second.pose.inverseTimes(this->formation_map_.at(neighbour).pose));
            ROS_WARN("Rel psoes: %lf %lf %lf ",relative_poses.back().getOrigin().x(),relative_poses.back().getOrigin().y(),relative_poses.back().getOrigin().z());
        }
        robot.second.laser->guess(relative_poses);
        robot.second.laser->startClustering(frequenzy);
    }
}



Formation::Matrix<bool> Formation::determineConnectivity()
{
    Matrix<bool> connectivity;
    connectivity.resize(this->number_of_robots_,std::vector<bool>(this->number_of_robots_,false));

    for(auto robot : this->formation_map_)
    {
        Poses relative_poses;
        for(auto neighbour: robot.second.neighbours)
        {
           connectivity.at(index_map_.at(robot.first)).at(index_map_.at(neighbour))=true;
        }
    }
    return connectivity;
}


Formation::Matrix<double> Formation::determineAdjacency()
{ 
    Matrix<double> adjacency;
    adjacency.resize(this->connectivity_.size(),std::vector<double>(this->connectivity_.size(),0.0));
    for(auto robot : formation_map_)
    {
        for(auto name: robot.second.neighbours)
        {
            tf::Pose pose=this->formation_map_.at(name).pose;
            double diff=robot.second.pose.inverseTimes(pose).getOrigin().length();
            unsigned int row=index_map_.at(robot.first);
            unsigned int col=index_map_.at(name);
            adjacency.at(row).at(col)=diff;            
        }
    }
    return adjacency;
}

tf::Transform Formation::transformBetweenRobots(Formation::Robot robot1, Formation::Robot robot2)
{
    Poses poses1;
    Poses poses2;
    poses1=robot1.laser->getPoses();
    poses2=robot2.laser->getPoses();

    for(auto pose1:poses1)
    {
        for(auto pose2:poses2)
        {
            if((pose1.getOrigin()-pose2.getOrigin()).length()<1.0)
            {
                return pose1;
            }
        }
    }
    return tf::Transform();
}