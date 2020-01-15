#include <simulation_env/formation.h>


Formation::Formation()
{
    adjacency_calculation_=Formation::calc_by_pose;
}

void Formation::addRobot(tf::Pose pose,std::string name,std::vector<int> neighbours)
{
    if(neighbours.empty())
    {
        this->formation_.push_back(pose);
        this->names_.push_back(name);
        this->scan_data_.push_back( sensor_msgs::PointCloud());
    }
    else
    {
        this->formation_.push_back(pose);
        this->names_.push_back(name);
        this->scan_data_.push_back( sensor_msgs::PointCloud());
        determineConnectivity(neighbours);
        this->adjacency_.resize(connectivity_.size(), std::vector<double>(connectivity_.size(),0.0));        
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
void Formation::modifieLaserdata(int i, sensor_msgs::PointCloud datum)
{
    if(i<this->size())
    {   
        this->scan_data_.at(i)=datum;
    }
    else
    {
        throw std::invalid_argument("Laser data to modifie does not belong to this formation!");
    } 
}
std::vector<std::vector<double> > Formation::getAdjacency()
{
    switch(this->adjacency_calculation_)
    {
        case Formation::calc_by_pose:  return this->calcAdjacencyFromPoses(this->formation_);
        case Formation::calc_by_scan:  return this->calcAdjacencyFromScan(this->scan_data_);
    }
}

std::vector<std::vector<double> > Formation::calcAdjacencyFromScan(std::vector<sensor_msgs::PointCloud>)
{
   
}
std::vector<std::vector<double> > Formation::calcAdjacencyFromPoses(std::vector<tf::Pose>)
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

std::vector< sensor_msgs::PointCloud> Formation::getScan()
{
    return this->scan_data_;
}

sensor_msgs::PointCloud Formation::getScan(int i)
{
    if(i<this->scan_data_.size())
    {
        return this->scan_data_.at(i);
    }
    else
    {
        std::stringstream ss;
        ss<<"Index "<<i<<" out of range "<<this->scan_data_.size()<<" of formation!";
        throw std::invalid_argument(ss.str());
    }
    return  sensor_msgs::PointCloud();   
}

 sensor_msgs::PointCloud Formation::getCombinedScan()
{
    return this->combineScanData(this->scan_data_);
}


sensor_msgs::PointCloud Formation::combineScanData(std::vector< sensor_msgs::PointCloud> data)
{
    sensor_msgs::PointCloud result;
    result.header.frame_id=this->refrence_frame;
    result.header.stamp=data.at(0).header.stamp;
    for(int i=0;i<data.size();i++)
    {
        for(int k=0;k<data.at(i).points.size();k++)
        {
            result.points.push_back(data.at(i).points.at(k));
        }
    }
    return result;
}
std::vector<sensor_msgs::PointCloud> Formation::cluster(sensor_msgs::PointCloud data,std::vector<tf::Point> &centers)
{
    std::vector<std::vector<tf::Point> >clusters;
    clusters.resize(centers.size());   

    tf::Vector3 point;
    tf::Vector3 center;
    int best_cluster;
    double best_distance;
    double distance;
    bool convergent=false;
    int iterations=0;
    std::vector<tf::Point> old_centers=centers;

    while(!convergent && iterations<150)
    {
        for(int i=0;i<data.points.size();i++)
        {
            best_cluster=0;
            point=tf::Vector3(data.points.at(i).x,data.points.at(i).y,data.points.at(i).z);
            center=centers.at(0);
            best_distance=(point-center).length();
            for(int k=1;k<centers.size();k++)
            {
                center=centers.at(k);
                distance=(point-center).length();;
                if(distance<best_distance)
                {
                    best_cluster=k;
                    best_distance=distance;
                }
            }
            clusters.at(best_cluster).push_back(point);
        }

        
        for(int j=0;j<centers.size();j++)
        {
            centers.at(j)=std::accumulate(clusters.at(j).begin(),clusters.at(j).end(),tf::Point(0,0,0))/clusters.at(j).size();
            if(centers.at(j)==old_centers.at(j))
            {
                if(j==0)
                {
                    convergent=true;
                }
                else
                {
                    convergent*=true;
                }                
            }
        }  
        iterations++;   
    }
    std::vector<sensor_msgs::PointCloud> result;
    sensor_msgs::ChannelFloat32 channel;
    try
    {      
        result.resize(clusters.size());
        geometry_msgs::Point32 point_msg;        
        for(int i=0;i<clusters.size();i++)
        {
            sensor_msgs::ChannelFloat32 channel;
            result.at(i).header=data.header;
            for(int k=0;k<clusters.at(i).size();k++)
            {            
                point_msg.x=clusters.at(i).at(k).x();
                point_msg.y=clusters.at(i).at(k).y();
                point_msg.z=clusters.at(i).at(k).z();
                result.at(i).points.push_back(point_msg);
                channel.name="intensity";
                channel.values.push_back(1.0/clusters.size()*(i+1)) ;         
            }
            result.at(i).channels.push_back(channel);
        }
    }
    catch(std::exception &e)
    {
        ROS_WARN("In Clustering: %s",e.what());
    }
    
    return result;
}

std::vector<sensor_msgs::PointCloud> Formation::getClusteredData()
{
    std::vector<tf::Point> centers;
    centers.push_back(tf::Point(-1.5,-1.5,0));
    centers.push_back(tf::Point(-1.5,1.5,0));
    centers.push_back(tf::Point(1.5,1.5,0));
    centers.push_back(tf::Point(1.5,-1.5,0));
    return this->cluster(this->getCombinedScan(),centers);
}
//##############################################################################################################################################################
//##############################################################################################################################################################
FormationPublisher::FormationPublisher()
{

}
FormationPublisher::FormationPublisher(ros::NodeHandle nh,std::string topic):   nh_(nh),
                                                                                topic_name_(topic)

{
    this->pub_form_=this->nh_.advertise<multi_robot_msgs::Formation>("formation",10);
    this->pub_scan_=this->nh_.advertise<sensor_msgs::PointCloud>("scan",10);
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
        sensor_msgs::PointCloud cloud;
        std::vector<sensor_msgs::PointCloud> clouds=formation.getClusteredData();
        for(int i=0;i<clouds.size();i++)
        {
            this->pub_scan_.publish(clouds.at(i)); 
        }
              
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



//##############################################################################################################################################################
//##############################################################################################################################################################

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
        if(topics.size()>1)
        {  
            str=this->formation_->getName(i)+"/"+topics.at(1);
            ROS_INFO("Subscribing to: %s",nh.resolveName(str).c_str());
            this->laser_subscribers_.push_back(nh.subscribe<sensor_msgs::LaserScan>(str,10,boost::bind(&FormationSubscriber::callback_laserscanner,this,_1,i)));
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

void FormationSubscriber::callback_laserscanner(const sensor_msgs::LaserScanConstPtr &msg, int number)
{
    sensor_msgs::PointCloud cloud;
    
    cloud.header.frame_id=msg->header.frame_id;
    try{
        //this->projector_.transformLaserScanToPointCloud(this->formation_->getReferenceFrame(),*msg, cloud,this->listener_);        
        this->projector_.projectLaser(*msg, cloud);       
        this->formation_->modifieLaserdata(number,cloud);
        // ROS_INFO("Received at %i  %i points and transformed to %i",number,msg->ranges.size(),cloud.points.size());
    }
    catch(std::exception &e)
    {
        e.what();
    }    
}



