#include <controller/laser_predictor.h>

LaserPredictor::LaserPredictor()
{
    
}

LaserPredictor::LaserPredictor( ros::NodeHandle &nh,
                                Frames frames,
                                Topics topics):
                                nh_(nh),
                                frames_(frames)
{
    ROS_INFO("Laser Predictor: Subscribing to:%s", this->nh_.resolveName(topics.front).c_str());
    this->sub_front_=this->nh_.subscribe<sensor_msgs::LaserScan>(this->nh_.resolveName(topics.front),100,boost::bind(&LaserPredictor::subscriberFrontCallback,this,_1));
    ROS_INFO("Laser Predictor: Subscribing to:%s", this->nh_.resolveName(topics.back).c_str());
    this->sub_back_=this->nh_.subscribe<sensor_msgs::LaserScan>(this->nh_.resolveName(topics.back),100,boost::bind(&LaserPredictor::subscriberBackCallback,this,_1));

    tf::TransformListener listener;
    ROS_INFO(   "Laser Predicitor: Get transformation from %s to %s",
                this->nh_.resolveName(frames.base).c_str(),
                this->nh_.resolveName(frames.back).c_str());
    if(!listener.waitForTransform(  this->nh_.resolveName(frames.base),
                                    this->nh_.resolveName(frames.back),
                                    ros::Time::now(),
                                    ros::Duration(10.0)))       
                              
    {     
        throw tf::TransformException("Wait for transformation for laser scannner back data timed out!");
    }
    else
    {
        listener.lookupTransform(   this->nh_.resolveName(frames.base),
                                    this->nh_.resolveName(frames.back),
                                    ros::Time::now(),
                                    this->trafos_.back);
    }


    ROS_INFO(   "Laser Predicitor: Get transformation from %s to %s",
                this->nh_.resolveName(frames.base).c_str(),
                this->nh_.resolveName(frames.front).c_str());
    if(!listener.waitForTransform(  this->nh_.resolveName(frames.base),
                                    this->nh_.resolveName(frames.front),
                                    ros::Time::now(),
                                    ros::Duration(10.0)))       
                              
    {    
        throw tf::TransformException("Wait for transformation for laser scannner front data timed out!");
    }
    else
    {
        listener.lookupTransform(   this->nh_.resolveName(frames.base),
                                    this->nh_.resolveName(frames.front),
                                    ros::Time::now(),
                                    this->trafos_.front);
    }
}


//Getter/Setter##############################################################################################################################
// ##########################################################################################################################################
// ##########################################################################################################################################

void LaserPredictor::guess(Poses poses)
{
    this->poses_=poses;
}
int LaserPredictor::getNumberOfPredictions()
{
    return this->poses_.size();
}
LaserPredictor::Poses LaserPredictor::getPose()
{
    std::vector<tf::Point> points=this->kMeans(this->data_.combined,this->poses_);
    Poses poses;
    for (auto point :points)
    {
        poses.push_back(tf::Pose(tf::createIdentityQuaternion(),point));
    }
    if(poses.empty())
    {
        throw std::out_of_range("No poses estaminated!");
    }
    return poses;
}
tf::Pose LaserPredictor::getPose(int i)
{
    Poses poses;
    poses=this->getPose();
    if(i<poses.size())
    {
        return poses.at(i);
    }   
    else
    {
        std::stringstream ss;
        ss<<ros::this_node::getName()<<": Requested pose "<<i<<" is out of estamination range";
        throw std::out_of_range(ss.str().c_str());
    }   
}
sensor_msgs::PointCloud LaserPredictor::getRegisteredPoints()
{
    return this->data_.combined;
}
sensor_msgs::PointCloud LaserPredictor::getClusteredPoints()
{
    return this->data_.clustered;
}


//Clustering##################################################################################################################################
// ##########################################################################################################################################
// ##########################################################################################################################################

LaserPredictor::Points LaserPredictor::kMeans(sensor_msgs::PointCloud &data,std::vector<tf::Point> &centers)
{
    if(data.points.size()<1 || centers.size()<1)
    {
        return std::vector<tf::Point>();
    }
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
            point=tf::Vector3(data.points.at(i).x,data.points.at(i).y,data.points.at(i).z);
            best_distance=HUGE_VALF;
            for(int k=0;k<centers.size();k++)
            {
                center=centers.at(k);
                distance=(point-center).length();
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
                channel.name="Cluster";
                channel.values.push_back(1.0/clusters.size()*(i+1)) ;         
            }
            result.at(i).channels.push_back(channel);
        }
    }
    catch(std::exception &e)
    {
        ROS_WARN("In Clustering: %s",e.what());
    }
  
    data.channels.clear();
    data.points.clear();
    data.header=result.at(0).header;
    sensor_msgs::ChannelFloat32 ch;
    ch.name="cluster";
    for(auto it: result)
    {
        ch.values.insert(    ch.values.end(),
                            it.channels.back().values.begin(),
                            it.channels.back().values.end());
        
        data.points.insert( data.points.end(),
                            it.points.begin(),
                            it.points.end());
    }
    data.channels.push_back(ch);
    return centers;
}

std::vector<tf::Point> LaserPredictor::kMeans(sensor_msgs::PointCloud &data,Poses &centers)
{
    if(data.points.size()<1 || centers.size()<1)
    {
        return std::vector<tf::Point>();
    }

    std::vector<tf::Point> points;
    for(auto it:centers)
    {
        points.push_back(it.getOrigin());
    }
    return this->kMeans(data,points);
}

void LaserPredictor::clustering(const ros::TimerEvent &event)
{
    this->data_.clustered=this->data_.combined;
    this->kMeans(this->data_.clustered,this->poses_);
}


sensor_msgs::PointCloud LaserPredictor::combineData(sensor_msgs::PointCloud front, sensor_msgs::PointCloud back)
{
    if(front.points.empty() &&  back.points.empty())
    {
        throw std::invalid_argument("Empty pointcloud should not be combined");
    }
    else if(back.points.empty())
    {
        return front;
    }
    else if(front.points.empty())
    {
        return back;
    }
  
    sensor_msgs::PointCloud ret;
    if(front.header.frame_id!=back.header.frame_id)
    {
        ret.header.frame_id="undefined";        
    }
    else
    {
        ret.header.frame_id=front.header.frame_id;
    }
    ret.header.stamp=ros::Time((front.header.stamp.toSec()+back.header.stamp.toSec())/2.0); 
    ret.points.insert(  ret.points.end(),
                        front.points.begin(),
                        front.points.end());



    ret.points.insert(  ret.points.end(),
                            back.points.begin(),
                            back.points.end());   
    return ret;
   
}

void LaserPredictor::transformCloud(sensor_msgs::PointCloud &cloud, tf::Transform trafo)
{
    if(cloud.points.empty())
    {
        throw std::invalid_argument("Cannot transform an empty point cloud");
    }
    for(int i=0;i<cloud.points.size();i++)
    {
        tf::Vector3 tf_point(cloud.points.at(i).x,cloud.points.at(i).y,cloud.points.at(i).z);
        tf_point=trafo*tf_point;
        cloud.points.at(i).x=tf_point.x();
        cloud.points.at(i).y=tf_point.y();
        cloud.points.at(i).z=tf_point.z();
    }
}

void LaserPredictor::startClustering(double frequenzy)
{
    this->cluster_scope_=this->nh_.createTimer(ros::Duration(1/frequenzy),&LaserPredictor::clustering,this);
}

//Callbacks##################################################################################################################################
// ##########################################################################################################################################
// ##########################################################################################################################################

void LaserPredictor::subscriberFrontCallback(const sensor_msgs::LaserScanConstPtr msg)
{
    this->data_.front=sensor_msgs::PointCloud();
    this->projector_.projectLaser(*msg,this->data_.front);
    this->transformCloud(this->data_.front,this->trafos_.front);
    this->data_.front.header.frame_id=this->nh_.resolveName(this->frames_.base);
    this->data_.combined=this->combineData(this->data_.front,this->data_.back);
    return;
}


void LaserPredictor::subscriberBackCallback(const sensor_msgs::LaserScanConstPtr msg)
{
    this->data_.back=sensor_msgs::PointCloud();
    this->projector_.projectLaser(*msg,this->data_.back);
    this->transformCloud(this->data_.back,this->trafos_.back);
    this->data_.back.header.frame_id=this->nh_.resolveName(this->frames_.base);
    this->data_.combined=this->combineData(this->data_.front,this->data_.back);    
    return;       
}

