#include <controller/laser_predictor.h>

LaserPredictor::LaserPredictor(ros::NodeHandle &nh,std::string topic_name_front,std::string topic_name_back)
{
    this->nh_=nh;
    this->sub_front_=this->nh_.subscribe<sensor_msgs::LaserScan>(this->nh_.resolveName(topic_name_front),10,boost::bind(&LaserPredictor::subscriberFrontCallback,this,_1));
    this->sub_front_=this->nh_.subscribe<sensor_msgs::LaserScan>(this->nh_.resolveName(topic_name_back),10,boost::bind(&LaserPredictor::subscriberBackCallback,this,_1));
    this->first_front_msgs_received_=false;
    this->first_back_msgs_received_=false;
    this->base_frame_="base_link";
}
void LaserPredictor::guess(GuessedValues guessed_values)
{
    this->guessed_values_=guessed_values;
}
int LaserPredictor::getNumberOfPredictions()
{
    return this->estaminated_poses_.size();
}
std::vector<tf::Pose> LaserPredictor::getPose()
{
    std::vector<tf::Point> points=this->kMeans(this->registered_point_cloud_,this->guessed_values_.poses);
    std::vector<tf::Pose> poses;
    for (auto point :points)
    {
        poses.push_back(tf::Pose(tf::createIdentityQuaternion(),point));
    }
    return poses;
}
tf::Pose LaserPredictor::getPose(int i)
{
    std::vector<tf::Pose> poses;
    poses=this->getPose();
    if(i<poses.size())
    {
        return poses.at(i);
    }   
    else
    {
        throw std::out_of_range("Requested pose in laser estamination is out of range!");
    }   
}
sensor_msgs::PointCloud LaserPredictor::getRegisteredPoints()
{
    return this->registered_point_cloud_;
}

std::vector<tf::Point> LaserPredictor::kMeans(sensor_msgs::PointCloud &data,std::vector<tf::Point> &centers)
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

std::vector<tf::Point> LaserPredictor::kMeans(sensor_msgs::PointCloud &data,std::vector<tf::Pose> &centers)
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


void LaserPredictor::subscriberFrontCallback(const sensor_msgs::LaserScanConstPtr msg)
{
   
    if(!listener_.waitForTransform(  this->nh_.resolveName(this->base_frame_),
                                    msg->header.frame_id,
                                    msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),
                                    ros::Duration(1.0)))    
                              
    {
        return;
    }
    else
    {
        sensor_msgs::PointCloud cloud;
        this->registered_point_cloud_.header.frame_id=this->nh_.resolveName(this->base_frame_);
        this->projector_.transformLaserScanToPointCloud(this->nh_.resolveName(this->base_frame_),*msg,cloud,listener_);         
        this->registered_point_cloud_.points.clear();
        this->registered_point_cloud_.channels.clear();
        this->registered_point_cloud_.points.insert(    this->registered_point_cloud_.points.end(),
                                                        cloud.points.begin(),
                                                        cloud.points.end()); 
         return;
    }    
}

void LaserPredictor::subscriberBackCallback(const sensor_msgs::LaserScanConstPtr msg)
{
    if(!listener_.waitForTransform(  this->nh_.resolveName(this->base_frame_),
                                    msg->header.frame_id,
                                    msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment),
                                    ros::Duration(1.0)))       
                              
    {
        return;
    }
    else
    {
        sensor_msgs::PointCloud cloud;
        this->registered_point_cloud_.header.frame_id=this->nh_.resolveName(this->base_frame_);
        this->projector_.transformLaserScanToPointCloud(this->nh_.resolveName(this->base_frame_),*msg,cloud,listener_);
        this->registered_point_cloud_.points.insert(    this->registered_point_cloud_.points.end(),
                                                        cloud.points.begin(),
                                                        cloud.points.end());
        return;
    }    
}

