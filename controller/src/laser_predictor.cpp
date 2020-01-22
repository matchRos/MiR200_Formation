#include <controller/laser_predictor.h>

LaserPredictor::LaserPredictor(ros::NodeHandle &nh):nh_(nh)
{
    
}

int LaserPredictor::getNumberOfPredictions()
{
    return this->estaminated_poses_.size();
}
std::vector<tf::Pose> LaserPredictor::getPose()
{
    return this->estaminated_poses_;
}
tf::Pose LaserPredictor::getPose(int i)
{
    return this->estaminated_poses_.at(i);
}


std::vector<sensor_msgs::PointCloud> LaserPredictor::kMeans(sensor_msgs::PointCloud &data,std::vector<tf::Point> &centers)
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
    
    data=sensor_msgs::PointCloud();
    data.channels.at(0).name="Cluster";
    for(auto it: result)
    {
        data.channels.at(0).values.insert(  data.channels.at(0).values.end(),
                                            it.channels.at(0).values.begin(),
                                            it.channels.at(0).values.end());
        data.points.insert(data.points.begin(),
                            it.points.begin(),
                            it.points.end());
    }
    return result;
}

std::vector<sensor_msgs::PointCloud> LaserPredictor::kMeans(sensor_msgs::PointCloud &data,std::vector<tf::Pose> &centers)
{
    std::vector<tf::Point> points;
    for(auto it:centers)
    {
        points.push_back(it.getOrigin());
    }
    return this->kMeans(data,points);
}


void LaserPredictor::subscriberCallback(const sensor_msgs::LaserScanPtr msg)
{
    this->registered_point_cloud_.header=msg->header;      
    this->projector_.projectLaser(*msg,  this->registered_point_cloud_);
}
