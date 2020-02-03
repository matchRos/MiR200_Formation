#include <formation/formation_publisher.h>

FormationPublisher::FormationPublisher()
{

}
FormationPublisher::FormationPublisher(Formation* formation):formation_(formation),
                                                                publish_combined_(true),
                                                                publish_pose_(true),
                                                                publish_seperated_(true),
                                                                publish_scans_(true),
                                                                publish_scans_clustered_(true)
                                                    

{
    this->nh_=ros::NodeHandle("formation");
    ros::NodeHandle parametr_nh("formation_publisher");

    parametr_nh.param("publish_scans",this->publish_scans_,this->publish_scans_);
    parametr_nh.param("publish_scans_clustered_",this->publish_scans_clustered_,this->publish_scans_clustered_);   
    parametr_nh.param("publish_pose",this->publish_pose_,this->publish_pose_);
    parametr_nh.param("publish_seperated",this->publish_seperated_,this->publish_seperated_);
    parametr_nh.param("publish_combined",this->publish_combined_,this->publish_combined_);
    
    

    this->scan_pub_=this->nh_.advertise<sensor_msgs::PointCloud>("scan",10);
    this->cluster_scan_pub_=this->nh_.advertise<sensor_msgs::PointCloud>("clustered_scan",10);

    for(auto name:this->formation_->getNames())
    {
        ros::NodeHandle nh(this->nh_.resolveName(name));
        this->pose_pub_list_.insert(std::pair<std::string,ros::Publisher >(name,nh.advertise<geometry_msgs::PoseStamped>("pose",10)));        
        this->scan_pub_list_.insert(std::pair<std::string,ros::Publisher>(name,(nh.advertise<sensor_msgs::PointCloud>("scan",10))));
        this->cluster_scan_pub_list_.insert(std::pair<std::string,ros::Publisher>(name,(nh.advertise<sensor_msgs::PointCloud>("clustered_scan",10))));
    }
}


void FormationPublisher::publish()
{
    if(pose_pub_list_.empty() || scan_pub_list_.empty()  || cluster_scan_pub_list_.empty())
    {
        throw std::invalid_argument("Cannot publish empty Formation!");
    }
    else
    {
        if(publish_seperated_)
        {
            if(publish_pose_)
            {
               this->publishPoses();
            }
            if(publish_scans_)
            {
                this->publishSeperatedLaserScans();   
            }
            if(publish_scans_clustered_)
            {
                this->publishSeperatedClusterScans();
            }
        }
        if (publish_combined_)
        {
            if(publish_pose_)
            {
                this->publishPoses();
            }
            if(publish_scans_)
            {
                this->publishLaserScans();
            }
            if(publish_scans_clustered_)
            {
                
                this->publishClusteredLaserScans();
            }
        }
    }    
}



void FormationPublisher::publishLaserScans()
{
    this->scan_pub_.publish(this->formation_->getScan());
}

void FormationPublisher::publishClusteredLaserScans()
{
    this->cluster_scan_pub_.publish(this->formation_->getClusteredScan());
}

void FormationPublisher::publishSeperatedLaserScans()
{
   for(auto publisher:this->scan_pub_list_)
    {
        publisher.second.publish(this->formation_->getScan(publisher.first));
    }
}

void FormationPublisher::publishSeperatedClusterScans()
{
    for(auto publisher:this->cluster_scan_pub_list_)
    {
        publisher.second.publish(this->formation_->getClusteredScan(publisher.first));
    }
}

void FormationPublisher::publishPoses()
{
    for(auto publisher:this->pose_pub_list_)
    {
        geometry_msgs::PoseStamped msg;
        msg.header.frame_id=this->formation_->getReferenceFrame();
        tf::poseTFToMsg(this->formation_->getPose(publisher.first),msg.pose);
        publisher.second.publish(msg);
    }
}
