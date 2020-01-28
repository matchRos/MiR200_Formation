#include <formation/formation_publisher.h>

FormationPublisher::FormationPublisher()
{

}
FormationPublisher::FormationPublisher(Formation* formation):formation_(formation)

{
    this->nh_=ros::NodeHandle("formation");

    this->scan_pub_=this->nh_.advertise<sensor_msgs::PointCloud>("scan",10);
    this->cluster_scan_pub_=this->nh_.advertise<sensor_msgs::PointCloud>("clustered_scan",10);

    for(auto name:this->formation_->getNames())
    {
        ros::NodeHandle nh(this->nh_.resolveName(name));
        this->scanned_pose_pub_list_.push_back(nh.advertise<geometry_msgs::PoseStamped>("scan_pose",10));
        this->scanned_pose_sep_pub_list_.insert(std::pair<std::string,std::vector<ros::Publisher> >(name,std::vector<ros::Publisher>()));        
        this->scan_pub_list_.insert(std::pair<std::string,ros::Publisher>(name,(nh.advertise<sensor_msgs::PointCloud>("scan",10))));
        this->cluster_scan_pub_list_.insert(std::pair<std::string,ros::Publisher>(name,(nh.advertise<sensor_msgs::PointCloud>("clustered_scan",10))));
    }
}


void FormationPublisher::publish()
{
    if(scanned_pose_sep_pub_list_.empty()   ||  scan_pub_list_.empty()  || cluster_scan_pub_list_.empty())
    {
        std::string str;
        throw std::invalid_argument("Cannot publish empty Formation!");
    }
    else
    {
        this->publishLaserScans();
        this->publishClusteredLaserScans();
        this->publishScanPoses();
        this->publishSeperatedClusterScans();
        this->publishSeperatedLaserScans();
        this->publishSeperatedScannedPoses();
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

void FormationPublisher::publishSeperatedScannedPoses()
{
    for(auto pair:this->scanned_pose_sep_pub_list_)
    {
        std::string name=pair.first;

        //Prepare namespaces
        ros::NodeHandle nh_topic=this->nh_.resolveName(name+"/scanned_poses");
        Formation::Poses poses=this->formation_->getScannedPose(name);

        //Allocate enought publihser for publishing estaminations
        allocScannedPosePublishers(nh_topic,name,poses.size());        

        //Publish every esatminated pose
        for(int i=0; i<pair.second.size();i++)
        {
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.frame_id=ros::NodeHandle(name).resolveName("base_link");
            tf::poseTFToMsg(poses.at(i),pose_msg.pose);
            pair.second.at(i).publish(pose_msg);
        }  
    }
}

void FormationPublisher::publishScanPoses()
{
    Formation::Poses poses;
    poses=this->formation_->getScannedPose();
    for(int i=0;i<this->scanned_pose_pub_list_.size();i++)
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id=this->formation_->getReferenceFrame();
        tf::poseTFToMsg(poses.at(i),pose_msg.pose);
        this->scanned_pose_pub_list_.at(i).publish(pose_msg);
    }
}

void FormationPublisher::allocScannedPosePublishers(ros::NodeHandle nh_topic,std::string name,size_t size)
{
    while(size>this->scanned_pose_sep_pub_list_.at(name).size())
    {
        std::stringstream ss;
        ss<<"prediction_"<<this->scanned_pose_sep_pub_list_.at(name).size();
        ROS_WARN("Expanding publisher lsit to size: %i!",this->scanned_pose_sep_pub_list_.at(name).size());
        this->scanned_pose_sep_pub_list_.at(name).push_back(nh_topic.advertise<geometry_msgs::PoseStamped>(ss.str(),1));        
    }
}

