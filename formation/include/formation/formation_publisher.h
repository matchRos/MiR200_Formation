#pragma once

#include<ros/ros.h>
#include<formation/formation.h>

class FormationPublisher{
    
    public:
        /**
         * @brief Construct a new Formation Publisher object
         * 
         */
        FormationPublisher();

        /**
         * @brief Construct a new Formation Publisher object for a given formation
         * 
         * @param formation pointer to the formation wich data should be published
         */
        FormationPublisher(Formation* formation);



        void publishLaserScans();


        void publishClusteredLaserScans();


        void publishScanPoses();

        /**
         * @brief Publishs the Laser scanner data of the formation
         * 
         */
        void publishSeperatedLaserScans();

        /**
         * @brief Publishs the clustered Laser scanner data of the formation
         * 
         */        
        void publishSeperatedClusterScans();

        /**
         * @brief Publish the Poses wich are determined from the lasser scanner data within the formation
         * 
         */
        void publishSeperatedScannedPoses();

       /**
        * @brief Publishs all the formation data
        * 
        */
        void publish();

    private:
        std::shared_ptr<Formation> formation_;  //< Pointer to the formation wich data should be published
        ros::NodeHandle nh_;    //< Nodehandle for clearing namespaces of the published topics

        ros::Publisher scan_pub_;   //< Publihser for the hole formation scanner data
        ros::Publisher cluster_scan_pub_;   //<Publisher for the hole clustered scan data
        std::map<std::string,std::vector<ros::Publisher> > scanned_pose_sep_pub_list_;  //< map that links a robot name to a list of publihshers for publishing every scanned pose
        std::map<std::string,ros::Publisher> scan_pub_list_;    //< List of publishers for publishing each robots laser scanner data
        std::vector<ros::Publisher> scanned_pose_pub_list_;
        std::map<std::string,ros::Publisher> cluster_scan_pub_list_;    //< List of publisher for publishing each robots clustered laser scanner data

        /**
         * @brief Allocates size publisher for topics within the nodehandel namespaces and for the robot with name
         * 
         * @param nh_topic Namespace for resolving topic names
         * @param name  Name of the robot to allocate publishers for
         * @param size Number of publishers to allocate
         */
        void allocScannedPosePublishers(ros::NodeHandle nh_topic,std::string name,size_t size);
};
