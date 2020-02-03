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


        void publishPoses();

        /**
         * @brief Publishs the laser scan data of the hole formation
         * 
         */
        void publishLaserScans();

        /**
         * @brief Publishs the clustered laser scan data of the hole formation
         * 
         */
        void publishClusteredLaserScans();

        /**
         * @brief Publishs the Poses wich are predicted by the laser scanner data
         * 
         */
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
      
        std::map<std::string,ros::Publisher> pose_pub_list_;
        std::map<std::string,ros::Publisher> scan_pub_list_;    //< List of publishers for publishing each robots laser scanner data
        std::map<std::string,ros::Publisher> cluster_scan_pub_list_;    //< List of publisher for publishing each robots clustered laser scanner data
       
        bool publish_pose_;
        bool publish_scans_;
        bool publish_scans_clustered_;
        bool publish_seperated_;
        bool publish_combined_;
};
