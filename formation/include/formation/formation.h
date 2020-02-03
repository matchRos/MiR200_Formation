#pragma once

#include <ros/ros.h>
#include <tf/tf.h>

#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <numeric>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <formation/laser_predictor.h>
#include <formation/odometry_predictor.h>



/**
 * @brief A Class for handeling basic mobile Robot formation work. Properly adds Robots to a 
 * formation and provides basic interpretation tools as the adjacency matrix of the formation.
 * 
 */

   


class Formation{
    public:        
        enum PoseEstamination
        {
            by_laser_scanner,
            by_odometry,
            by_ekf
        };
        /**
         * @brief Template for a generic matrix as std::vector of std::vectors
         * 
         * @tparam T type of the matrix values
         */
        template <typename T>
        using Matrix =std::vector<std::vector<T> >;

        /**
         * @brief Defines Neighbours as vector of robot names
         * 
         */
        typedef std::vector<std::string> Neighbours;

        /**
         * @brief Defines LaserPointer as shared pointer to a LaserPredictor
         * 
         */
        typedef std::shared_ptr<LaserPredictor> LaserPointer;

        typedef std::shared_ptr<OdometryPredictor> OdomPointer;

        /**
         * @brief Brings Poses up to scope
         * 
         */
        using Poses=LaserPredictor::Poses;

        /**
         * @brief Brings Cloud up to scope
         * 
         */
        using Cloud=LaserPredictor::Cloud;

        
        /**
         * @brief Defines Properties of a Robot
         * 
         */
        struct RobotProperties
        {
            RobotProperties(){}
            std::string name;   ///<Name of the robot
            tf::Pose pose;  ///<Pose of the robot
            Formation::Neighbours neighbours;   ///<neighbours of the robot by name
            LaserPredictor::Frames laser_frames;    //<Frames of the laserscanner links
            LaserPredictor::Topics laser_topics;    ///<Topics the laserscanner messages are published at
            std::string odom_topic; ///<Name of the odometry topic
            std::string ekf_topic;  ///<Name of the ekf topic
        };

        /**
         * @brief Holds a robot to handle with formation member stuff
         * 
         */
        struct Robot{
            Robot(){};
            tf::Pose pose;  ///<Pose of the robot
            Neighbours neighbours;  ///<Neighbours of the robot
            LaserPointer laser; ///<LaserPredictor for this robot
            OdomPointer odom;   ///<OdometryPredictor for this robot
            OdomPointer ekf;    ///<EKF Predictor for this robot
        };

    
        /**
         * @brief Defines a Transformation between two Formations
         * 
         */
        typedef Formation Transformation;

      

        /**
         * @brief Construct a new Formation object
         * 
         */
        Formation();

        /**
         * @brief Gets the number of Robots within the formation
         * 
         * @return int Number of Robots
         */
        int size();

        /**
         * @brief Checks wheather the formation doesnt contain any robot
         * 
         * @return true Formation is empty
         * @return false Formation contains at least one robot
         */
        bool empty(); 

        /**
         * @brief Adds a robot with given properties to the formation
         * 
         * @param robot Properties of the added robot
         */
        void addRobot(RobotProperties robot);

        /**
         * @brief Modifies the pose of the ith robot within the Formation
         * 
         * @param i Index of the robot to be modified
         * @param pose modified Pose of the robot
         */
        void modifiePose(std::string name,tf::Pose pose);

        /**
         * @brief Get the Names object
         * 
         * @return std::vector<std::string> Names of all robots within the formation
         */
        std::vector<std::string> getNames();

        /**
         * @brief Get the Reference Frame object
         * 
         * @return std::string Name of the reference frame formation wide geometry is defined in
         */
        std::string getReferenceFrame();

        /**
         * @brief Gets a vector that contains every single robot pose. Estamination of pose is specified by the class type paramter
         * 
         * @return std::vector<tf::Pose> Vector of the robot poses
         */
        Poses getPose();

        /**
         * @brief Get the Poses object.
         * 
         * @param type type of estamination to be used
         * @return Poses Estaminated poses
         */
        Poses getPose(PoseEstamination type);  

        /**
         * @brief Get the Pose object of a specified robot. Estamination of pose is specified by the class type paramter
         * 
         * @param name Name of the robot to get the pose from
         * @return tf::Pose pose of the robot
         */
        tf::Pose getPose(std::string name);

        /**
         * @brief Get the Poses object of a specified robot
         * 
         * @param name Name of the robot
         * @param type type of pose estamination
         * @return tf::Pose Pose of the robot
         */
        tf::Pose getPose(std::string name,PoseEstamination type);



        /**
         * @brief Get the Scanner data from all of the formation robots
         * 
         * @return Cloud Data from the laser scanners
         */
        Cloud getScan();
        
        /**
         * @brief Get the Scan data from a specific robot
         * 
         * @param name name of the robot
         * @return Cloud data from its laser scanners
         */
        Cloud getScan(std::string name);

        /**
         * @brief Get the clustered scanner data for every robot within the formation
         * 
         * @return LaserPredictor::Cloud 
         */
        Cloud getClusteredScan();

        /**
         * @brief Get the Clustered Scan data for a specific robot
         * 
         * @param name Name of the robot to get data from
         * @return Cloud Data from the robots laserscanner
         */
        Cloud getClusteredScan(std::string name);


         /**
         * @brief Get the Scanned Poses object
         * 
         * @return Poses Poses of the identified neighbours of the robot
         */
        Poses getScannedPose();

        /**
         * @brief Get the Scanned Poses object
         * 
         * @param name Name of the robot to get the scanned poses of its neighbours from 
         * @return Poses Poses of the identified neighbours of the robot
         */
        Poses getScannedPose(std::string name);

        /**
         * @brief Starts the prediciton algorithms within the formation
         * 
         * @param frequenzy    Frequenzy at wich a prediction should be calculated
         */
        void startPrediction(double frequenzy);
        



        /**
         * @brief Calculates and returns the Adjacency matrix of the formation
         * 
         * @return std::vector<std::vector<double> > adjacency matrix
         */       
        Formation::Matrix<double> getAdjacency();  

        /**
         * @brief Calculates and returns the Connectivity matrix of the formation
         * 
         * @return Formation::Matrix<bool> 
         */
        Formation::Matrix<bool> getConnectivity();


       
    private:
        std::string reference_frame_;   ///<Name of the referece frame formation geometries are defined in
        unsigned int number_of_robots_; ///<Contains the number of robots within the formation

        std::map<std::string,Robot> formation_map_; ///<Maps a special robot name to its correspondence Robot
        std::map<std::string,unsigned int> index_map_;  ///<Maps a special idex of a robot to its name

     
        Matrix<double>  adjacency_;   ///<contains the adjacence matrice of the formation
        Matrix<bool>  connectivity_;  ///<contains the connectivity matrice of the formation

        PoseEstamination type_;
        
        /**
         * @brief Calcualte the robot pose from the laserscanner data
         * 
         * @param name Name of the robot to get the pose of
         * @return tf::Pose Pose of the robot specified by name
         */
        tf::Pose getPoseByLaser(std::string name);

           /**
         * @brief Calcualte the robot pose from the odomotry data
         * 
         * @param name Name of the robot to get the pose of
         * @return tf::Pose Pose of the robot specified by name
         */
        tf::Pose getPoseByOdom(std::string name);

        /**
         * @brief Calcualte the robot pose from the kalman filtered data
         * 
         * @param name Name of the robot to get the pose of
         * @return tf::Pose Pose of the robot specified by name
         */
        tf::Pose getPoseByEkf(std::string name);

        /**
         * @brief Calcualte the robot poses from the laserscanner data
         * 
         * @return Poses Poses of the robots
         */        
        Poses getPoseByLaser();

        /**
         * @brief Calcualte the robot poses from the odometry data
         * 
         * @return Poses Poses of the robots
         */        
        Poses getPoseByOdom();

        /**
         * 
         * @brief Calcualte the robot poses from the kalman filtered data
         * 
         * @return Poses Poses of the robots
         */        
        Poses getPoseByEkf();
        
        /**
         * @brief Determines the connectivity matrix of the formation
         * 
         * @return Matrix<bool> Connectivity matrix
         */
        Matrix<bool> determineConnectivity();


        /**
         * @brief Determines the adjacency matrix of the formation
         * 
         * @return Matrix<double> Adjacency matrix
         */
        Matrix<double> determineAdjacency();


        tf::Transform transformBetweenRobots(Robot robot1, Robot robot2);
};