#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
#include <multi_robot_msgs/Formation.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <numeric>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/ChannelFloat32.h>

#include <controller/laser_predictor.h>



/**
 * @brief A Class for handeling basic mobile Robot formation work. Properly adds Robots to a 
 * formation and provides basic interpretation tools as the adjacency matrix of the formation.
 * 
 */
class Formation{
    
    public:
        /**
         * @brief Template for a generic matrix as std::vector of std::vectors
         * 
         * @tparam T type of the matrix values
         */
        template <typename T>
        using Matrix =std::vector<std::vector<T> >;

        typedef std::vector<std::string> Neighbours;

        typedef std::shared_ptr<LaserPredictor> LaserPointer;

        using Poses=LaserPredictor::Poses;

        using Cloud=LaserPredictor::Cloud;

        struct RobotProperties{
            RobotProperties(){}
            std::string name;
            tf::Pose pose;
            Formation::Neighbours neighbours;
            LaserPredictor::Frames laser_frames;
            LaserPredictor::Topics laser_topics;
        };

        struct Robot{
            Robot(){};
            tf::Pose pose;
            Neighbours neighbours;
            LaserPointer predictor;           
        };

    
        /**
         * @brief Defines a Transformation between two Formations
         * 
         */
        typedef Formation Transformation;

      
        Formation();


        void addRobot(RobotProperties robot);

        /**
         * @brief Modifies the pose of the ith robot within the Formation
         * 
         * @param i Index of the robot to be modified
         * @param pose modified Pose of the robot
         */
        void modifiePose(std::string name,tf::Pose pose);

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
         * @brief Get the Names object
         * 
         * @return std::vector<std::string> Names of all robots within the formation
         */
        std::vector<std::string> getNames();

        std::string getReferenceFrame();

        /**
         * @brief Gets a vector that contains every single robot pose
         * 
         * @return std::vector<tf::Pose> Vector of the robot poses
         */
        std::vector<tf::Pose> getPoses();    

        /**
         * @brief Get the Pose object of a specified robot
         * 
         * @param name Name of the robot to get the pose from
         * @return tf::Pose pose of the robot
         */
        tf::Pose getPose(std::string name);


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