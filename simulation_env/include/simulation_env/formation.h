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



/**
 * @brief A Class for handeling basic mobile Robot formation work. Properly adds Robots to a 
 * formation and provides basic interpretation tools as the adjacency matrix of the formation.
 * 
 */
class Formation{

    public:
        /**
         * @brief Defines a Transformation between two Formations
         * 
         */
        typedef Formation Transformation;

        template <typename T>
        using Matrix =std::vector<std::vector<T> >;

        /**
         * @brief Construct a new Formation object
         * 
         */
        Formation();

        /**
         * @brief Adds another Robot to the formation
         * 
         * @param name Name of the robot for representation
         * @param pose Pose of the robot to be added
         * @param neighbours Neighbours of the Robot if they should be linked (optional)
         */
        void addRobot(tf::Pose pose,std::string name,std::vector<int> neighbours=std::vector<int>());

        /**
         * @brief Set the Reference Frame object
         * 
         * @param frame_name Name of the frame geometry is defined in
         */
        void setReferenceFrame(std::string frame_name);

        /**
         * @brief Modifies the pose of the ith robot within the Formation
         * 
         * @param i Index of the robot to be modified
         * @param pose modified Pose of the robot
         */
        void modifiePose(int i,tf::Pose pose);




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
         * @brief Gets a vector that contains every single robot pose
         * 
         * @return std::vector<tf::Pose> Vector of the robot poses
         */
        std::vector<tf::Pose> getPoses();        

        /**
         * @brief Get the Name of robot i
         * 
         * @param i Number of the robot
         * @return std::string name of the robot
         */
        std::string getName(int i);        

        /**
         * @brief Get the Reference Frame object
         * 
         * @return std::string Name of the reference frame the geometry is defined in
         */
        std::string getReferenceFrame();

        /**
         * @brief Get the adjacency matrix of the formation
         * 
         * @return std::vector<std::vector<double> > adjacency matrix
         */       
        Matrix<double> getAdjacency();  






        /**
         * @brief Applies Transformation to the formation
         * 
         * @param trafo Transformation to apply
         * @param formation The Formation object that should be transformed
         * @return Formation Transformed Formation
         */
        static Formation transform(Formation formation,tf::Transform trafo);

        
        /**
         * @brief Calculates the difference between this and another formations
         * 
         * @param target The Formation the difference vector leads to
         * @return Formation 
         */
        Transformation operator-(Formation &target);

       
    private:
        std::string refrence_frame; ///<Name of the reference frame geometry is defined in
        
        std::vector<std::string> names_;    ///<Vector that holds the names of the different robots
        std::vector<tf::Pose> formation_;   ///<contains the poses of the robots
        
        Matrix<double>  adjacency_;   ///<contains the adjacence matrice of the formation
        Matrix<bool>  connectivity_;  ///<contains the connectivity matrice of the formation



        /**
         * @brief Determines the connectivity matrix from a given neighbours vector
         * 
         * @param neighbours Defines the neighbourhood the connectivity is determined from
         */
        void determineConnectivity(std::vector<int> neighbours);


        void determineAdjacency();

};