#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <numeric>
#include <tf/transform_listener.h>


/**
 * @brief A class for predictings neighbour robots from the laser scanner data of another robot.
 * 
 */
class LaserPredictor{
    
    public:
        /**
         * @brief Holds the names of all the necessary frames for the prediction class.
         * 
         */
        struct Frames{
            Frames(){};
            /**
             * @brief Construct a new Frames object
             * 
             * @param base Name of the base link
             * @param front Name of the front laser link
             * @param back Name of the back laser link
             */
            Frames(std::string base,std::string front, std::string back):   base(base),
                                                                            front(front),
                                                                            back(back)
            {}
            std::string base;   ///<Frame name of base link
            std::string front;  ///<Frame name of front laser link
            std::string back;   ///<Frame name of the back laser link
        };        
        
        /**
         * @brief Holds all necessary transformations
         * 
         */
        struct Transforms{
            tf::StampedTransform front; ///<Transformation from front link to base link
            tf::StampedTransform back;  ///<Transformation from back link to base link
        };

        /**
         * @brief Holds all the data used within the predictor
         * 
         */
        struct Data{
            sensor_msgs::PointCloud front;  ///< Data from the front laser scanner
            sensor_msgs::PointCloud back;   ///< Data from the back laser scanner
            sensor_msgs::PointCloud combined;   ///< Combined data of frot and back laser scanner
            sensor_msgs::PointCloud combined_clustered;  ///< combined Clustered data 
            std::vector<sensor_msgs::PointCloud> clusters;   //<Clusters
        };

        /**
         * @brief Defines Poses as a std::vector of tf::Poses
         * 
         */
        typedef std::vector<tf::Pose> Poses;
        /**
         * @brief Defines Points as a std::vector of tf::Points
         * 
         */
        typedef std::vector<tf::Point> Points;
        /**
         * @brief Defines Cloud as sensor_msgs::PointCloud
         * 
         */
        typedef sensor_msgs::PointCloud Cloud;
        /**
         * @brief Defines Topics as Frames. Topics holds simmilare names as Frames but for topics not for frames.
         * 
         */
        typedef Frames Topics;

        typedef std::vector<sensor_msgs::PointCloud> Clusters;



        /**
         * @brief Combines the data front and back together
         * 
         * @param front First Cloud to combine
         * @param back Second Cloud to combine
         * @return Cloud Combined cloud
         */
        static Cloud combineData(Cloud front, Cloud back);


        /**
         * @brief Applies the transform trafo to each point in the cloud
         * 
         * @param cloud cloud to be transformed
         * @param trafo trafo to be applied
         */
        static void transformCloud(Cloud &cloud, tf::Transform trafo);

        
        
        
        
        
        
        
        
        
        LaserPredictor();
        /**
         * @brief Construct a new Laser Predictor object
         * 
         * @param nh Node handle to handle namespaces and ros fucntionality
         * @param frames Holds all the nescessary frames for the predictor
         * @param topic Holds all the nescessary topics for the predictor   
         */
        LaserPredictor( ros::NodeHandle &nh,
                        Frames frames,
                        Topics topic);

        /**
         * @brief Get the Number Of Predictions object
         * 
         * @return int Number of Predictions
         */
        int getNumberOfPredictions();


        /**
         * @brief Get the predicted Poses
         * 
         * @return Poses Predicted Poses
         */
        Poses getPoses();

        /**
         * @brief Get one of the Pose objects
         * 
         * @param i Index of the Pose to get
         * @return tf::Pose  Pose at this index
         */
        tf::Pose getPose(int i);

        /**
         * @brief Get the combination of front and back laser data
         * 
         * @return Cloud The combination of front and back laser data
         */
        Cloud getRegisteredPoints();

        /**
         * @brief Get the combinaton of front and back laser data with cluster signature
         * 
         * @return Cloud Data wich holds a channel for the cluster number
         */
        Cloud getClusteredPoints();

        /**
         * @brief Give a guess where cluster(robots) are with respect to the base link frame
         * 
         * @param values each tf::Pose specifies a possible robot and is used as start value for clustering algorithms
         */
        void guess(Poses values);

        /**
         * @brief Starts the cluster algorithm
         * 
         * @param frequenzy Frequenzy how often the algorithm should be executed
         */
        void startClustering(double frequenzy);

    private:
        ros::NodeHandle nh_;    ///<Nodehandle for handling namespaces and ros functionality

        ros::Subscriber sub_front_; ///<Subscriber for front laser scanner
        ros::Subscriber sub_back_;  ///<Subscriber for back laser scanner

        ros::Timer cluster_scope_;  ///<Timer for the clustering scope

        laser_geometry::LaserProjection projector_; ///< Is yoused to project the laser scanner data into cartesian space

        Transforms trafos_; ///<Holds the trasnformatiosn from base to the laser links        
        Frames frames_; ///<Holds the names of the links of the base and the laser links
        Data data_; ///<Holds all pointcloud data
        Poses poses_;   ///< Holds all estaminated poses of the robots
        

        /**
         * @brief Timer callback to execute the cluster algorithm
         * 
         * @param event Ros timer event
         */
        void clustering(const ros::TimerEvent& event);

        /**
         * @brief Executes k-means clustering at the data with respect to the guessed cluster ceneters centers
         * 
         * @param data Data wich are modified during the clustering process
         * @param centers Centers wich are modified during the clustering process
         * @return Points Final cluster centers
         */
        Clusters kMeans(Cloud &data,Points &centers);
        
         /**
         * @brief Executes k-means clustering at the data with respect to the guessed cluster ceneters centers
         * 
         * @param data Data wich are modified during the clustering process
         * @param centers Centers wich are modified during the clustering process
         * @return Points Final cluster centers
         */
        Clusters kMeans(Cloud &data,Poses &centers); 

        /**
         * @brief Callback for the front laser scanner subscribtion
         * 
         * @param msg data at the laser scanner topic
         */
        void subscriberFrontCallback(const sensor_msgs::LaserScanConstPtr msg);

        /**
         * @brief Callback for the back laser scanner subscribtion
         * 
         * @param msg data at the laser scanner topic
         */
        void subscriberBackCallback(const sensor_msgs::LaserScanConstPtr msg);
};
