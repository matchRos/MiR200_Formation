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
        enum AdjacencyCalculation{
            calc_by_pose,
            calc_by_scan
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
         * @brief Adds another Robot to the formation
         * 
         * @param name Name of the robot for representation
         * @param pose Pose of the robot to be added
         * @param neighbours Neighbours of the Robot if they should be linked (optional)
         */
        void addRobot(tf::Pose pose,std::string name,std::vector<int> neighbours=std::vector<int>());

        /**
         * @brief Gets the number of Robots within the formation
         * 
         * @return int Number of Robots
         */
        int size();

        /**
         * @brief Gets a vector that contains every single robot pose
         * 
         * @return std::vector<tf::Pose> Vector of the robot poses
         */
        std::vector<tf::Pose> getPoses();

        /**
         * @brief Get the Scan data
         * 
         * @return std::vector<sensor_msgs::LaserScan> vector of laser scan data of each robot
         */
        std::vector< sensor_msgs::PointCloud> getScan();

        /**
         * @brief Gets the scan data of  the i-th robot
         * 
         * @param i index of the robot to get scan data from
         * @return sensor_msgs::LaserScan Laser scan data of the i-th robot
         */
         sensor_msgs::PointCloud getScan(int i);
        
        /**
         * @brief Get the Combined Scan
         * 
         * @return sensor_msgs::LaserScan Combined Laserscan data of the formation
         */
         sensor_msgs::PointCloud getCombinedScan();

        /**
         * @brief Get the Name of robot i
         * 
         * @param i Number of the robot
         * @return std::string name of the robot
         */
        std::string getName(int i);

        /**
         * @brief Set the Reference Frame object
         * 
         * @param frame_name Name of the frame geometry is defined in
         */
        void setReferenceFrame(std::string frame_name);

        /**
         * @brief Get the Reference Frame object
         * 
         * @return std::string Name of the reference frame the geometry is defined in
         */
        std::string getReferenceFrame();


        std::vector<sensor_msgs::PointCloud> getClusteredData();
        /**
         * @brief Get the adjacency matrix of the formation
         * 
         * @return std::vector<std::vector<double> > adjacency matrix
         */       
        std::vector<std::vector<double> > getAdjacency();

        /**
         * @brief Applies Transformation to the formation
         * 
         * @param trafo Transformation to apply
         * @param formation The Formation object that should be transformed
         * @return Formation Transformed Formation
         */
        static Formation transform(Formation formation,tf::Transform trafo);

        /**
         * @brief Determines the connectivity matrix from a given neighbours vector
         * 
         * @param neighbours Defines the neighbourhood the connectivity is determined from
         */
        void determineConnectivity(std::vector<int> neighbours);

         /**
         * @brief Modifies the pose of the ith robot within the Formation
         * 
         * @param i Index of the robot to be modified
         * @param pose modified Pose of the robot
         */
        void modifiePose(int i,tf::Pose pose);

        /**
         * @brief Modifies the scan data of the ith robot within the Formation
         * 
         * @param i Index of the robot to be modified
         * @param datum modified laserscan data
         */
        void modifieLaserdata(int i,  sensor_msgs::PointCloud);

        /**
         * @brief Checks wheather the formation doesnt contain any robot
         * 
         * @return true Formation is empty
         * @return false Formation contains at least one robot
         */
        bool empty();      
        
        /**
         * @brief Calculates the difference between this and another formations
         * 
         * @param target The Formation the difference vector leads to
         * @return Formation 
         */
        Transformation operator-(Formation &target);

       
    private:
        AdjacencyCalculation adjacency_calculation_; ///<defines how the adjacency matrix of the formation is determined
        
        std::string refrence_frame; ///<Name of the reference frame geometry is defined in
        std::vector<std::string> names_;    ///<Vector that holds the names of the different robots
        std::vector<tf::Pose> formation_;   ///<contains the poses of the robots
        std::vector<std::vector<double> > adjacency_;   ///<contains the adjacence matrice of the formation
        std::vector<std::vector<bool> > connectivity_;  ///<contains the connectivity matrice of the formation
        std::vector< sensor_msgs::PointCloud> scan_data_; ///<data of the laserscanners of each single robot
        std::vector<sensor_msgs::PointCloud> clustered_data_;///<data after clustering
        std::vector<std::vector<double> > calcAdjacencyFromScan(std::vector< sensor_msgs::PointCloud> data);
        std::vector<std::vector<double> > calcAdjacencyFromPoses(std::vector<tf::Pose> data);
        sensor_msgs::PointCloud combineScanData(std::vector< sensor_msgs::PointCloud> data);
        std::vector<sensor_msgs::PointCloud> cluster(sensor_msgs::PointCloud data,std::vector<tf::Point> &centers);
};



class FormationPublisher{
    public:
        FormationPublisher();
        FormationPublisher(ros::NodeHandle nh,std::string topic);
        /**
         * @brief Publishes the formation to the topic defined by the publisher
         * 
         * @param formation Formation to be published
         */
        void publish(Formation formation);
        /**
         * @brief Converts a vector of poses to a Formation message type
         * 
         * @param formation Poses that should be converted
         * @param msg Reference to the message storage
         */
        static void Formation2Msg(std::vector<tf::Pose> formation,multi_robot_msgs::Formation &msg);
        /**
         * @brief Converts a Formation object to a Formation msg type
         * 
         * @param formation Formation that should be converted
         * @param msg Reference to the message storage
         */
        static void Formation2Msg(Formation formation,multi_robot_msgs::Formation &msg);

    private:
        std::string topic_name_;    
        ros::NodeHandle nh_;
        ros::Publisher pub_form_;
        ros::Publisher pub_scan_;

};



class FormationSubscriber{
    public:
        FormationSubscriber(ros::NodeHandle &nh,Formation* formation,std::vector<std::string> topics);
        void callback_odometry(const nav_msgs::OdometryConstPtr& msg,int number);
        void callback_laserscanner(const sensor_msgs::LaserScanConstPtr &msg, int number);
        
    private:
        laser_geometry::LaserProjection projector_;
        Formation* formation_;
        tf::TransformListener listener_;
        std::list<ros::Subscriber> odom_subscribers_;
        std::list<ros::Subscriber> laser_subscribers_;
        
};

