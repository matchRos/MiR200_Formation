#include <ros/ros.h>
#include <tf/tf.h>
#include <multi_robot_msgs/Formation.h>


/**
 * @brief A Class for handeling basic mobile Robot formation work. Properly adds Robots to a 
 * formation and provides basic interpretation tools as the adjacency matrix of the formation.
 * 
 */
class Formation{
    public:
        typedef Formation Transformation;
        /**
         * @brief Construct a new Formation object
         * 
         */
        Formation();
        /**
         * @brief Adds another Robot to the formation
         * 
         * @param pose Pose of the robot to be added
         * @param neighbours Neighbours of the Robot if they should be linked (optional)
         */
        void addRobot(tf::Pose pose,std::vector<int> neighbours=std::vector<int>());
        /**
         * @brief Gets the number of Robots within the formation
         * 
         * @return int Number of Robots
         */
        int size();
        /**
         * @brief Calculates the transformations from one Formation to another
         * 
         * @param c1 Formation reference
         * @param c2 Foramtion target
         * @return std::vector<tf::Transform> Transformation vector that contains the transformation for every single robot in the formation
         */
        std::vector<tf::Pose> getPoses();
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
         * @brief Modifies the pose of the ith robot within the Formation
         * 
         * @param i Index of the robot to be modified
         * @param pose modified Pose of the robot
         */
        void modifiePose(int i,tf::Pose pose);
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
        std::vector<tf::Pose> formation_;
        std::vector<std::vector<double> > adjacency_;
        std::vector<std::vector<bool> > connectivity_;

};


class FormationPublisher{
    public:
        FormationPublisher();
        FormationPublisher(ros::NodeHandle nh, std::string topic);
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
        ros::Publisher publisher_;

};