#include <ros/ros.h>
#include <tf/tf.h>
#include <multi_robot_msgs/Formation.h>


class Formation{
    public:
        /**
         * @brief Construct a new Formation object
         * 
         */
        Formation();
        /**
         * @brief Converts a Formation object to a Formation message type
         * 
         * @param formation Formation that should be converted
         * @param msg Reference to the message storage
         */
        static void Formation2Msg(std::vector<tf::Pose> formation,multi_robot_msgs::Formation &msg);
        /**
         * @brief Adds another robot to the formation
         * 
         * @param pose pose of the robot given in the formation refernce frame
         */
        void addRobot(tf::Pose pose);
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
        friend std::vector<tf::Transform> operator-(Formation &c1, Formation &c2);

        /**
         * @brief Get robot poses within the formation
         * 
         * @return std::vector<tf::Pose> Vector of the robot poses
         */
        std::vector<tf::Pose> getPoses();
    private:
        std::vector<tf::Pose> formation_;

};


class FormationPublisher{
    public:
        FormationPublisher(ros::NodeHandle nh, std::string topic);
        void publish(Formation formation);
    private:
        std::string topic_name_;    
        ros::NodeHandle nh_;
        ros::Publisher publisher_;

};