#pragma once

#include<ros/ros.h>
#include<simulation_env/formation.h>

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
