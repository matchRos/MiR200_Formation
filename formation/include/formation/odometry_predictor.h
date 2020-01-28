#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>


class OdometryPredictor{
    public:
        OdometryPredictor();
        OdometryPredictor(ros::NodeHandle &nh);
        OdometryPredictor(ros::NodeHandle &nh,std::string topic_name,tf::Transform initial_trafo);
        tf::Pose getPose();
    private:
        ros::NodeHandle nh_;
        ros::Subscriber odom_sub_;
        std::string topic_name_;        
        tf::Transform initial_trafo_;

        tf::Pose pose_;

        void callbackOdometry(const nav_msgs::OdometryConstPtr msg);    

};