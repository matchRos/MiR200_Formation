#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <controller/controller.h>


/**
 * @brief Class that implements a master robot for multi robot formation control
 * 
 */
class Master:public Controller{
    public:
        /**
         * @brief Construct a new Master object
         * 
         * @param nh Nodehandle
         */
        Master(ros::NodeHandle &nh);
    private:
};