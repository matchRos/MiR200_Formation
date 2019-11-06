#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <controller/controller.h>

class Master:public Controller{
    public:
        Master(ros::NodeHandle &nh);        
        void scope();
    private:
};