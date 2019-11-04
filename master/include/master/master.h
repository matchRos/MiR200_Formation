#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <slave/slave.h>

class Master:public Slave{
    public:
        Master(ros::NodeHandle &nh);        
        void scope();
    private:
};