#include <master/master.h>

Master::Master(ros::NodeHandle &nh):Slave(nh)
{

}


void Master::input_callback(geometry_msgs::Twist msg)
{
    this->control.v=msg.linear.x;
    this->control.omega=msg.angular.z;
}  
