#include <multi_robot_controller/slave.h>

int main (int argc,char** argv)
{
    ros::init(argc,argv,"slave");
    ros::NodeHandle node_;
    ros::NodeHandle robot_ns_("/"+std::string(argv[1]));
    ros::NodeHandle parameter_ns(robot_ns_.resolveName("controller"));

   
    Slave slave=Slave(argv[1],node_,robot_ns_,parameter_ns);
    ros::spin();
}