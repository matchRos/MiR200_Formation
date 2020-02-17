#include <controller/master.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"master");
    ros::NodeHandle formation_ns_("/formation");
    ros::NodeHandle robot_ns_("/"+std::string(argv[1]));
    ros::NodeHandle temp(formation_ns_.resolveName(std::string(argv[1])));
    ros::NodeHandle parameter_ns(temp.resolveName("controller"));  

   
    Master master=Master(argv[1],formation_ns_,robot_ns_,parameter_ns);
    master.setReference(atof(argv[2]),atof(argv[3]),0.0,atof(argv[4]));  

    ros::spin();
}