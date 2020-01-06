#include <controller/slave.h>

int main (int argc,char** argv)
{
    ros::init(argc,argv,"slave");
    ros::NodeHandle nh;
    Slave slave=Slave(nh);
    slave.setName(argv[1]);  
    slave.load();
    slave.setReference(atof(argv[2]),atof(argv[3]),0.0,atof(argv[4]));  
    ros::spin();
}