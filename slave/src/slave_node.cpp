#include <slave/slave.h>

int main (int argc,char** argv)
{
    ros::init(argc,argv,"slave");
    ros::NodeHandle nh;
    Slave slave=Slave(nh);
    slave.set_name(argv[1]);   

    slave.load();
    ros::Rate rate(100);
    while(ros::ok())
    {       
        slave.execute();      
        rate.sleep();
    }
 
}