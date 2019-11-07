#include <slave/slave.h>

int main (int argc,char** argv)
{
    ros::init(argc,argv,"slave");
    ros::NodeHandle nh;
    Slave slave=Slave(nh);
    slave.set_name(argv[1]);   
    slave.set_reference(0.0,0.0,0.0);

    slave.load();
    slave.set_type(Slave::pseudo_inverse);
    ros::Rate rate(10);
    while(ros::ok())
    {       
        slave.execute();      
        rate.sleep();
    }
 
}