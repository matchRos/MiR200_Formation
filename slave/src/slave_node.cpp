#include <slave/slave.h>

int main (int argc,char** argv)
{
    ros::init(argc,argv,"slave");
    ros::NodeHandle nh;
    Slave slave=Slave(nh);
    slave.set_name(argv[1]);
    slave.link_input_velocity(argv[2]);
    slave.link_output_velocity(argv[3]);    
    slave.set_reference(atof(argv[4]),atof(argv[5]),0.0);

    ros::Rate rate(10);
    while(ros::ok())
    {
        slave.scope();
        rate.sleep();
    }
 
}