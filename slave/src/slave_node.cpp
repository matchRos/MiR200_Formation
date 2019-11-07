#include <slave/slave.h>

int main (int argc,char** argv)
{
    ros::init(argc,argv,"slave");
    ros::NodeHandle nh;
    Slave slave=Slave(nh);
    slave.set_name(argv[1]);
    slave.link_state(nh.getNamespace()+"/state");
    slave.link_input_velocity("/"+std::string(argv[2])+"/mobile_base_controller/cmd_vel");
    slave.link_output_velocity(nh.getNamespace()+"/mobile_base_controller/cmd_vel");    
    
    slave.set_reference(atof(argv[3]),atof(argv[4]),0.0);
    slave.set_type(Slave::pseudo_inverse);

    ros::Rate rate(10);
    while(ros::ok())
    {       
        slave.execute();      
        rate.sleep();
    }
 
}