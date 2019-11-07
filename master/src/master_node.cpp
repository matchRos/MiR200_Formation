#include <master/master.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"master");
    ros::NodeHandle nh;


    Master master=Master(nh);
    master.set_name(argv[1]);
    master.link_state(nh.getNamespace()+"/state");   
    master.link_input_velocity("/key_vel");
    master.link_output_velocity(nh.getNamespace()+"/mobile_base_controller/cmd_vel");      
    master.set_reference(0.0,0.0,0.0);





    ros::Rate rate(10);
    while(ros::ok())
    {
        master.execute();    
        rate.sleep();
    }
}