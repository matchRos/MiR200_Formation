#include <master/master.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"master");
    ros::NodeHandle nh;
    Master master=Master(nh);
    master.set_name(argv[1]);   
    master.link_input_velocity(argv[2]);
    master.link_output_velocity(argv[3]);    
    master.set_reference(atof(argv[4]),atof(argv[5]),0.0);

    ros::Rate rate(10);
    while(ros::ok())
    {
        master.scope();

        rate.sleep();
    }
}