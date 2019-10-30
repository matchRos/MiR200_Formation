#include <master/master.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"master");
    ros::NodeHandle nh;
    Master master=Master(nh);
    master.set_name(argv[1]);
    master.link_input(argv[2]);
    master.link_output(argv[3]);    
    master.set_position(atof(argv[4]),atof(argv[5]),0.0);
    master.run();
}