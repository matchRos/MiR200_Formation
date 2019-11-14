#include <master/master.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"master");
    ros::NodeHandle nh;
    
    
    Master master=Master(nh);
    master.set_name(argv[1]);   

    master.load();




    ros::Rate rate(50);
    while(ros::ok())
    {
        master.execute();    
        rate.sleep();
    }
}