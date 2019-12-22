#include <controller/master.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"master");
    ros::NodeHandle nh;   
    
    Master master=Master(nh);
    master.setName(argv[1]); 
    master.load();
    master.setReference(atof(argv[2]),atof(argv[3]),0.0,atof(argv[4]));  

    ros::spin();
}