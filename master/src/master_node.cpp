#include <master/master.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"master");
    ros::NodeHandle nh;
    
    
    Master master=Master(nh);
    master.set_name(argv[1]); 
    master.load();
<<<<<<< HEAD
    master.set_reference(atof(argv[2]),atof(argv[3]),0.0,atof(argv[4]));  
=======
    master.set_reference(atof(argv[2]),atof(argv[3]),0.0);  
>>>>>>> 47fe403a204362736da9611652b746f3fcc8d0f2




    ros::Rate rate(50);
    while(ros::ok())
    {
        master.execute();    
        rate.sleep();
    }
}