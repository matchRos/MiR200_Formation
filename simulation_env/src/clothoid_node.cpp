#include <simulation_env/planner.h>


int main (int argc,char**argv)
{
    ros::init(argc,argv,"ClothoidPlanner");
    ros::NodeHandle nh;
    
    EulerPlanner euler(nh);
    ros::spin(); 
}
