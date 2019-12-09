#include <ros/ros.h>
#include <std_srvs/SetBool.h>

class Environment{
    public:
        Environment(ros::NodeHandle &nh);
        void start_simulation();

    private:
        void  scope();
        std::list<std::string> list_controller;
        ros::NodeHandle nh;
        template <class T> void call_services(std::string srv,T service);
        
}