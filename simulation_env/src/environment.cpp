#include <simulation_env/environment.h>

Environment::Environment(ros::NodeHandle &nh):nh(nh)
{

}

void Environment::scope()
{
    

}

template <class T> void Environment::call_services(std::string srv,T service)
{
    std::list<std::string>::iterator it;
    for(it=this->list_controller.begin();it!=this->list_controller.end();it++)
    {
        ros::ServiceClient client = this->nh.serviceClient<T>(*it+srv);     
        req.data=value;   
        client.call(service::Request);
    }
}