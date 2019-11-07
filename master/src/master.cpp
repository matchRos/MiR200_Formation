#include <master/master.h>

Master::Master(ros::NodeHandle &nh):Controller(nh)
{
}
void Master::scope()
{
    this->msg_velocities_out=this->msg_velocities_in;
}