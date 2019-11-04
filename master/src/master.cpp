#include <master/master.h>

Master::Master(ros::NodeHandle &nh):Slave(nh)
{

}
void Master::scope()
{
    this->msg_velocities_out=this->msg_velocities_in;
    this->output.publish(this->msg_velocities_out);

    ros::spinOnce();
}