#include <master/master.h>

Master::Master(ros::NodeHandle &nh):Controller(nh)
{
}
void Master::scope()
{
   switch(this->type)
    {
        case pseudo_inverse: this->msg_velocities_out=this->msg_velocities_in;break;
        case lypanov:this->calc_Lyapunov(0.3,0.3,0.3,0.3);break;
        default: break;
    }
}
