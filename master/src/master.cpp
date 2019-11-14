#include <master/master.h>

Master::Master(ros::NodeHandle &nh):Controller(nh)
{
}
void Master::scope()
{
   switch(this->type)
    {
        case pseudo_inverse: 
            this->lin_vel_out=this->lin_vel_in;
            this->ang_vel_out=this->ang_vel_in;
            break;
        case lypanov:
            this->calc_Lyapunov(0.3,0.3,0.3,0.3);
            break;
        default: 
            break;
    }
}
