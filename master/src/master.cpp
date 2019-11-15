#include <master/master.h>

Master::Master(ros::NodeHandle &nh):Controller(nh)
{
    this->kx=3;
    this->kphi=3;
    this->omegad=1.0;
    this->vd=1.0;

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
            this->calc_Lyapunov(this->kx,this->kphi,this->vd,this->omegad);
            break;
        default: 
            break;
    }
}
