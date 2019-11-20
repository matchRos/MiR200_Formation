#include <master/master.h>

Master::Master(ros::NodeHandle &nh):Controller(nh)
{
}
void Master::scope()
{
   switch(this->type)
    {
        case pseudo_inverse: 
            this->lin_vel_out=lin_vel_in;
            this->ang_vel_out=this->ang_vel_in;
            break;
        case lypanov:
            this->calc_Lyapunov(this->kx,this->ky,this->kphi,this->vd,this->omegad);
            break;
        case angle_distance: this->calc_angle_distance(this->kr,this->kang);break;
        default: 
            break;
    }
}
