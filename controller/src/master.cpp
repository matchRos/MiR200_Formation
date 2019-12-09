#include <controller/master.h>

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
            this->calc_Lyapunov(    this->lyapunov_parameter.kx,
                                    this->lyapunov_parameter.ky,
                                    this->lyapunov_parameter.ktheta,
                                    sqrt(lin_vel_in.x()*lin_vel_in.x()+lin_vel_in.y()*lin_vel_in.y()),
                                    this->ang_vel_in.z());
            break;
        case angle_distance: this->calc_angle_distance(this->kr,this->kang);break;
        default: 
            break;
    }
}
