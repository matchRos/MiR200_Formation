#include<controller/lyapunov.h>


LyapunovController::LyapunovController(ros::NodeHandle nh):Controller(nh)
{

}
LyapunovController::~LyapunovController()
{

}

LyapunovController::Velocities LyapunovController::calculate(tf::Transform difference,LyapunovController::Velocities desired)
{
    LyapunovController::Velocities velocities;
   
    double x=difference.getOrigin().getX();
    double y=difference.getOrigin().getY();
    double phi=tf::getYaw(difference.getRotation());

    

    velocities.vd=(this->param_.kx*x+vd*cos(phi));    
    velocities.omega=this->param_.kphi*sin(phi)+this->param_.ky*vd*y+omegad); 
    
    return velocities;
}