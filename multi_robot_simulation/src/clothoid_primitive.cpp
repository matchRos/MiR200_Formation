#include <multi_robot_simulation/clothoid_primitive.h>
#include<bits/stdc++.h> 
ClothoidPrimitive::ClothoidPrimitive(double accel,double radius):accel_(accel),
                                                                    radius_(radius),
                                                                    vel_max_(0.2),
                                                                    Primitive()
{
    
}
bool ClothoidPrimitive::interpolate(double linspace)
{
    tf::Vector3 position(this->start_point_.getOrigin());
    this->positions_.push_back(this->start_point_);
    
    double t=0.0;
    double R=0.0;
    double A=1.0;
    double L=std::pow(A,2)/this->radius_;
    
    double l=0.0;
    double kappa=0.0;  
    double vel=this->start_vel_;
    double phi=0.0;  
    int iterations=0;
    bool abort=false;
    bool vel_reached=false;
    while(!abort)
    {        
        
        double d_vel=this->accel_*linspace;
        if(vel+d_vel>this->vel_max_)
        {
            d_vel=this->vel_max_-vel;
            vel_reached=true;
        }
        double d_l=(vel+0.5*d_vel)*linspace;

        if(l+d_l>L)
        {
            d_l=L-l;
        }




        // tf::Vector3 d_pos=A*std::sqrt(M_PI)*tf::Vector3(std::cos(M_PI*std::pow(t,2)/2),
        //                                         std::sin(M_PI*std::pow(t,2)/2),
        //                                         0.0);
        
        
        tf::Vector3 d_pos=tf::Vector3(  std::cos(l*l/2.0/A/A),
                                        std::sin(l*l/2.0/A/A),
                                        0.0);
        


        ROS_INFO_STREAM("vel:"<<vel<<"\t"<<"l:"<<l<<"\t"<<"d_l_vel:"<<d_l<<"\t"<<"d_l:"<<d_pos.length());
        position+=d_pos;
        vel+=d_vel;
        l+=d_l;
        phi+=kappa;
       
        t+=linspace;
        R=A*A/l;
        if(R<=this->radius_)
        {
            abort=true;
        }
        kappa=std::sqrt(M_PI)*A/R;
        
        tf::Transform trafo(tf::createQuaternionFromYaw(phi),position);
        tf::Transform rot(trafo.getRotation(),tf::Vector3(0.0,0.0,0.0));

        this->positions_.push_back(trafo);
        this->velocities_.push_back(vel*(rot*tf::Vector3(1.0,0.0,0.0)));
        this->angular_velocities_.push_back(vel*kappa);
       
        
        
        
        iterations++;
        if(iterations>10000)
        {
            return false;
        }
        this->time_=t;
    }   
    return vel_reached;
    
}