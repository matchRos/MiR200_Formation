#include<multi_robot_simulation/line_primitive.h>
LinePrimitive::LinePrimitive(double accel,double vel_max,double length):
                                                                                            accel_(accel),
                                                                                            vel_max_(vel_max),
                                                                                            length_(length),
                                                                                            Primitive()
{
    
}

bool LinePrimitive::interpolate(double linspace)
{
    this->positions_.clear();
    this->velocities_.clear();
    this->accelerations_.clear();
    this->angular_velocities_.clear();
   

    tf::Transform rot=tf::Transform(this->start_point_.getRotation(),tf::Vector3(0.0,0.0,0.0));
    this->accelerations_.push_back(this->accel_*(rot*tf::Vector3(1.0,0.0,0.0)));
    this->velocities_.push_back(this->start_vel_*(rot*tf::Vector3(1.0,0.0,0.0)));   
    this->positions_.push_back(this->start_point_);

    
    double length=0.0;
    double time=0.0;

    bool vel_reached=
    false;
    bool abort=false;
    int iterations=0;

    double vel=this->start_vel_;
    

    while(!abort && iterations<100000)                                                                                                                                                                                                         
    {   
        
        double d_vel=this->accel_*linspace;        
        if(vel+d_vel>this->vel_max_)
        {
            d_vel=this->vel_max_-vel;
            vel_reached=true;
        }
        
        double d_l=(vel+0.5*d_vel)*linspace;
        if(length+d_l>this->length_)
        {
            double scale=(this->length_-length)/d_l;
            d_l=scale*d_l;
            linspace=scale*linspace;
            abort=true;
        }
        vel+=d_vel;               
        length+=d_l;
        time+=linspace;

       
        this->accelerations_.push_back(this->accel_*(rot*tf::Vector3(1.0,0.0,0.0)));
        this->velocities_.push_back(vel*(rot*tf::Vector3(1.0,0.0,0.0)));        
        this->positions_.push_back(tf::Transform((this->start_point_).getRotation(),
                                                  this->start_point_*(length*(tf::Vector3(1.0,0.0,0.0)))));


       
        iterations++;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    }
    for(int i=0;i<this->velocities_.size();i++)
    {
        this->angular_velocities_.push_back(0.0);
    }
    this->time_=time;
    return vel_reached;                                                                                                                                                                                                                                                                                                                                                                                                                                                 
}
