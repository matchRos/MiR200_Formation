#include<multi_robot_simulation/circle_primitive.h>
CirclePrimitive::CirclePrimitive(double accel, double vel_max, double radius, double angle):
                                                     angle_(angle),
                                                     radius_(radius),
                                                     accel_(accel),
                                                     vel_max_(vel_max)
{
    
}
bool CirclePrimitive::interpolate(double linspace)
{
    this->positions_.clear();
    this->velocities_.clear();
    this->accelerations_.clear();
    this->angular_velocities_.clear();
    
    tf::Transform rot=tf::Transform(tf::createQuaternionFromYaw(0)*this->start_point_.getRotation(),tf::Vector3(0.0,0.0,0.0));
    this->velocities_.push_back(this->start_vel_*(rot*tf::Vector3(1.0,0.0,0.0)));
    this->accelerations_.push_back(this->accel_*(rot*tf::Vector3(1.0,0.0,0.0)));    
    this->positions_.push_back(tf::Transform((this->start_point_*rot).getRotation(),
                                                this->start_point_.getOrigin()));

    
    double length=0.0;
    double time=0.0;

    bool vel_reached=false;
    bool abort=false;
    int iterations=0;

    double ang_acc=this->accel_/this->radius_;
    double omega=this->start_vel_/this->radius_;
    double omega_max=this->vel_max_/this->radius_;


    while(!abort && iterations<100000)                                                                                                                                                                                                         
    {   
        double d_omega=ang_acc*linspace;        
        if(omega+d_omega>omega_max)
        {
            d_omega=omega_max-omega;
            vel_reached=true;
        }
        
        double d_l=(omega+0.5*d_omega)*linspace;
        if(length+d_l>this->angle_)
        {
            double scale=(this->angle_-length)/d_l;
            d_l=scale*d_l;
            linspace=scale*linspace;
            abort=true;
        }
        omega+=d_omega;               
        length+=d_l;
       
        rot.setRotation(this->start_point_.getRotation()*tf::createQuaternionFromYaw(length));
        tf::Vector3 d_pos=d_l*this->radius_*(rot*tf::Vector3(1.0,0.0,0.0));
        
        this->accelerations_.push_back(ang_acc*this->radius_*(rot*tf::Vector3(1.0,0.0,0.0)));
        this->velocities_.push_back(omega*this->radius_*(rot*tf::Vector3(1.0,0.0,0.0)));        
        this->positions_.push_back(tf::Transform(rot.getRotation(),
                                            this->positions_.back().getOrigin()+d_pos));
       
        time+=linspace;
        iterations++;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    }
    for(int i=0;i<this->velocities_.size();i++)
    {
        this->angular_velocities_.push_back(omega);
    }
    this->time_=time;
    return vel_reached;                                                                                                                                                                                                                                                                                                                                                                                                                        
}