#ifndef PRIMITVE_H
#define PRIMITVE_H

#include <ros/ros.h>
#include <tf/tf.h>
class Primitive
{
    public:
        Primitive();
        
        tf::Transform start_point_;
        double start_vel_;
        double start_ang_vel_;
        double time_;

        std::vector<tf::Transform> positions_;
        std::vector<tf::Vector3> velocities_;
        std::vector<double> angular_velocities_;
        std::vector<tf::Vector3> accelerations_;     
        
            
        std::vector<tf::Transform> getPosition();    
        std::vector<tf::Vector3> getVelocity();  
        std::vector<double> getAngularVelocity();
        std::vector<tf::Vector3> getAccecleration();
        
        int getTime();
        int getSize(); 

        virtual bool interpolate(double linspace)=0;      
};
#endif