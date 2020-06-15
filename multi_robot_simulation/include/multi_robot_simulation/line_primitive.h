#ifndef LINE_PRIMITIVE_H
#define LINE_PRIMITIVE_H
#include <multi_robot_simulation/primitive_base.h>
class LinePrimitive: public Primitive
{
    public:
        LinePrimitive(double accel,double vel_max,double length);
        double accel_;
        double vel_max_; 
        double length_;                    
        bool interpolate(double linspace);    
};
#endif