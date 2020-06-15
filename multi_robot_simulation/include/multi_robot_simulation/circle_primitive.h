#ifndef CIRCLE_PRIMITIVE_H
#define CIRCLE_PRIMITIVE_H
#include <multi_robot_simulation/primitive_base.h>
class CirclePrimitive: public Primitive
{
    public:
        CirclePrimitive(double accel, double vel_max, double radius, double angle);
        double angle_;
        double radius_;
        double accel_;
        double vel_max_;
        bool interpolate(double linspace);                  
};
#endif