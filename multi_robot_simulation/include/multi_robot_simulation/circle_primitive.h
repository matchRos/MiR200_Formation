#ifndef CIRCLE_PRIMITIVE_H
#define CIRCLE_PRIMITIVE_H
#include <multi_robot_simulation/primitive_base.h>
class CirclePrimitive: public Primitive
{
    public:
        /**
         * @brief Construct a new Circle Primitive object which plans an angular primitive with velocity trapezoid
         * 
         * @param accel linear acceleration used for planning 
         * @param vel_max maximum velcoity used for planning
         * @param radius radius of the circle to be planned
         * @param angle angle of the circle to be planned
         */
        CirclePrimitive(double accel, double vel_max, double radius, double angle);
        double angle_;
        double radius_;
        double accel_;
        double vel_max_;
        bool interpolate(double linspace);                  
};
#endif