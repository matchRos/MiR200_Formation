#ifndef CLOTHOID_H
#define CLOTHOID_H

#include <multi_robot_simulation/primitive_base.h>
class ClothoidPrimitive: public Primitive
{    
    public:
        /**
         * @brief Construct a new Clothoid Primitive object 
         * 
         * @param accel angular acceleration for planning 
         * @param radius Target Radius the clothoid ends in
         */
        ClothoidPrimitive(double accel,double radius);
        bool interpolate(double linspace);
        double vel_max_;
        double accel_;
        double radius_;

};

#endif