#include <multi_robot_simulation/primitive_base.h>
Primitive::Primitive():start_point_(tf::Transform::getIdentity()),
                                         start_vel_(0.0),
                                         start_ang_vel_(0.0),
                                         time_(0.0)
{
}
std::vector<tf::Transform> Primitive::getPosition()
{
    return this->positions_;
}  
std::vector<tf::Vector3> Primitive::getVelocity()
{
    return this->velocities_;
}
std::vector<double> Primitive:: getAngularVelocity()
{
    return this->angular_velocities_;
}
std::vector<tf::Vector3> Primitive::getAccecleration()
{
    return this->accelerations_;
}
int Primitive::getTime()
{
    return this->time_;
}
int Primitive::getSize()
{
    return this->positions_.size();
}


