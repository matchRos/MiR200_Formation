#include<multi_robot_simulation/primitive_planner.h>

//Implementation of a StepResponse planner that gives a spiral########################################################################################
//#######################################################################################################################################

PrimitivePlanner::PrimitivePlanner(ros::NodeHandle &nh):Planner(nh),time_offset_(0.0),index_of_primitive_(0)
{
    this->list_of_primitives_.push_back(new LinePrimitive(0.1,0.5,2));
    this->list_of_primitives_.push_back(new ClothoidPrimitive(1.0,3.0));
    this->list_of_primitives_.push_back(new CirclePrimitive(0.1,0.5,3,1.57));    
    // this->list_of_primitives_.push_back(new LinePrimitive(0.1,0.5,4));
    // this->list_of_primitives_.push_back(new CirclePrimitive(0.1,0.5,3,1.57));    
    // this->list_of_primitives_.push_back(new LinePrimitive(0.1,0.5,4));
    // this->list_of_primitives_.push_back(new CirclePrimitive(0.1,0.5,3,1.57));    
    // this->list_of_primitives_.push_back(new LinePrimitive(0.1,0.5,4));
    // this->list_of_primitives_.push_back(new CirclePrimitive(0.1,0.5,3,1.57));   

    this->current_it_=this->list_of_primitives_.begin();
    this->current_primitive_=*(this->current_it_);
    
    if(!this->checkCompatibility(this->list_of_primitives_))
    {
        ROS_WARN("Incompatibility in primitive list occured!");
    }
}
PrimitivePlanner::~PrimitivePlanner()
{
    delete this->current_primitive_;
    this->list_of_primitives_.clear();
}   
void PrimitivePlanner::loadChild()
{
}
tf::Vector3 PrimitivePlanner::get_position(ros::Duration time)
{
    std::vector<tf::Transform> pos=this->current_primitive_->getPosition();
    
    return pos.at(this->lookupIndexTime(time.toSec())).getOrigin();
}
tf::Quaternion PrimitivePlanner::get_orientation(ros::Duration time)
{
    std::vector<tf::Transform> pos=this->current_primitive_->getPosition();
  
    return pos.at(this->lookupIndexTime(time.toSec())).getRotation();
}
tf::Vector3 PrimitivePlanner::get_velocity(ros::Duration time)
{
    std::vector<tf::Vector3> vel=this->current_primitive_->getVelocity();

    return vel.at(this->lookupIndexTime(time.toSec()));
}
double PrimitivePlanner::get_angular_velocity(ros::Duration time)
{
    std::vector<double> vel=this->current_primitive_->getAngularVelocity();
  
    return vel.at(this->lookupIndexTime(time.toSec()));
}
tf::Vector3 PrimitivePlanner::get_acceleration(ros::Duration time)
{

}

int PrimitivePlanner::lookupIndexTime(double time)
{
    double complete_time=this->current_primitive_->getTime();
    int index=(int)floor((time-this->time_offset_)/complete_time*this->current_primitive_->getSize());
    if(index<0)
    {
        ROS_WARN("Index below zero!");
        index=0;
    }
    if(index>=this->current_primitive_->getSize())
    {
        index=this->current_primitive_->getSize()-1;
        ROS_WARN("Reached end of primitive!");
    }
    return index;
}
bool PrimitivePlanner::checkCompatibility(std::list<Primitive*> list)
{
    list.front()->interpolate(0.01);
    tf::Transform last_position=list.front()->getPosition().back();
    double last_vel=list.front()->getVelocity().back().length();
    bool succeed=true;

    for(auto element=list.begin()++;element!=list.end();element++)
    {
        (*element)->start_vel_=last_vel;
        (*element)->start_point_=last_position;
        if(!(*element)->interpolate(0.01))
        {
            succeed=false;
        }
        last_position=(*element)->getPosition().back();
        last_vel=(*element)->getVelocity().back().length();
    }     
    return succeed;
}
void PrimitivePlanner::check_period(ros::Duration time)
{
    if(time.toSec()>=this->current_primitive_->getTime()+this->time_offset_)
    {
       
       if(this->current_it_!=this->list_of_primitives_.end())
       {
           ROS_INFO_STREAM("Changing primitive!");
           this->time_offset_+=this->current_primitive_->getTime();
           this->current_it_++;
           this->current_primitive_=*(this->current_it_);
       }
       else
       {
           this->stop();
       }
       
    }
}





