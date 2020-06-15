#ifndef PRIMITIVE_PLANNER_H
#define PRIMITIVE_PLANNER_H

#include <multi_robot_simulation/planner.h>
#include <multi_robot_simulation/circle_primitive.h>
#include <multi_robot_simulation/line_primitive.h>
#include <multi_robot_simulation/clothoid_primitive.h>

//Primitive############################################################################################################################################
//###################################################################################################################################################
class PrimitivePlanner:public Planner{
    public:
        PrimitivePlanner(ros::NodeHandle &nh);
        ~PrimitivePlanner();

        struct PrimitivePlan
        {  
            double ang_acc_lim_;
            double trans_acc__lim_;
            double ang_vel_lim_;
            double trans_vel__lim_;
            std::vector<Primitive> primitives_;
            PrimitivePlan()
            {;}
        };

        void loadChild();
    private:
        Primitive* current_primitive_;
        std::list<Primitive*>::iterator current_it_;
        std::list<Primitive*> list_of_primitives_;
        PrimitivePlan plan;
        int index_of_primitive_;
        double time_offset_;

        bool checkCompatibility(std::list<Primitive*> list);

        int lookupIndexTime(double time);
        
        tf::Vector3 get_position(ros::Duration time);

        tf::Quaternion get_orientation(ros::Duration time);

        tf::Vector3 get_velocity(ros::Duration time);

        double get_angular_velocity(ros::Duration time);

        tf::Vector3 get_acceleration(ros::Duration time);

        void check_period(ros::Duration time);     
};

#endif