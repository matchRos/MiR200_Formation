#include<ros/ros.h>
#include<tf/tf.h>
#include<math.h>
#include<geometry_msgs/PoseStamped.h>

class Planner{
   
    public:
        Planner(ros::NodeHandle &nh);
        void start();
        void stop();
        void pause();
       
    private:
        ros::NodeHandle nh;
        ros::Timer tim_sampling;
        ros::Publisher pub_current_pose;
        bool is_planning;
        bool is_paused;

        std::string frame_name;

        ros::Duration paused;
        ros::Time start_time;

        tf::Pose planned_pose;

        void plan(const ros::TimerEvent& events);
        virtual tf::Pose get_current_pose(ros::Duration time)=0;
        
};

class CirclePlanner:public Planner{
    public:
        CirclePlanner(ros::NodeHandle &nh);
        struct CirclePlan{
            float r;
            float x;
            float y;
            float omega;
        };
        void set_parameter(double r, double x,double y,double omega);
    private:
        tf::Pose get_current_pose(ros::Duration time);
        CirclePlan plan;
       
};

class EulerSpiralPlanner:public Planner{
    public:
        EulerSpiralPlanner(ros::NodeHandle &nh);
        struct EulerPlan{
            float l0; //Length of curve
            float r0; //radius at the end
        }
}