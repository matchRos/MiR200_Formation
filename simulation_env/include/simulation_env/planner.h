#include<ros/ros.h>
#include<tf/tf.h>
#include<math.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_srvs/SetBool.h>

class Planner{
   
    public:
        Planner(ros::NodeHandle &nh);
        void start();
        void stop();
        void pause();
        void set_start_pose(tf::Pose pose);
        bool srv_start( std_srvs::SetBool::Request &req,std_srvs::SetBool::Response &res);
    private:
        ros::NodeHandle nh;        
        ros::Timer tim_sampling;
        ros::Publisher pub_current_pose;
        ros::ServiceServer set_start_service;
        bool is_planning;
        bool is_paused;

        std::string frame_name;

        ros::Duration paused;
        ros::Time start_time;

        tf::Pose planned_pose;
        tf::Transform start_reference;

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

class LissajousPlanner:public Planner{
    public:
        LissajousPlanner(ros::NodeHandle &nh);
        struct  LissajousPlan{
            float dphi;          //phase shif
            float omegax;       //frequenzy in x direction
            int ratio;          //ratio fy/fx
            float Ax;           //Magnitude x direction
            float Ay;            //Magnitude y directions
        };
        void set_parameter(float omegax,float dphi=0.0, int ratio=2,float Ax=1.0,float Ay=1.0);
        private:
            tf::Pose get_current_pose(ros::Duration time);
            tf::Pose old_pose;
            LissajousPlan plan;
};