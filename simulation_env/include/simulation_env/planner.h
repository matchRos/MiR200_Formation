#include<ros/ros.h>
#include<tf/tf.h>
#include<math.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_srvs/SetBool.h>

//Generic path planning class
class Planner{
   
    public:
        Planner(ros::NodeHandle &nh);
        //Start planning
        void start();
        //Stop planning and shutdown
        void stop();
        //Pause planning
        void pause();
        //Sets the initial pose of the robot from wich the planning starts
        void set_start_pose(tf::Pose pose);
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
        bool srv_start( std_srvs::SetBool::Request &req,std_srvs::SetBool::Response &res);
      
        
};
//Planner Class for generating a circle path in map frame. 
class CirclePlanner:public Planner{
    public:
        CirclePlanner(ros::NodeHandle &nh);
        struct CirclePlan{
            float r;           
            float omega;
        };
        //Sets the parameter of the planner
        void set_parameter(double r=3.0,double omega=0.5);
    private:
        //Calclulation of the curren pose dependent on time
        tf::Pose get_current_pose(ros::Duration time);
        CirclePlan plan;
       
};

//Planner Class for generating a lissajous figure path in map frame. 
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
        void set_parameter(float omegax,float dphi=0.0, int ratio=2,float Ax=3.0,float Ay=3.0);
        private:
            //Calclulation of the curren pose dependent on time
            tf::Pose get_current_pose(ros::Duration time);
            LissajousPlan plan;
};