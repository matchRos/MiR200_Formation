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
        //Load nessecarry parameter from ros parameter server
        virtual void load()=0;

    protected:
        int iterations_counter;
        ros::NodeHandle nh;   
        
    private:
             
        ros::Timer tim_sampling;                                //Timer for publishing trajectory
        ros::Publisher pub_current_pose;                        //Publisher for current trajectory pose
        ros::ServiceServer set_start_service;                   //Service for starting the planner
        ros::ServiceServer set_stop_service;                    //Service for shutting the planner down
        bool is_planning;                                       //Flag if the planner is planning at the moment
        bool is_paused;                                         //Flag if the planner is paused at the moment

        int iterations;                                         //Number of Iterations
        

        std::string frame_name;                                 //Frame in wich the pose is calculated

        ros::Duration paused;                                   //Time the planner paused
        ros::Time start_time;                                   //Time at wich the planning procedure started

        tf::Pose planned_pose;                                  //Calculated current poses
        tf::Transform start_reference;                          //Transformation for modifieing offset to initial pose

        //Planning Scope as handle for timer event
        void plan(const ros::TimerEvent& events);      
        //Determine the current pose
        //@param time : time the pose is determined at       
        virtual tf::Pose get_current_pose(ros::Duration time)=0; 
        //service procedure for satrting the planner
        //@param req: Reqest the service is getting
        //@oparam res Response the service is sending
        bool srv_start( std_srvs::SetBool::Request &req,std_srvs::SetBool::Response &res);
        //service procedure for stopping the planner
        //@param req: Reqest the service is getting
        //@oparam res Response the service is sending
        bool srv_stop( std_srvs::SetBool::Request &req,std_srvs::SetBool::Response &res);
      
        
};


//CIRCLE############################################################################################################################################
//##################################################################################################################################################
#define PARAM_R "radius"
#define PARAM_OMEGA "omega"
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
        void load();
    private:
        //Calclulation of the curren pose dependent on time
        tf::Pose get_current_pose(ros::Duration time);
        CirclePlan plan;
       
};

//LISSAJOUS############################################################################################################################################
//##################################################################################################################################################

#define PARAM_OMEGA "omega"
#define PARAM_RATIO "ratio"
#define PARAM_PHASE "phaseshift"
#define PARAM_AMP_X "amplifier_x"
#define PARAM_AMP_Y "amplifier_y"

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
        void load();
        private:
            //Calclulation of the curren pose dependent on time
            tf::Pose get_current_pose(ros::Duration time);
            LissajousPlan plan;
};