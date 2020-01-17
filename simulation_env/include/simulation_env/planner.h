#include<ros/ros.h>
#include<tf/tf.h>
#include<math.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>
#include<std_srvs/Empty.h>

//Generic path planning class
#define PARAM_ITERATION "iterations"

/**
 * @brief A generic planner class that provides general interface functions and architecture
 * 
 */
class Planner{
   
    public:
        /**
         * @brief Construct a new Planner object
         * 
         * @param nh Ros Nodehandle
         */
        Planner(ros::NodeHandle &nh);
        /**
         * @brief Starts the planner
         * 
         */
        void start();
        /**
         * @brief Stops the planner
         * 
         */
        void stop();
        /**
         * @brief Pauses the planner
         * 
         */
        void pause();
        //Sets the initial pose of the robot from wich the planning starts
        void set_start_pose(tf::Pose pose);
        //Load nessecarry parameter from ros parameter server
        virtual void load()=0;

    protected:
        int iterations_counter;                                 ///Counter for the iterations of periodic planner functions
        ros::NodeHandle nh;
        int iterations;        
        tf::Transform start_reference;                          //Transformation for modifieing offset to initial pose
    private:
         
        ros::Timer tim_sampling;                                //Timer for publishing trajectory
        ros::Publisher pub_current_odometry;

        ros::ServiceServer set_start_service;                   //Service for starting the planner
        ros::ServiceServer set_stop_service;                    //Service for shutting the planner down

        bool is_planning;                                       //Flag if the planner is planning at the moment
        bool is_paused;                                         //Flag if the planner is paused at the moment

                                              //Number of Iterations to do

        std::string frame_name;                                 //Frame in wich the pose is calculated

        ros::Duration paused;                                   //Time the planner paused
        ros::Time start_time;                                   //Time at wich the planning procedure started

        tf::Pose start_pose;
        tf::Vector3 planned_vel;

        tf::Vector3 vel;
        tf::Vector3 pos;
        tf::Quaternion orientation;
        double ang_vel;
        
   

        //Planning Scope as handle for timer event
        void plan(const ros::TimerEvent& events);
        void publish();

        ///Gets the transformation form the given pose to the pose at timestamp zero
        ///@param pose : Pose from wich the planner should start
        tf::Transform get_transform(tf::Pose pose); 

        ///Transforming all values that are not rotation or translation invariant
        ///@param trafo : The Transformation applied to all values
        void transform_values(tf::Transform trafo);       

        ///service procedure for satrting the planner
        ///@param req : Reqest the service is getting
        ///@param res : Response the service is sending
        bool srv_start( std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
        ///service procedure for stopping the planner
        ///@param req : Reqest the service is getting
        ///@param res : Response the service is sending
        bool srv_stop( std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);

        

        //Virtual function to overload by child classes####################################  
        //Determines the velocity at a specific time
        //@param time       
        virtual tf::Vector3 get_position(ros::Duration time)=0;
        virtual tf::Quaternion get_orientation(ros::Duration time)=0;
        virtual tf::Vector3 get_velocity(ros::Duration time)=0;
        virtual double get_angular_velocity(ros::Duration time)=0;
        virtual void check_period(ros::Duration time)=0;         
        
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
        CirclePlan plan;
        //Calclulation of the curren pose dependent on time
        //Calclulation of the curren pose dependent on time
        tf::Vector3 get_position(ros::Duration time);
        tf::Quaternion get_orientation(ros::Duration time);
        tf::Vector3 get_velocity(ros::Duration time);
        double get_angular_velocity(ros::Duration time);
        void check_period(ros::Duration time);            
        
       
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
        LissajousPlan plan;
        //Calclulation of the curren pose dependent on time
        tf::Vector3 get_position(ros::Duration time);
        tf::Quaternion get_orientation(ros::Duration time);
        tf::Vector3 get_velocity(ros::Duration time);
        double get_angular_velocity(ros::Duration time);
        void check_period(ros::Duration time);           
};



//ClickedPose############################################################################################################################################
//##################################################################################################################################################

class ClickedPosePlanner:public Planner{
    
    public:
        ClickedPosePlanner(ros::NodeHandle &nh,std::string topic_name);
        tf::Vector3 get_position(ros::Duration time);
        tf::Quaternion get_orientation(ros::Duration time);
        tf::Vector3 get_velocity(ros::Duration time);
        double get_angular_velocity(ros::Duration time);
        void check_period(ros::Duration time);   

    private:       
        ros::Subscriber sub_;
        tf::Pose pose_;
        tf::Vector3 direction_;
        void clickedCallback(const geometry_msgs::PoseStampedConstPtr msg);
        void load();


};


//Euler############################################################################################################################################
//##################################################################################################################################################
class EulerPlanner:public Planner{
    public:
        EulerPlanner(ros::NodeHandle &nh);
               
        void load();
    
    private:
        double omega;
        double radius_; 
        std::vector<double> old_value_;  
        //Calclulation of the curren pose dependent on time
        tf::Vector3 get_position(ros::Duration time);
        tf::Quaternion get_orientation(ros::Duration time);
        tf::Vector3 get_velocity(ros::Duration time);
        double get_angular_velocity(ros::Duration time);
        void check_period(ros::Duration time);   

        std::vector<double> clothoid(int order,double L,double R);
        unsigned int fakultaet(unsigned int zahl);    
};

