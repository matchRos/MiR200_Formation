#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_srvs/Empty.h>
#include <multi_robot_msgs/ControlData.h>
#include <multi_robot_msgs/ControlState.h>
#include <multi_robot_msgs/ControlVector.h>
#include <multi_robot_msgs/ControlDifference.h>
#include <multi_robot_msgs/SetInitialPose.h>

#include <math.h>
#include <stdio.h>


#define PARAM_PUBISH_TF "publish_tf"

#define PARAM_REFERENCE_POSE "reference"
#define PARAM_TARGET_VEL "topic_target_velocity"
#define PARAM_TARGET_STATE "topic_target_state"
#define PARAM_TARGET_ODOM "topic_target_odometry"
#define PARAM_CURRENT_ODOM "topic_current_odometry"
#define PARAM_CURRENT_STATE "topic_current_state"


#define PARAM_COORD "coord"
#define PARAM_WORLD_FRAME "world_frame"
#define PARAM_TYPE "controller_type"
#define PARAM_THRESHOLD "thresh"

#define PARAM_LYAPUNOV "lyapunov"
#define PARAM_ANG_DIST "angle_distance"


/**
 * @brief Provides a easy to use implementation of a generic mobile robot controller.
 */
class Controller{
    public:
        typedef tf::Vector3 VelocityCartesian;                      ///<Defines VelocityCartesian wich is used to stores cartesian defined velocities
        typedef tf::Vector3 PositionCartesian;                      ///<Defines PositionCartesian wich is used to store a cartesian defined position
        typedef tf::Transform ControlDifference;                    ///<Defines ControlDifference wich is a transformation from the current state in the target state

        /**
         * @brief Specifies the different implemented control laws
         * 
         */
        enum ControllerType
        {
            disable=0,
            pseudo_inverse=1,   /**< Control law is based on pseudo inveres the input for generate a output (least squares optimisation) */
            lypanov=2,          /**< Control law is based on the lyapunov approache. Output is determined from a lyapunov stable function */
            angle_distance=3,    /**< Control law based on linear approach in angle and distance respectively */
        };
        
        /**
         * @brief Specifies the parameters needed for the lyapunov based control law
         * 
         */
        struct LyapunovParameter
        {
            float kx;           ///< Control gain in x-direction 
            float ky;           ///< Control gain in y-direction  
            float kphi;         ///< Control gain in theta-direction

            LyapunovParameter(){kx=0.0;ky=0.0;kphi=0.0;};
            LyapunovParameter(float kx,float ky,float kphi){this->kx=kx;this->ky=ky;this->kphi=kphi;}
            LyapunovParameter(std::vector<float> param)
            {
                if(param.size()!=3)
                {
                    throw std::invalid_argument("Wrong number of LyapunovParameters");
                }
                else
                {
                    this->kx=param[0];
                    this->ky=param[1];
                    this->kphi=param[2];
                }
            }
                
        };

        /**
         * @brief Specifies the Parameters of the angle distance based control law
         * 
         */
        struct AngleDistanceParameter
        {
            float angular_gain; ///< Control gain in phi direction  
            float linear_gain;  ///< Control gain in l direction   
            float d; 

            AngleDistanceParameter(){angular_gain=0.0;linear_gain=0.0;d=0.0;};
            AngleDistanceParameter(float angular_gain,float linear_gain,float d){this->angular_gain=angular_gain;this->linear_gain=linear_gain;this->d=d;}
            AngleDistanceParameter(std::vector<float> param)
            {
                if(param.size()!=3)
                {
                    throw std::invalid_argument("Wrong number of AngleDistanceParameters");
                }
                else
                {
                    this->angular_gain=param[0];
                    this->linear_gain=param[1];
                    this->d=param[2];
                }
            }
        };

        /**
         * @brief A struct that holds linear and angular velocity of the Robot since this are the control variable for kinematic tracking control
         * 
         */
        struct ControlVector
        {
            double v;                   ///<Linear velocity
            double omega;               ///<Angular velocity
            ControlVector(){v=0.0;omega=0.0;}
        };
        typedef ControlVector VelocityEulerian;                     ///<Defines VelocityEulerian wich is used to store Velocities wich are defined locally in the moved base system

        /**
         * @brief Holds the System state combined by the pose of the system, the cartesian velocity and the angular velocity
         * 
         */
        struct ControlState
        {
            tf::Pose pose;                  ///<Complete pose of the robot in 3 dimensional space
            VelocityCartesian velocity;     ///<Velocity in cartesian space
            double angular_velocity;        ///<Angular velocity around z-axis
            ControlState()
            {
                pose=tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0.0,0.0,0.0));
                velocity=tf::Vector3(0.0,0.0,0.0);
                angular_velocity=0.0;
            }
        };


         /*Konstruktor and Destruktor###################################################################################################################
        ##################################################################################################################################################*/
       

         /**
         * @brief Construct a new Controller object
         * 
         * @param nh Ros nodehandle for managing namespaces and ros functionality within the controller object
         */
        Controller( std::string name,
                    ros::NodeHandle nh=ros::NodeHandle("~"),
                    ros::NodeHandle nh_topics=ros::NodeHandle("~"),
                    ros::NodeHandle nh_parameters=ros::NodeHandle("~"));
        /**
         * @brief Destroy the Controller object
         * 
         */
        ~Controller();


       
        
        /*Setter and parameter methods ###################################################################################################################
        ##################################################################################################################################################*/
        
        /**
         * @brief Set the name of object
         * 
         * @param name Name to be set
         */
        void setName(std::string name);       


        /**
         * @brief Set the world frame name all references are given in
         * 
         * @param frame Name of the world frame
         */
        void setWorldFrame(std::string frame);

        /**
         * @brief Set the Reference pose
         * 
         * @param pose pose to be set
         */
        void setReferenceFrame(tf::Pose pose);          

        /**
         * @brief Sets the type of the used control law
         * 
         * @param type Type of the controller as defined in controllerType
         */
        void setType(Controller::ControllerType type);

        /**
         * @brief Set the Control Parameter object
         * 
         * @param param Lyapunov parameters
         */
        void setControlParameter(Controller::LyapunovParameter param);

        /**
         * @brief Set the Control Parameter object
         * 
         * @param param Angle Distance Parameters
         */
        void setControlParameter(Controller::AngleDistanceParameter param);


        /**
         * @brief Loading of ROS parameterset for this controller
         * 
         */
        void load();
     

        

        
        /*Linking topics #################################################################################################################################
        ##################################################################################################################################################*/
        
        /**
         * @brief Links the current odometry topic to the controller. At this topic the current odomotry of the robot must be published
         * 
         * @param topic_name Name of the topic
         */
        void linkCurrentOdom(std::string topic_name);  
       
         /**
         * @brief Links the target/desired odometry topic to the controller. At this topic the desired odometry is published.
         * 
         * @param topic_name Name of the topic
         */                           
        void linkTargetOdom(std::string topic_name);

        /**
         * @brief Links the output velocity of the controller to a given topic
         * 
         * @param topic_name Name of the topic
         */
        void linkOutputVelocity(std::string topic_name);      
        
        /**
         * @brief Links the meta data of the controller to a given topic
         * 
         * @param topic_name Name of the topic
         */
        void linkOutputControlData(std::string topic_name);
        
        /*Callbacks########################################################################################################################################
        ##################################################################################################################################################*/
        
        /**
         * @brief Callback for incoming odometry data for the current robot odometry.
         * 
         * @param msg Incoming message
         */
        virtual void currentOdomCallback(nav_msgs::Odometry msg);
       
        /**
         * @brief Callback for incoming state data for the target odometry of the robot. This is overloaded by child classes.
         * 
         * @param msg Incoming message
         */                
        virtual void targetOdomCallback(nav_msgs::Odometry msg);
        
         
     

        /**
         * @brief Scope of the controller that is called in control frequence 
         * 
         */
        void execute(const ros::TimerEvent &ev);
        
        /**
         * @brief Resetting controller to initial state
         * 
         */
        void reset();   

        
        /*Service routines ####################################################################################################################
        ##################################################################################################################################################*/
       
        /**
         * @brief Service procedure for resetting the Controller
         * 
         * @param req Reqest for the service
         * @param res Responce of the service
         * @return true succeeded
         * @return false not succeeded
         */
        bool srvReset(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);

        /**
         * @brief Service for setting the initial Pose of the controller
         * 
         * @param req Reqest that contains the pose
         * @param res Response that contains the succeded flag
         * @return true Succeeded
         * @return false Not Succeeded
         */
        bool srvSetReferenceFrame(multi_robot_msgs::SetInitialPoseRequest &req,multi_robot_msgs::SetInitialPoseResponse &res);


        //###################################################################################################################################
        //###################################################################################################################################

        /**
         * @brief Converts ControlState struct to a COntrol state message
         * 
         * @param state Control state to be converted
         * @param msg Referenze to the msg the conversion is stored in
         */
        void controlState2controlStateMsg(ControlState &state,multi_robot_msgs::ControlState &msg); 


        /**
         * @brief Converts ControlDifference struct to a COntrol state message
         * 
         * @param difference ControlDifference to be converted
         * @param msg Referenze to the msg the conversion is stored in
         */
        void controlDifference2controlDifferenceMsg(ControlDifference &difference,multi_robot_msgs::ControlDifference &msg);  
        

        /**
         * @brief Converts ControlVector struct to a COntrol state message
         * 
         * @param control ControlVector to be converted
         * @param msg Referenze to the msg the conversion is stored in
         */
        void controlVector2controlVectorMsg(ControlVector &control,multi_robot_msgs::ControlVector &msg);         

        
    protected:
        ros::NodeHandle nh;                                         ///<Node Handle
        ros::NodeHandle robot_nh_;
        ros::NodeHandle controller_nh;

        tf::TransformListener* listener;                            ///<Listener for any transformation
        tf::TransformBroadcaster broadcaster_;                      ///<Broadcaster for broadcasting transformations
       
        std::string world_frame;                                    ///<Name of the world frame
        std::string name;                                            ///<Name of the node respective Controller
        
        ControlState current_state_;                                ///<The current state of the robot
        ControlState target_state_;                                 ///<The target state of the robot
        ControlState target_state_old_;                             ///<The target state of a timepstep backwards for numerical differentiaiton
        ros::Time time_old_;                                        ///<Time of the old state
        ControlDifference control_dif_;                             ///<Transformation from current configuration to target configuration
        ControlVector control_;                                     ///<The calculated control vector

        tf::Transform reference_frame_;                             ///<Transformation from a world to the controllers refrence frame
        ControllerType type;                                         ///<Type of control algorythm that is used
        VelocityCartesian thresh_; ///<Threshold velocity. Values smaller then thresh are interpreted as zero
    private:
        ros::Publisher pub_cmd_vel;                                  ///<publisher object for velocity outoput topic
        ros::Publisher pub_state_out;                                ///<publisher object for state output topic
        ros::Publisher pub_control_data;                             ///<publisher object for control difference topic
      
        ros::Subscriber sub_odom_current;                            ///<Subscriber object for odometry
        ros::Subscriber sub_odom_target;                             ///<Subscriber object for target odometry topic

        ros::ServiceServer srv_reset;                                ///<Service for resetting the controller
        ros::ServiceServer srv_set_reference_frame_;     ///<Service for setting the initial pose
        ros::Timer time_scope_;                                      ///<Timer for control scope

        bool publish_tf_;

      
        
        LyapunovParameter lyapunov_parameter_;                        ///<Parameter set for lyapunov determinations
        AngleDistanceParameter angle_distance_parameter_;   ///<Parameter set for angle distance control law
       


        void publish();                                              ///<Publish all outgoing data

        /**
         * @brief Adding the world frame. Broadcastes a world frame respectiveliy the robot reference frame expressed in world coordinates.
         * 
         */
        void publishReferenceFrame();

        void publishControlMetaData();

        void publishVelocityCommand();

        void publishBaseLink();


        /*Calculations and executions ####################################################################################################################
        ##################################################################################################################################################*/
       
        /**
        * @brief 
        * 
        * @param parameter Set of lyapunov parameter for calculating the control vector
        * @param desired Desired velocities in eulerian description (moved base). Contains angular and linear velocity
        * @param relative Transformation from current to target state
        * @return ControlVector 
        */
        ControlVector calcLyapunov(LyapunovParameter parameter,ControlState target, ControlState current);

        virtual ControlVector calcAngleDistance(AngleDistanceParameter parameter,ControlState target, ControlState current);

        virtual ControlVector calcOptimalControl();

        ControlVector passVelocity();

        
};