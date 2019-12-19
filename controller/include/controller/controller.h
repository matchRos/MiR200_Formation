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
#include <multi_robot_msgs/State.h>
#include <multi_robot_msgs/Velocity.h>


#include <math.h>
#include <stdio.h>


#define PARAM_TARGET_VEL "topic_target_velocity"
#define PARAM_TARGET_STATE "topic_target_state"
#define PARAM_TARGET_ODOM "topic_target_odometry"
#define PARAM_CURRENT_ODOM "topic_current_odometry"
#define PARAM_CURRENT_STATE "topic_current_state"


#define PARAM_COORD "coord"
#define PARAM_WORLD_FRAME "world_frame"
#define PARAM_TYPE "controller_type"

#define PARAM_LYAPUNOV "/algorithm/lyapunov"
#define PARAM_ANG_DIST "/algorithm/angle_distance"

/**
 * @brief Provides a easy to use implementation of a generic mobile robot controller.
 */
class Controller{
    public:
        /**
         * @brief Construct a new Controller object
         * 
         * @param nh Ros nodehandle for managing namespaces and ros functionality within the controller object
         */
        Controller(ros::NodeHandle &nh);
        ~Controller();
        /**
         * @brief Specifies the different implemented control laws
         * 
         */
        enum controllerType{
            pseudo_inverse=1,   /**< Control law is based on pseudo inveres the input for generate a output (least squares optimisation) */
            lypanov=2,          /**< Control law is based on the lyapunov approache. Output is determined from a lyapunov stable function */
            angle_distance=3    /**< Control law based on linear approach in angle and distance respectively */
        };
        
        /**
         * @brief Specifies the parameters needed for the lyapunov base control law
         * 
         */
        struct lyapunov
        {
            float kx;           /**< Control gain in x-direction */
            float ky;           /**< Control gain in y-direction */ 
            float ktheta;       /**< Control gain in theta-direction */
        };
        
        
        /*Setter and parameter methods ###################################################################################################################
        ##################################################################################################################################################*/
        /**
         * @brief Set the name of object
         * 
         * @param name Name to be set
         */
        void set_name(std::string name);       

        /**
         * @brief Set the reference of the mobile robot to be controlled from a global frame
         * 
         * @param x X-position of robot
         * @param y Y-position of robot
         * @param z Z-position of robot
         * @param angle Angular-position of robot 
         */
        void set_reference(double x,double y,double z,double angle);
        /**
         * @brief Set the reference of the mobile robot to be controlled from a global frame
         * 
         * @param coord vector of the robot position [x,y,z]
         * @param angle angel of the robot
         */
        void set_reference(std::vector<double> coord,double angle);
  


       
        /**
         * @brief Set the world frame name all references are given in
         * 
         * @param frame Name of the world frame
         */
        void set_world_frame(std::string frame);

        /**
         * @brief Sets the type of the used control law
         * 
         * @param type Type of the controller as defined in controllerType
         */
        void set_type(Controller::controllerType type);

        /**
         * @brief Sets the parameter of the lyapunov control law
         * 
         * @param param Parameterset as defined in lyapunov struct
         */
        void set_lyapunov(Controller::lyapunov param);

        /**
         * @brief  Sets the parameter of the lyapunov control law
         * 
         * @param param vector of given parameters [kx,ky,ktheta,vd,omega]
         */
        void set_lyapunov(std::vector<float> param);

        ///Loading parameter for a specified Controller. Empty for Controller base class and implemented in inheriting classes.
        virtual void load_parameter();
       
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
        void link_current_odom(std::string topic_name);       
       
       /**
        * @brief Links the target/desired velocity topic to the controller. At this topic the current desired velocity is published.
        * 
        * @param topic_name Name of the topic
        */
        void link_target_velocity(std::string topic_name); 
        /**
         * @brief Links the target/desired velocity topic to the controller. At this topic the desired velocity is published.
         * 
         * @param topic_name Name of the topic
         */
        
        /**
         * @brief Links the target/desired state topic to the controller. At this topic the current state of the robot is published.
         * 
         * @param topic_name Name of the topic
         */       
        void link_target_state(std::string topic_name);

         /**
         * @brief Links the target/desired odometry topic to the controller. At this topic the desired odometry is published.
         * 
         * @param topic_name Name of the topic
         */                           
        void link_target_odometry(std::string topic_name);

        /**
         * @brief Links the output velocity of the controller to a given topic
         * 
         * @param topic_name Name of the topic
         */
        void link_output_velocity(std::string topic_name);      
        
        /**
         * @brief Links the output ste of the controller to a given topic
         * 
         * @param topic_name Name of the topic
         */                   
        void link_output_state(std::string topic_name);        
        
        /**
         * @brief Links the meta data of the controller to a given topic
         * 
         * @param topic_name Name of the topic
         */
        void link_output_control_data(std::string topic_name);
        
        
         
        /*Calculations and executions ####################################################################################################################
        ##################################################################################################################################################*/
       /**
        * @brief Calculates the lyapunov base control output from a given input
        * 
        * @param kx Control gain in x-direction
        * @param ky Control gain in y-direction
        * @param kphi Control gain in z-direction
        * @param vd  Desired linear velocity
        * @param omegad Desired angular velocity
        */
        void calc_Lyapunov(double kx, double ky, double kphi,double vd,double omegad);

        /**
         * @brief Calculate control output just by angle difference and eklidian difference from target
         * 
         * @param kr Control gain in linear direction
         * @param kphi Control gain in angular direction
         */
        void calc_angle_distance(double kr,double kphi);
        
        /**
         * @brief Scope of the controller that is called in control frequence 
         * 
         */
        void execute();
        /**
         * @brief Resetting controller to initial state
         * 
         */
        void reset();     

        

        /*Callbacks########################################################################################################################################
        ##################################################################################################################################################*/
        /// Callback for input odometry message. Is executed everytima a Odometry input is incoming. Writes data to input current_pose
        void current_odom_callback(nav_msgs::Odometry msg);
        /// Callback for input velocitiy message. Is executed everytime a velocitiy input is incoming. Writes data to input state
        virtual void target_velocities_callback(geometry_msgs::Twist msg);         
        /// Callback for input target state message. Is executed everytima a target state input is incoming. Writes data to target_pose state
        virtual void target_state_callback(geometry_msgs::PoseStamped msg);   
          /// Callback for input target odometry message. Is executed everytima a target odometry input is incoming. Writes data to target_pose state and target_vel
        virtual void target_odometry_callback(nav_msgs::Odometry msg);
        
        
        //Callback for the reset server
        bool srv_reset(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);



        /// A Scope within the execute function. Repeating calculation in inheriting classes are implemented here
        virtual void scope()=0;       
       
                

        
    protected:
        ros::NodeHandle nh;                                          ///<Node Handle
        tf::TransformListener* listener;
        controllerType type;                                         ///<Type of control algorythm that is used


        ros::Publisher pub_vel_out;                                  ///<publisher object for velocity outoput topic
        ros::Publisher pub_state_out;                                ///<publisher object for state output topic
        ros::Publisher pub_control_data;                       ///<publisher object for control difference topic
        
        ros::Subscriber sub_vel_target;                              ///<Subscirber object for input topic
        ros::Subscriber sub_odom_current;                            ///<Subscriber object for odometry
        ros::Subscriber sub_state_target;                            ///<Subscriber object for target state of controller
        ros::Subscriber sub_target_odometry;                         ///<Subscriber object for target odometry topic

        ros::ServiceServer reset_service;                           ///<Service for resetting the controller

        std::string name;                                           ///<Name of the node respective Controller
        std::string world_frame;                                    ///<Name of the world frame
        
        tf::Vector3 ang_vel_in;                                 ///<target linear velocity
        tf::Vector3 lin_vel_in;                                 ///<target angular velocity

        tf::Vector3 lin_vel_out;                                    ///<Outgoing linear velocity
        tf::Vector3 ang_vel_out;                                    ///<Outgoing angular velocity
        
        tf::StampedTransform world2robot;                   
        tf::StampedTransform world2odom;

        tf::Transform control_dif;                                  ///<Transformation from current to target

        tf::Pose reference_pose;                                    ///<Reference pose to a global system
        tf::Pose current_pose;                                      ///<Pose of Controller at the moment in world
        tf::Pose target_pose;                                       ///<The Target for the Controller poses
        
        void getTransformation();                                   ///<Listen to all neccesary trasnformations
        void publish();                                             ///<Publish all outgoing data
        lyapunov lyapunov_parameter;
       

        double kx;
        double ky;
        double kphi;
        double omegad;
        double vd;

        double kr;
        double kang;


        //Use the 
        bool loaded_parameter;

    private:
        ///Adding a transformation to the map frame, the reference pose is defined in
        void add_map();
        
};