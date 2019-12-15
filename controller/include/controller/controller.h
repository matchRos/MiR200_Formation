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
            pseudo_inverse=1,   //*< Control law is based on pseudo inveres the input for generate a output (least squares optimisation) */
            lypanov=2,          //*< Control law is based on the lyapunov approache. Output is determined from a lyapunov stable function */
            angle_distance=3    //*< Control law based on linear approach in angle and distance respectively */
        };
        
        /**
         * @brief Specifies the parameters needed for the lyapunov base control law
         * 
         */
        struct lyapunov
        {
            float kx;   //*< Control gain in x-direction */
            float ky;   //*< Control gain in y-direction */ 
            float ktheta;   //*< Control gain in theta-direction */
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
        ///Initialise the coordinate system of the Controller to position
        ///param: vector<double> coord: Position vector    
        void set_reference(std::vector<double> coord,double angle);
        ///Initialise the coordinate system of the Controller to position defined by reference pose
        void set_reference();


       

        ///Set the name of the reference/world frame
        ///param: frame : Name of the frame in tf tree
        void set_world_frame(std::string frame);

        ///Set the type of the controllaw 
        ///param: type: Controllaw type (see Controller::controllerType)
        void set_type(Controller::controllerType type);

        ///Setting parameters for the lyapunov control law
        ///param param: struct that hold the parameters for the law: kx,ky,ktheta,omega,v in that order        
        void set_lyapunov(Controller::lyapunov param);
        ///Setting parameters for the lyapunov control law
        ///param param: std::vector that hold the parameters for the law: kx,ky,ktheta,omega,v in that order        
        void set_lyapunov(std::vector<float> param);

        ///Loading parameter for a specified Controller. Empty for Controller base class and implemented in inheriting classes.
        virtual void load_parameter();
        ///Loading ros parameter and calling load_parameter inside
        void load();
     

        

        
        /*Linking topics #################################################################################################################################
        ##################################################################################################################################################*/
        ///link Controller to it's odom input topic
        ///'topic_name'Name of the topic the Controller reads its oddom from                       
        void link_current_odom(std::string topic_name);       
        ///link Controller to it's input topic
        ///'topic_name': Name of tjhe topic the Controller gets input from
        void link_target_velocity(std::string topic_name);   
        ///link Controller to it's state topic
        ///'topic_name' Name of the topic the Controller gets its target state from                       
        void link_target_state(std::string topic_name);
        ///link Controller to it's target odometry topic (combines target state and target velocity)
        ///'topic_name' Name of the topic the Controller gets its target state from                     
        void link_target_odometry(std::string topic_name);
        
        ///link Controller to it's output topic
        ///'topic_name'Name of the topic the Controller writes its output to                       
        void link_output_velocity(std::string topic_name);      
        ///link Controller to it's state topic
        ///'topic_name' Name of the topic the Controller writes its state to                       
        void link_output_state(std::string topic_name);
        ///link Controller to it's control difference topic
        ///'topic_name' Name of the topic the Controller writes its control difference to                       
        void link_output_control_data(std::string topic_name);
        
        
         
        /*Calculations and executions ####################################################################################################################
        ##################################################################################################################################################*/
        ///Calculate the control vector with the Lyapunov method
        ///param: kx: gain in x direction
        ///param kphi: gain in angular direction
        ///param vd: tangential velocity
        ///param omegad : rotational velocity
        void calc_Lyapunov(double kx, double ky, double kphi,double vd,double omegad);

        void calc_angle_distance(double kr,double kphi);
        ///Controller scope
        void execute();
        //Resets the controller to start configuration
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
        ros::NodeHandle nh;                                          //Node Handle
        tf::TransformListener* listener;
        controllerType type;                                         //Type of control algorythm that is used


        ros::Publisher pub_vel_out;                                  //publisher object for velocity outoput topic
        ros::Publisher pub_state_out;                                //publisher object for state output topic
        ros::Publisher pub_control_data;                       //publisher object for control difference topic
        
        ros::Subscriber sub_vel_target;                              //Subscirber object for input topic
        ros::Subscriber sub_odom_current;                            //Subscriber object for odometry
        ros::Subscriber sub_state_target;                            //Subscriber object for target state of controller
        ros::Subscriber sub_target_odometry;                         //Subscriber object for target odometry topic

        ros::ServiceServer reset_service;                           //Service for resetting the controller

        std::string name;                                           //Name of the node respective Controller
        std::string world_frame;                                    //Name of the world frame
        
        tf::Vector3 ang_vel_in;                                 //target linear velocity
        tf::Vector3 lin_vel_in;                                 //target angular velocity

        tf::Vector3 lin_vel_out;                                    //Outgoing linear velocity
        tf::Vector3 ang_vel_out;                                    //Outgoing angular velocity
        
        tf::StampedTransform world2robot;                   
        tf::StampedTransform world2odom;

        tf::Transform control_dif;                                  //Transformation from current to target

        tf::Pose reference_pose;                                    //Reference pose to a global system
        tf::Pose current_pose;                                      //Pose of Controller at the moment in world
        tf::Pose target_pose;                                       //The Target for the Controller poses
        
        void getTransformation();                                   //Listen to all neccesary trasnformations
        void publish();                                             //Publish all outgoing data
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