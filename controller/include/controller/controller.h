#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>


#include <math.h>
#include <stdio.h>


#define PARAM_TARGET_VEL "topic_target_velocity"
#define PARAM_TARGET_STATE "topic_target_state"
#define PARAM_CURRENT_ODOM "topic_input_odometry"
#define PARAM_CURRENT_STATE "topic_current_state"

#define PARAM_COORD "coord"
#define PARAM_WORLD_FRAME "world_frame"
#define PARAM_TYPE "controller_type"

#define PARAM_LYAPUNOV "/algorithm/lyapunov"
#define PARAM_ANG_DIST "/algorithm/angle_distance"


class Controller{
    public:
        Controller(ros::NodeHandle &nh);
        ~Controller();

        enum controllerType{
            pseudo_inverse=1,
            lypanov=2,
            angle_distance=3
        };
       
        
        
        /*Setter and parameter methods ###################################################################################################################
        ##################################################################################################################################################*/
        ///setting the name of the Controller and its node
        void set_name(std::string);  
        ///setting the frequenzy of the controller to
        ///param: double frequenzy
        void set_frequenzy(double frequenzy);
        ///Initialise the coordinate system of the Controller to position
        ///param: double x: x Position
        ///param: double y: y Position
        ///param: double z: z Position
        void set_reference(double x,double y,double z,double angle); 
        ///Initialise the coordinate system of the Controller to position
        ///param: vector<double> coord: Position vector    
        void set_reference(std::vector<double> coord,double angle);     
        ///Set the name of the reference/world frame
        ///param: frame : Name of the frame in tf tree
        void set_world_frame(std::string frame);      
        ///Set the type of the controllaw 
        ///param: type: Controllaw type (see Controller::controllerType)
        void set_type(Controller::controllerType type);        
        ///Loading parameter for a specified Controller. Empty for Controller base class and implemented in inheriting classes.
        virtual void load_parameter();
        ///Loading ros parameter and calling load_parameter inside
        void load();
        ///Adding a transformation to the map frame, the reference pose is defined in
        void add_map();


        

        
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
        ///link Controller to it's input topic
        ///'topic_name'Name of the topic the Controller writes its output to                       
        void link_output_velocity(std::string topic_name);      
        ///link Controller to it's state topic
        ///'topic_name' Name of the topic the Controller writes its state to                       
        void link_output_state(std::string topic_name);
        ///link Controller to it's control difference topic
        ///'topic_name' Name of the topic the Controller writes its control difference to                       
        void link_output_ctrldiff(std::string topic_name);
        
        
         
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

        void reset();     

        

        /*Callbacks########################################################################################################################################
        ##################################################################################################################################################*/
        /// Callback for input velocitiy message. Is executed everytime a velocitiy input is incoming. Writes data to input state
        void target_velocities_callback(geometry_msgs::Twist msg);         
        /// Callback for input odometry message. Is executed everytima a Odometry input is incoming. Writes data to input current_pose
        void current_odom_callback(nav_msgs::Odometry msg);
        /// Callback for input target state message. Is executed everytima a target state input is incoming. Writes data to target_pose state
        virtual void target_state_callback(geometry_msgs::PoseStamped msg);   




        /// A Scope within the execute function. Repeating calculation in inheriting classes are implemented here
        virtual void scope()=0;       
       
                

        
    protected:
        ros::NodeHandle nh;                                          //Node Handle
        tf::TransformListener* listener;
        controllerType type;                                         //Type of control algorythm that is used


        ros::Publisher pub_vel_out;                                  //publisher object for velocity outoput topic
        ros::Publisher pub_state_out;                                //publisher object for state output topic
        ros::Publisher pub_control_difference;                       //publisher object for control difference topic
        
        ros::Subscriber sub_vel_target;                              //Subscirber object for input topic
        ros::Subscriber sub_odom_current;                            //Subscriber object for odometry
        ros::Subscriber sub_state_target;                            //Subscriber object for target state of controller


        std::string name;                                           //Name of the node respective Controller
        std::string world_frame;                                    //Name of the world frame
        
        tf::Vector3 ang_vel_in;                                     //target angular velocity
        tf::Vector3 lin_vel_in;                                     //target linear velocity

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

        //Control algorithm parameter
        double kx;
        double ky;
        double kphi;
        double omegad;
        double vd;

        double kr;
        double kang;


        //Use the 
        bool loaded_parameter;


        
};