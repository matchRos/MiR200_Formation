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


#define PARAM_IN_VEL "topic_input_velocity"
#define PARAM_OUT_VEL "topic_output_velocity"
#define PARAM_IN_ODOM "topic_input_odometry"
#define PARAM_OUT_STATE "topic_output_state"
#define PARAM_X  "x_coord"
#define PARAM_Y  "y_coord"
#define PARAM_GAINS
#define PARAM_WORLD_FRAME "world_frame"
#define PARAM_TYPE "controller_type"


class Controller{
    public:
        Controller(ros::NodeHandle &nh); 
        enum controllerType{
            pseudo_inverse=1,
            lypanov=2,
        };

        
        
        
        //Setter and parametring methods

        ///setting the name of the Controller and its node
        void set_name(std::string);  
        ///setting the frequenzy of the controller to
        ///param: double frequenzy
        void set_frequenzy(double frequenzy);
        ///Initialise the coordinate system of the Controller to position
        ///param: double x: x Position
        ///param: double y: y Position
        ///param: double z: z Position
        void set_reference(double x,double y,double z);   
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


        
        //Methods for linking the Controller with important topics as input output an odom
        
        ///link Controller to it's input topic
        ///'topic_name': Name of tjhe topic the Controller gets input from
        void link_input_velocity(std::string topic_name);   
        ///link Controller to it's odom input topic
        ///'topic_name'Name of the topic the Controller reads its oddom from                       
        void link_input_odom(std::string topic_name); 
        ///link Controller to it's state topic
        ///'topic_name' Name of the topic the Controller gets its target state from                       
        void link_input_state(std::string topic_name);      
        ///link Controller to it's input topic
        ///'topic_name'Name of the topic the Controller writes its output to                       
        void link_output_velocity(std::string topic_name);      
        ///link Controller to it's state topic
        ///'topic_name' Name of the topic the Controller writes its state to                       
        void link_output_state(std::string topic_name);
        ///link Controller to it's control difference topic
        ///'topic_name' Name of the topic the Controller writes its control difference to                       
        void link_control_difference(std::string topic_name);
        
        

        ///Calculate the control vector with the Lyapunov method
        ///param: kx: gain in x direction
        ///param kphi: gain in angular direction
        ///param vd: tangential velocity
        ///param omegad : rotational velocity
        virtual void calc_Lyapunov(double kx, double kphi,double vd,double omegad);


        /// A Scope within the execute function. Repeating calculation in inheriting classes are implemented here
        virtual void scope();       
        ///Controller scope
        void execute();

      

        

        //Callbacks
        /// Callback for input velocitiy message. Is executed everytima a velocitiy input is incoming. Writes data to input state
        void input_velocities_callback(geometry_msgs::Twist msg);         
        /// Callback for input odometry message. Is executed everytima a Odometry input is incoming. Writes data to input current_pose
        void input_odom_callback(nav_msgs::Odometry msg);              
         /// Callback for input target state message. Is executed everytima a target state input is incoming. Writes data to target_pose state
        void input_state_callback(nav_msgs::Odometry msg);              

        
    protected:
        ros::NodeHandle nh;                                     //Node Handle
        tf::TransformListener listener;

        ros::Publisher vel_out;                                  //publisher object for velocity outoput topic
        ros::Publisher state_out;                               //publisher object for state output topic
        ros::Publisher control_difference;                      //publisher object for control difference topic
        
        ros::Subscriber vel_in;                                  //Subscirber object for input topic
        ros::Subscriber odom;                                   //Subscriber object for odometry
        ros::Subscriber state_in;                               //Subscriber object for target state of controller


      

        std::string name;                                       //Name of the node and Controller
        std::string world_frame;                                //Name of the world frame

        controllerType type;                                    //Type of control algorythm that is used

        geometry_msgs::Twist msg_velocities_in;                 //Input velocities
        geometry_msgs::Twist msg_velocities_out;                //output velocities
        geometry_msgs::Twist msg_velocities_ideal;              //Ideal velocity state
        
        tf::StampedTransform robot2world;

        geometry_msgs::Pose reference;                          //Reference position
        tf::Pose current_pose;                                  //Pose of Controller at the moment in world
        tf::Pose target_pose;                                   //The Target for the Controller poses
        
        void getTransformation();
        void publish();
};