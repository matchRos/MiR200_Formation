#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <multi_robot_msgs/Formation.h>
#include <tf/transform_listener.h>
#include <multi_robot_msgs/ControlData.h>
#include <simulation_env/formation.h>


Formation form_initial;
Formation form_target;
Formation form_current;
Formation form_estimated;

FormationPublisher* pub_initial;
FormationPublisher* pub_current;
FormationPublisher* pub_target;
FormationPublisher* pub_estimated;
FormationPublisher* pub_difference;


std::vector<tf::Pose> formation_initial;

std::vector<tf::Pose> formation_target;
std::vector<tf::Pose> formation_current;
std::vector<tf::Pose> formation_estimated;




ros::Publisher formation_current_pub;
ros::Publisher formation_target_pub;
ros::Publisher formation_estimated_pub;
ros::Publisher formation_difference_pub;



void callback_target(nav_msgs::Odometry msg)
{
    try{
        if(!form_initial.empty())
        {
            tf::Transform trafo;
            tf::poseMsgToTF(msg.pose.pose,trafo);
            form_target=Formation::transform(form_initial,trafo);
            
        }
        pub_target->publish(form_target);
    }
    catch(std::exception &e)
    {
        ROS_INFO("Target: %s",e.what());
    }
   
}

void callback_pos_master(nav_msgs::Odometry msg)
{    
    try{
        tf::Pose pose;
        tf::poseMsgToTF(msg.pose.pose,pose);
        form_current.modifiePose(0,pose);
        pub_current->publish(form_current);
    }
    catch(std::exception &ex)
    {
        ROS_INFO("callback_pos_master: %s",ex.what());
    }
    
}

void callback_pos_slave1(nav_msgs::Odometry msg)
{ 
    try{
        tf::Pose pose;
        tf::poseMsgToTF(msg.pose.pose,pose);
        form_current.modifiePose(1,pose);
        pub_current->publish(form_current);
    }
    catch(std::exception &ex)
    {
        ROS_INFO("callback_pos_slave1: %s",ex.what());
    }
    
}

void callback_pos_slave2(nav_msgs::Odometry msg)
{    
    try{
        tf::Pose pose;
        tf::poseMsgToTF(msg.pose.pose,pose);
        form_current.modifiePose(2,pose);
        pub_current->publish(form_current);
    }
    catch(std::exception &ex)
    {
        ROS_INFO("callback_pos_slave2: %s",ex.what());
    }
}



void callback_pos_est_master(multi_robot_msgs::ControlData msg)
{
    try
    {
        tf::Pose pose;
        tf::poseMsgToTF(msg.current.pose,pose);
        form_estimated.modifiePose(0,pose);
        pub_estimated->publish(form_estimated);

        pub_difference->publish(form_target-form_current); 
    }
    catch(std::exception &ex)
    {
        ROS_INFO("callback_pos_est_master: %s",ex.what());
    }  
}

void callback_pos_est_slave1(multi_robot_msgs::ControlData msg)
{
   try
   {
        tf::Pose pose;
        tf::poseMsgToTF(msg.current.pose,pose);
        form_estimated.modifiePose(1,pose);
        pub_estimated->publish(form_estimated);

        pub_difference->publish(form_target-form_current); 
    }
    catch(std::exception &ex)
    {
        ROS_INFO("callback_pos_est_slave1: %s",ex.what());
    }
   
}

void callback_pos_est_slave2(multi_robot_msgs::ControlData msg)
{
    try
    {
        tf::Pose pose;
        tf::poseMsgToTF(msg.current.pose,pose);
        form_estimated.modifiePose(2,pose);
        pub_estimated->publish(form_estimated);

        pub_difference->publish(form_target-form_current); 
    }
    catch(std::exception &ex)
    {
        ROS_INFO("callback_pos_est_slave2: %s",ex.what());
    }
}



int main(int argc,char** argv)
{
    try
    {
        form_initial.addRobot(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)),std::vector<int>{1,2});
        form_initial.addRobot(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(-1.5,1.5,0)),std::vector<int>(1,0));
        form_initial.addRobot(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(-1.5,-1.5,0)),std::vector<int>(1,0));

        form_current=form_initial;
        form_estimated=form_initial;
        form_target=form_initial;
    
    }
    catch(std::exception &ex)
    {
        ROS_WARN(ex.what());
    }    
    
    ros::init(argc,argv,"Measure");
    ros::NodeHandle nh;   


    ros::Subscriber pos_master=nh.subscribe("robot_master/base_pose_ground_truth",10,callback_pos_master);
    ros::Subscriber pos_slave1=nh.subscribe("robot1/base_pose_ground_truth",10,callback_pos_slave1);
    ros::Subscriber pos_slave2=nh.subscribe("robot2/base_pose_ground_truth",10,callback_pos_slave2);

    ros::Subscriber pos_master_est=nh.subscribe("robot_master/control_data",10,callback_pos_est_master);
    ros::Subscriber pos_slave_est_1=nh.subscribe("robot1/control_data",10,callback_pos_est_slave1);
    ros::Subscriber pos_slave_est_2=nh.subscribe("robot2/control_data",10,callback_pos_est_slave2);

    ros::Subscriber input=nh.subscribe("/trajectory_odom",10,callback_target);

    pub_target=new FormationPublisher(nh,"target_formation");
    pub_current=new FormationPublisher(nh,"current_formation");
    pub_estimated=new FormationPublisher(nh,"estimated_formation");
    pub_difference= new FormationPublisher(nh,"difference_formation");

    ros::spin();


    delete pub_target;
    delete pub_difference;
    delete pub_initial;
    delete pub_estimated;

}