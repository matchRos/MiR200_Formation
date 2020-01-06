#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <multi_robot_msgs/Formation.h>
#include <tf/transform_listener.h>
#include <multi_robot_msgs/ControlData.h>
#include <simulation_env/formation.h>



std::vector<tf::Pose> formation_initial;

std::vector<tf::Pose> formation_target;
std::vector<tf::Pose> formation_current;
std::vector<tf::Pose> formation_estimated;




ros::Publisher formation_current_pub;
ros::Publisher formation_target_pub;
ros::Publisher formation_estimated_pub;
ros::Publisher formation_difference_pub;

tf::TransformListener* listener;


void Formation2Msg(std::vector<tf::Pose>formation,multi_robot_msgs::Formation &msg)
{    
    for(int i=0;i<formation.size();i++)
    {
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(formation.at(i),pose);
        msg.formation.push_back(pose);
    }
}
std::vector<tf::Pose> operator-(std::vector<tf::Pose> first,std::vector<tf::Pose> second)
{
    std::vector<tf::Pose> result;
    for(int i=0;i<second.size();i++)
    {
        result.push_back(first.at(i).inverseTimes(second.at(i)));
    }
    return result;
}

void callback_target(nav_msgs::Odometry msg)
{
    if(!formation_initial.empty())
    {
        for(int i=0;i<formation_initial.size();i++)
        {
            tf::Transform trafo;
            tf::poseMsgToTF(msg.pose.pose,trafo);
            if(i<formation_target.size())
            {
                formation_target.at(i)=trafo*formation_initial.at(i);
            }
            else
            {
                formation_target.push_back(trafo*formation_initial.at(i));
            }        
        }
        multi_robot_msgs::Formation formation_msg;
        Formation2Msg(formation_target,formation_msg);
        formation_target_pub.publish(formation_msg);
    }
    
}

void callback_pos_master(nav_msgs::Odometry msg)
{    
    multi_robot_msgs::Formation formation_msg;
    tf::poseMsgToTF(msg.pose.pose,formation_current.at(0));
    Formation2Msg(formation_current,formation_msg);
    formation_current_pub.publish(formation_msg);
}

void callback_pos_slave1(nav_msgs::Odometry msg)
{ 
    multi_robot_msgs::Formation formation_msg;
    tf::poseMsgToTF(msg.pose.pose,formation_current.at(1));
    Formation2Msg(formation_current,formation_msg);
    formation_current_pub.publish(formation_msg);
}

void callback_pos_slave2(nav_msgs::Odometry msg)
{
    multi_robot_msgs::Formation formation_msg;
    tf::poseMsgToTF(msg.pose.pose,formation_current.at(2));
    Formation2Msg(formation_current,formation_msg);
    formation_current_pub.publish(formation_msg);

}




void callback_pos_est_master(multi_robot_msgs::ControlData msg)
{
    tf::poseMsgToTF(msg.current.pose,formation_estimated.at(0));
    
    multi_robot_msgs::Formation formation_msg;
    Formation2Msg(formation_estimated,formation_msg);
    formation_estimated_pub.publish(formation_msg);
        
    multi_robot_msgs::Formation difference;
    Formation2Msg(formation_current-formation_target,difference);
    formation_difference_pub.publish(difference);
}

void callback_pos_est_slave1(multi_robot_msgs::ControlData msg)
{
    tf::poseMsgToTF(msg.current.pose,formation_estimated.at(1));
    
    multi_robot_msgs::Formation formation_msg;
    Formation2Msg(formation_estimated,formation_msg);
    formation_estimated_pub.publish(formation_msg);

    multi_robot_msgs::Formation difference;
    Formation2Msg(formation_current-formation_target,difference);
    formation_difference_pub.publish(difference);
}

void callback_pos_est_slave2(multi_robot_msgs::ControlData msg)
{
    tf::poseMsgToTF(msg.current.pose,formation_estimated.at(2));
    
    multi_robot_msgs::Formation formation_msg;
    Formation2Msg(formation_estimated,formation_msg);
    formation_estimated_pub.publish(formation_msg);

    multi_robot_msgs::Formation difference;
    Formation2Msg(formation_current-formation_target,difference);
    formation_difference_pub.publish(difference);

}

int main(int argc,char** argv)
{
    formation_initial.push_back(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)));
    formation_initial.push_back(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(-1.5,1.5,0)));
    formation_initial.push_back(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(-1.5,-1.5,0)));

    formation_target=formation_initial;
    formation_estimated=formation_initial;
    formation_current=formation_initial;
    
    
    ros::init(argc,argv,"Measure");
    ros::NodeHandle nh;   

    listener=new tf::TransformListener;

    ros::Subscriber pos_master=nh.subscribe("robot_master/base_pose_ground_truth",10,callback_pos_master);
    ros::Subscriber pos_slave1=nh.subscribe("robot1/base_pose_ground_truth",10,callback_pos_slave1);
    ros::Subscriber pos_slave2=nh.subscribe("robot2/base_pose_ground_truth",10,callback_pos_slave2);

    ros::Subscriber pos_master_est=nh.subscribe("robot_master/control_data",10,callback_pos_est_master);
    ros::Subscriber pos_slave_est_1=nh.subscribe("robot1/control_data",10,callback_pos_est_slave1);
    ros::Subscriber pos_slave_est_2=nh.subscribe("robot2/control_data",10,callback_pos_est_slave2);

    ros::Subscriber input=nh.subscribe("/trajectory_odom",10,callback_target);

    formation_target_pub=nh.advertise<multi_robot_msgs::Formation>("target_formation",10);
    formation_current_pub=nh.advertise<multi_robot_msgs::Formation>("current_formation",10);
    formation_estimated_pub=nh.advertise<multi_robot_msgs::Formation>("estimated_formation",10);
    formation_difference_pub=nh.advertise<multi_robot_msgs::Formation>("difference_formation",10);;

    ros::spin();
    delete listener;

}