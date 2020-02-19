#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <multi_robot_msgs/Formation.h>


#include <tf/transform_listener.h>
#include <multi_robot_msgs/ControlData.h>

#include <formation/laser_predictor.h>

#include <formation/formation.h>
#include <formation/formation_publisher.h>
#include <formation/formation_subscriber.h>


// Formation form_initial;
// Formation form_target;


// void callback_target(nav_msgs::Odometry msg)
// {
//     try{
//         if(!form_initial.empty())
//         {
//             tf::Transform trafo;
//             tf::poseMsgToTF(msg.pose.pose,trafo);
//             form_target=Formation::transform(form_initial,trafo);
            
//         }
//     }
//     catch(std::exception &e)
//     {
//         ROS_INFO("Target: %s",e.what());
//     }
   
// }

int main(int argc,char** argv)
{
    ros::init(argc,argv,"Measure"); 

   
    LaserPredictor::Frames frames("base_link","front_laser_link","back_laser_link");
    LaserPredictor::Topics topics("none","f_scan","b_scan");
    

    Formation::RobotProperties properties_master;
    properties_master.name="master";
    properties_master.neighbours=Formation::Neighbours{"robot1","robot2"};
    properties_master.pose=tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0.0,0.0,0.0));
    properties_master.laser_frames=frames;
    properties_master.laser_topics=topics;
    properties_master.ekf_topic="odometry/filtered";
    properties_master.odom_topic="mobile_base_controller/odom";

    Formation::RobotProperties properties_robot1;
    properties_robot1.name="robot1";
    properties_robot1.neighbours=Formation::Neighbours{"master","robot2"};
    properties_robot1.pose=tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(-1.5,1.5,0.0));
    properties_robot1.laser_frames=frames;
    properties_robot1.laser_topics=topics;
    properties_robot1.ekf_topic="odometry/filtered";
    properties_robot1.odom_topic="mobile_base_controller/odom";

    Formation::RobotProperties properties_robot2;
    properties_robot2.name="robot2";
    properties_robot2.neighbours=Formation::Neighbours{"robot_master","robot1"};
    properties_robot2.pose=tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(-1.5,-1.5,0.0));
    properties_robot2.laser_frames=frames;
    properties_robot2.laser_topics=topics;
    properties_robot2.ekf_topic="odometry/filtered";
    properties_robot2.odom_topic="mobile_base_controller/odom";    


    Formation test;
    test.addRobot(properties_master);
    test.addRobot(properties_robot1);
    test.addRobot(properties_robot2);
    test.startPrediction(10);

    FormationPublisher pub(&test);

    ros::Rate rate(10);
    while(ros::ok())
    {
        try
        {
            pub.publish();
        }
        catch(std::exception &e)
        {
            ROS_WARN("%s",e.what());
        }
        
        ros::spinOnce();
        rate.sleep();
    }

}