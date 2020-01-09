#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <multi_robot_msgs/Formation.h>
#include <tf/transform_listener.h>
#include <multi_robot_msgs/ControlData.h>
#include <simulation_env/formation.h>


Formation form_initial;
Formation form_target;


void callback_target(nav_msgs::Odometry msg)
{
    try{
        if(!form_initial.empty())
        {
            tf::Transform trafo;
            tf::poseMsgToTF(msg.pose.pose,trafo);
            form_target=Formation::transform(form_initial,trafo);
            
        }
    }
    catch(std::exception &e)
    {
        ROS_INFO("Target: %s",e.what());
    }
   
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"Measure");
    ros::NodeHandle nh;    
    
    
    try
    {
        form_initial.addRobot(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)),"robot_master",std::vector<int>{1,2});
        form_initial.addRobot(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(-1.5,1.5,0)),"robot1",std::vector<int>(1,0));
        form_initial.addRobot(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(-1.5,-1.5,0)),"robot2",std::vector<int>(1,0));
        form_initial.setReferenceFrame("/map");
    }
    catch(std::exception &ex)
    {
        ROS_WARN(ex.what());
    }    

    ros::Subscriber input=nh.subscribe("/trajectory_odom",10,callback_target);    

    ros::Rate rate(10);

    FormationPublisher pub_tar(nh,"target_formation");
    FormationPublisher pub_curr(nh,"current_formation");
    FormationPublisher pub_est(nh,"estimated_formation");
    FormationPublisher pub_dif(nh,"difference_formation");

    Formation form_current=form_initial;
    Formation form_estimated=form_initial;
    form_target=form_initial;
    FormationSubscriber sub_curr(nh,&form_current,"base_pose_ground_truth");
    FormationSubscriber sub_est(nh,&form_estimated,"odometry/filtered");

    
    while(ros::ok())
    {
        pub_tar.publish(form_target);
        pub_curr.publish(form_current);
        pub_est.publish(form_estimated);
        pub_dif.publish(form_target-form_estimated);
        
        ros::spinOnce();
        rate.sleep();
    }

}