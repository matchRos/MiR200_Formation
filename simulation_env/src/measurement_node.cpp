#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <multi_robot_msgs/Formation.h>


#include <tf/transform_listener.h>
#include <multi_robot_msgs/ControlData.h>

#include <controller/laser_predictor.h>

#include <simulation_env/formation.h>
#include <simulation_env/formation_publisher.h>
#include <simulation_env/formation_subscriber.h>


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
        form_initial.setReferenceFrame("/map");
        form_initial.addRobot(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(-1.5,-1.5,0)),"robot2",std::vector<int>{2,1});
        form_initial.addRobot(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)),"robot_master",std::vector<int>{0,2});
        form_initial.addRobot(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(-1.5,1.5,0)),"robot1",std::vector<int>{1,0});
      
        
    }
    catch(std::exception &ex)
    {
        ROS_WARN("%s",ex.what());
    }    

    ros::Subscriber input=nh.subscribe("/trajectory_odom",10,callback_target);    

   

    ros::NodeHandle nh2("formation");
    FormationPublisher pub_tar(nh2,"target_formation");
    FormationPublisher pub_curr(nh2,"current_formation");
    FormationPublisher pub_est(nh2,"estimated_formation");
    FormationPublisher pub_dif(nh2,"difference_formation");

    Formation form_current=form_initial;
    Formation form_estimated=form_initial;
    form_target=form_initial;

    std::vector<std::string> topics_current;
    topics_current.push_back("base_pose_ground_truth");
    topics_current.push_back("scan");
    std::vector<std::string>topics_est;
    topics_est.push_back("odometry/filtered");
    
    FormationSubscriber sub_curr(nh,&form_current,topics_current);
    FormationSubscriber sub_est(nh,&form_estimated,topics_est);

    ros::Publisher pub=nh.advertise<sensor_msgs::PointCloud>("clustered_data",10);
    ros::Publisher pub2=nh.advertise<geometry_msgs::PoseStamped>("estaminated_pose",10);
    
    
   
    
    LaserPredictor::Frames frames("base_link","front_laser_link","back_laser_link");
    LaserPredictor::Topics topics("","f_scan","b_scan");
    ros::NodeHandle robot2("robot2");
    LaserPredictor predictor(robot2,frames,topics);

    LaserPredictor::Poses guessed_poses;
    guessed_poses.push_back(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(1.5,1.5,0.0)));
    guessed_poses.push_back(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0.0,3.0,0.0)));
    predictor.guess(guessed_poses);
    predictor.startClustering(10);


    ros::Rate rate(10);
    while(ros::ok())
    {
       try{
           // pub_tar.publish(form_target);
            pub_curr.publish(form_current);
            pub.publish(predictor.getClusteredPoints());
            // geometry_msgs::PoseStamped pose;
            // pose.header.frame_id="robot2/front_laser_link";
            // tf::poseTFToMsg(predictor.getPose(0),pose.pose);
            // pub2.publish(pose);
            //pub_est.publish(form_estimated);
            //pub_dif.publish(form_target-form_estimated);
       }
       catch(std::exception &e)
       {
           ROS_WARN("%s",e.what());
       }
        
        ros::spinOnce();
        rate.sleep();
    }

}