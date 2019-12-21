#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>



void callback_subscription(nav_msgs::Odometry msg)
{
    static tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped trafo;
    trafo.child_frame_id=msg.child_frame_id;
    trafo.header=msg.header;
    trafo.transform.rotation=msg.pose.pose.orientation;
    trafo.transform.translation.x=msg.pose.pose.position.x;
    trafo.transform.translation.y=msg.pose.pose.position.y;
    trafo.transform.translation.z=msg.pose.pose.position.z;
    broadcaster.sendTransform(trafo);
}

int main (int argc,char** argv)
{
    ros::init(argc,argv,"ground_truth_broadcaster",ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("base_pose_ground_truth",1,callback_subscription);
    ros::spin();    
}