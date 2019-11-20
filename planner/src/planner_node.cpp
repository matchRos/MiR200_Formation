#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>



int main(int argc, char **argv)
{
    ros::init(argc,argv,"planner");
    ros::NodeHandle nh;

    ros::Publisher pub=nh.advertise<nav_msgs::Odometry>("trajectory",10);


    
    double phi=0;
    double omega=0.5;
    double r=3;
    tf::Vector3 x(0,0,0);
    tf::Vector3 off(-2,0,0);
    ros::Rate rate(10);
    long int steps;
    while(nh.ok())
    {
        x.setX(cos(omega*steps/10)*r);
        x.setY(sin(omega*steps/10)*r);
        x=x+off;
        nav_msgs::Odometry msg;
        msg.pose.pose.position.x=x.x();
        msg.pose.pose.position.y=x.z();
        msg.pose.pose.position.z=x.z();

        tf::Quaternion quat;
        quat.setEulerZYX(omega*steps/10/360*M_PI,0,0);
        tf::quaternionTFToMsg(quat,msg.pose.pose.orientation);
        pub.publish(msg);
        rate.sleep();
    }


}