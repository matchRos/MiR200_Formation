#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>



int main(int argc, char **argv)
{
    ros::init(argc,argv,"circle");
    ros::NodeHandle nh;

    ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("/trajectory",10);

    double phi=0;
    double omega=0.1;
    double r=3;
    tf::Vector3 x(0,0,0);
    tf::Vector3 off(0,-r,0);
    ros::Rate rate(10);
    long int steps;
    while(nh.ok())
    {
        x.setX(sin(omega*steps/10)*r);
        x.setY(-cos(omega*steps/10)*r);
        x=x-off;
        geometry_msgs::PoseStamped msg;
        msg.pose.position.x=x.x();
        msg.pose.position.y=x.y();
        msg.pose.position.z=x.z();

        tf::Quaternion quat;
        quat.setEulerZYX(omega*steps/10,0,0);
        tf::quaternionTFToMsg(quat,msg.pose.orientation);
        pub.publish(msg);
        rate.sleep();
        steps++;
    }


}