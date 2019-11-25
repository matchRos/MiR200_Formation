#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

double curve(double t)
{
    return atan(sqrt(7)-4*cos(t))+atan(sqrt(7)+4*cos(t))-atan(4-sqrt(7))+atan(4+sqrt(7));
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"circle");
    ros::NodeHandle nh;

    ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("/trajectory",10);
    ros::Rate rate(100);

    double curve_parameter =3;
    double t;
    long int steps;
    double phi;
    double v=0.3;

    tf::Vector3 x;
    while(nh.ok())
    {
        t=steps/100;
        t*=v;
        x.setX(curve_parameter*sin(t));
        x.setY(curve_parameter*sin(t)*cos(t));
       
       
        phi=curve(t);
        tf::Quaternion quat;
        quat.setRPY(0,0,phi);

        // tf::Quaternion quat_rot;
        // quat_rot.setRPY(0,0,-curve(0));
        // quat=quat_rot*quat;
        // x=tf::Transform(quat_rot)*x;        
      

        geometry_msgs::PoseStamped msg;
        msg.pose.position.x=x.x();
        msg.pose.position.y=x.y();
        msg.pose.position.z=x.z();

        tf::quaternionTFToMsg(quat,msg.pose.orientation);

        pub.publish(msg);
        rate.sleep();
        steps++;
    }


}