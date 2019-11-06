#include <slave/slave.h>

Slave::Slave(ros::NodeHandle &nh):Controller(nh)
{
} 


void Slave::optimal_control()
{   

    //Calcualte ideal velocities
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    
    linear=this->msg_velocities_in.linear;
    angular=this->msg_velocities_in.angular;

    this->msg_velocities_ideal.angular.x=angular.x;
    this->msg_velocities_ideal.angular.y=angular.y;
    this->msg_velocities_ideal.angular.z=angular.z;

    this->msg_velocities_ideal.linear.x=linear.x-angular.z*this->reference.position.y;
    this->msg_velocities_ideal.linear.y=linear.y+angular.z*this->reference.position.x;
    this->msg_velocities_ideal.linear.z=linear.z;
    

    //Calculate controlvector
    double phi=this->current_pose.getRotation().getAngle();
    this->msg_velocities_out.linear.x=cos(phi)*msg_velocities_ideal.linear.x +sin(phi)*this->msg_velocities_ideal.linear.y;
    this->msg_velocities_out.angular.z=angular.z;
}



void Slave::scope()
{
    //publish
    geometry_msgs::Twist msg;
    this->optimal_control();

    this->output.publish(this->msg_velocities_out);

    ros::spinOnce();
}