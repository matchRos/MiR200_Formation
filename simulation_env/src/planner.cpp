#include <simulation_env/planner.h>


//Implementation of a generic planner class########################################################################################
//#################################################################################################################################
Planner::Planner(ros::NodeHandle &nh):nh(nh)
{
    this->tim_sampling=this->nh.createTimer(ros::Duration(0.05),&Planner::plan,this);
    this->pub_current_pose=nh.advertise<geometry_msgs::PoseStamped>("/trajectory",10);
    this->paused=ros::Duration(0,0);
    this->frame_name="/map";
}


void Planner::plan(const ros::TimerEvent& event)
{
    ros::Duration local_time=ros::Time::now()-this->start_time-this->paused;
    if(this->is_planning)
    {
        this->planned_pose=this->get_current_pose(local_time);
    }
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id=this->frame_name;
    msg.header.stamp=ros::Time::now();
    tf::poseTFToMsg(this->planned_pose,msg.pose);
    this->pub_current_pose.publish(msg);
}


void Planner::start()
{
    this->start_time=ros::Time::now();
    this->is_planning=true;
}
void Planner::stop()
{
    ros::shutdown();
}
void Planner::pause()
{
    this->is_planning=false;
    this->is_paused=true;
}


//Implementation of a Planner that gives a circle########################################################################################
//#######################################################################################################################################

CirclePlanner::CirclePlanner(ros::NodeHandle &nh):Planner(nh)
{
 
}
void CirclePlanner::set_parameter(double r, double x,double y,double omega)
{
    this->plan.r=r;
    this->plan.x=x;
    this->plan.y=y;
    this->plan.omega=omega;
}

tf::Pose CirclePlanner::get_current_pose(ros::Duration time)
{
    double t=time.toSec();

    tf::Pose pose;
    tf::Vector3 pos(sin(this->plan.omega*t)*this->plan.r+this->plan.x,
                    -cos(this->plan.omega*t)*this->plan.r+this->plan.y,
                    0);


    pose.setOrigin(pos);

    tf::Quaternion quat;
    quat.setRPY(0,0,this->plan.omega*t);

    pose.setRotation(quat);

    return pose;    
}