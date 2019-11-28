#include <simulation_env/planner.h>


//Implementation of a generic planner class########################################################################################
//#################################################################################################################################
Planner::Planner(ros::NodeHandle &nh):nh(nh)
{
    this->tim_sampling=this->nh.createTimer(ros::Duration(0.05),&Planner::plan,this);
    this->pub_current_pose=nh.advertise<geometry_msgs::PoseStamped>("/trajectory",10);
    this->set_start_service=nh.advertiseService("start_planner",&Planner::srv_start,this);
    //this->set_start_service=nh.advertiseService("pause_planner",&Planner::srv_start,this);

    this->paused=ros::Duration(0,0);
    this->frame_name="/map";
    tf::Quaternion quat;
    quat.setRPY(0,0,0);
    tf::Vector3 vec(0,0,0);
    this->planned_pose.setOrigin(vec);
    this->planned_pose.setRotation(quat);
    this->start_reference.setIdentity();
    this->is_planning=false;
}


void Planner::plan(const ros::TimerEvent& event)
{
    ros::Duration local_time=ros::Time::now()-this->start_time-this->paused;
    if(this->is_planning)
    {
        ROS_INFO("%lf",local_time.toSec());
        this->planned_pose=this->get_current_pose(local_time);
        this->planned_pose=this->start_reference*this->planned_pose;
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
bool Planner::srv_start(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    if(req.data)
    {
        this->start();
    }
    else
    {
        this->pause();
    }
    return true;
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

void Planner::set_start_pose(tf::Pose pose)
{
    tf::Pose start_pose_plan;
   
    start_pose_plan=this->get_current_pose(ros::Duration(0));
    ROS_INFO("Start Pose: %lf %lf %lf", start_pose_plan.getOrigin().x(),
                                        start_pose_plan.getOrigin().y(),
                                        tf::getYaw(start_pose_plan.getRotation()));
    this->start_reference=tf::Transform (start_pose_plan.inverseTimes(pose));
    ROS_INFO("Trafo: %lf ",tf::getYaw(start_reference.getRotation()));
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

    pose.setRotation(quat.normalize());

    return pose;    
}


//Implementation of a Planner that gives a Euler spiral########################################################################################
//#############################################################################################################################################

LissajousPlanner::LissajousPlanner(ros::NodeHandle &nh):Planner(nh)
{

}

tf::Pose LissajousPlanner::get_current_pose(ros::Duration time)
{
    double t=time.toSec();
    tf::Pose pose;
    tf::Vector3 r(0,0,0);
    r.setX(this->plan.Ax*sin(this->plan.omegax*t));
    r.setY(this->plan.Ay*sin(this->plan.omegax*this->plan.ratio*t+this->plan.dphi));

    tf::Vector3 tangent;
    tangent.setX(this->plan.Ax*cos(this->plan.omegax*t)*this->plan.omegax);
    tangent.setY(this->plan.Ay*cos(this->plan.omegax*this->plan.ratio*t+this->plan.dphi)*this->plan.omegax*this->plan.ratio);
    tf::Quaternion quat;
    quat.setRPY(    0,
                    0,
                    atan2(tangent.y(),tangent.x()));

    pose.setOrigin(r);   

    pose.setRotation(quat);    

    return pose;

}

void LissajousPlanner::set_parameter(float omegax,float dphi,int ratio,float Ax,float Ay)
{
    this->plan.Ax=Ax;
    this->plan.Ay=Ay;
    this->plan.omegax=omegax;
    this->plan.dphi=dphi;
    this->plan.ratio=ratio;
}