#include <simulation_env/planner.h>

//Implementation of a generic planner class########################################################################################
//#################################################################################################################################
Planner::Planner(ros::NodeHandle &nh):nh(nh)
{
    this->frame_name="/map";

    this->tim_sampling=this->nh.createTimer(ros::Duration(0.05),&Planner::plan,this);
    
    this->pub_current_odometry=nh.advertise<nav_msgs::Odometry>("/trajectory_odom",10);
    this->set_start_service=nh.advertiseService("start_planner",&Planner::srv_start,this);
    this->set_stop_service=nh.advertiseService("stop_planner",&Planner::srv_stop,this);

    this->paused=ros::Duration(0,0);
    this->start_reference.setIdentity();
    this->is_planning=false;
    this->iterations=1;
    this->iterations_counter=0;
    
    this->start_pose=tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0.0,0.0,0.0));
    this->vel=tf::Vector3(0,0,0);
    this->pos=tf::Vector3(0,0,0);
    this->ang_vel=0.0;
    this->orientation=tf::createQuaternionFromYaw(0);
}


void Planner::plan(const ros::TimerEvent& event)
{
    if(this->iterations_counter==this->iterations)
    {
        stop();
    }
    if(this->is_planning)
    {
        ros::Duration local_time;
        local_time=ros::Time::now()-this->start_time-this->paused;        
        this->vel=this->get_velocity(local_time);
        this->pos=this->get_position(local_time);
        this->orientation=this->get_orientation(local_time);
        this->ang_vel=this->get_angular_velocity(local_time);
        this->check_period(local_time);
    }
    else
    {
        this->vel=tf::Vector3(0,0,0);
        this->ang_vel=0;
    }
    this->transform_values(this->start_reference);
    this->publish();
}


void Planner::start()
{
    ROS_INFO("Started planner: %s",ros::this_node::getName().c_str());
    this->start_time=ros::Time::now();
    this->is_planning=true;
    this->start_reference=this->get_transform(this->start_pose);
}

void Planner::pause()
{
    this->is_planning=false;
    this->is_paused=true;
}

bool Planner::srv_start(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    if(this->is_planning)
    {this->pause();}
    else
    {this->start();}
}

void Planner::stop()
{
    ROS_INFO("Shutting down node: %s",ros::this_node::getName().c_str());
    ros::shutdown();
}
bool Planner::srv_stop(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    this->stop();
    return true;
}

void Planner::set_start_pose(tf::Pose pose)
{
    this->start_pose=pose;
}

tf::Transform Planner::get_transform(tf::Pose pose)
{   
    tf::Pose planner_start( this->get_orientation(ros::Duration(0)),
                            this->get_position(ros::Duration(0)));

    ROS_INFO(   "Planner starts at: x-%lf y-%lf theta-%lf",
                planner_start.getOrigin().x(),
                planner_start.getOrigin().y(),
                tf::getYaw(planner_start.getRotation()));
    
    tf::Transform trafo(planner_start.inverseTimes(pose));

    ROS_INFO(   "Trafo applied on planner values: x-%lf y-%lf z-%lf x-%lf y-%lf z-%lf w-%lf theta-%lf ",
                this->start_reference.getOrigin().x(),
                this->start_reference.getOrigin().y(),
                this->start_reference.getOrigin().z(),
                this->start_reference.getRotation().x(),
                this->start_reference.getRotation().y(),
                this->start_reference.getRotation().z(),
                this->start_reference.getRotation().w(),
                tf::getYaw(this->start_reference.getRotation()));
    return trafo;
}

void Planner::transform_values(tf::Transform trafo)
{
    tf::Transform rotate(trafo.getRotation());
    this->vel=rotate*this->vel;
    this->pos=trafo*this->pos;
    this->orientation=rotate*this->orientation;
}

void Planner::publish()
{    
    nav_msgs::Odometry msg2;
    msg2.header.frame_id=this->frame_name;
    msg2.header.stamp=ros::Time::now();
    tf::Pose pose(this->orientation,this->pos);
    tf::poseTFToMsg(pose,msg2.pose.pose);
    tf::vector3TFToMsg(this->vel,msg2.twist.twist.linear);
    msg2.twist.twist.angular.z=this->ang_vel;    
    this->pub_current_odometry.publish(msg2);
}









//Implementation of a Planner that gives a circle########################################################################################
//#######################################################################################################################################
CirclePlanner::CirclePlanner(ros::NodeHandle &nh):Planner(nh)
{
 
}
void CirclePlanner::set_parameter(double r, double omega)
{
    this->plan.r=r;
    this->plan.omega=omega;
}


tf::Vector3 CirclePlanner::get_position(ros::Duration time)
{
    double t=time.toSec();    
    tf::Vector3 pos(sin(this->plan.omega*t)*this->plan.r,
                    -cos(this->plan.omega*t)*this->plan.r,
                    0);
    return pos;

}

tf::Quaternion CirclePlanner::get_orientation(ros::Duration time)
{
    return tf::createQuaternionFromYaw(this->plan.omega*time.toSec());
}

tf::Vector3 CirclePlanner::get_velocity(ros::Duration time)
{
    double t=time.toSec();
    tf::Vector3 vel(cos(this->plan.omega*t)*this->plan.r*this->plan.omega,
                    sin(this->plan.omega*t)*this->plan.r*this->plan.omega,
                    0);
    return vel;
}

double CirclePlanner::get_angular_velocity(ros::Duration time)
{
    return this->plan.omega;    
}

void CirclePlanner::load()
{
    try{
        this->nh.getParam(PARAM_ITERATION,this->iterations);
        this->nh.getParam(PARAM_R,this->plan.r);
        this->nh.getParam(PARAM_OMEGA,this->plan.omega);
        ROS_INFO("Loaded %s : Radius: %f Omega: %f",    ros::this_node::getName().c_str(),
                                                        this->plan.r,
                                                        this->plan.omega);
    }
    catch(...)
    {
        ROS_INFO("Error due loading paramter of: %s",ros::this_node::getName().c_str());
    }
   
}

void CirclePlanner::check_period(ros::Duration time)
{
    if(this->plan.omega*time.toSec()>2*M_PI)
    {
        this->iterations_counter++;
    }
}




//Implementation of a Planner that gives a Lissajous figures###################################################################################
//#############################################################################################################################################
LissajousPlanner::LissajousPlanner(ros::NodeHandle &nh):Planner(nh)
{
}


tf::Vector3 LissajousPlanner::get_position(ros::Duration time)
{
    double t;
    t=time.toSec();
    tf::Vector3 r(0,0,0);
    double x;
    x=this->plan.Ax*sin(this->plan.omegax*t);
    double y;
    y=this->plan.Ay*sin(this->plan.omegax*this->plan.ratio*t+this->plan.dphi);

    return tf::Vector3(x,y,0);
}

tf::Vector3 LissajousPlanner::get_velocity(ros::Duration time)
{
    double t=time.toSec();
    double dx=this->plan.Ax*cos(this->plan.omegax*t)*this->plan.omegax;
    double dy=this->plan.Ay*cos(this->plan.omegax*this->plan.ratio*t+this->plan.dphi)*this->plan.omegax*this->plan.ratio;
    tf::Vector3 vel(dx,
                    dy,
                    0);
    return vel;
}

tf::Quaternion LissajousPlanner::get_orientation(ros::Duration time)
{
    double t=time.toSec();
    tf::Vector3 tangent;
    tangent.setX(this->plan.Ax*cos(this->plan.omegax*t)*this->plan.omegax);
    tangent.setY(this->plan.Ay*cos(this->plan.omegax*this->plan.ratio*t+this->plan.dphi)*this->plan.omegax*this->plan.ratio);
    return tf::createQuaternionFromYaw(atan2(tangent.y(),tangent.x()));

}

double LissajousPlanner::get_angular_velocity(ros::Duration time)
{
    tf::Vector3 pos=this->get_position(time);
    tf::Vector3 vel=this->get_velocity(time);
    return (pos.x()*vel.y()-pos.y()*vel.x())/(pow(pos.x(),2)+pow(pos.y(),2));
}


void LissajousPlanner::set_parameter(float omegax,float dphi,int ratio,float Ax,float Ay)
{
    this->plan.Ax=Ax;
    this->plan.Ay=Ay;
    this->plan.omegax=omegax;
    this->plan.dphi=dphi;
    this->plan.ratio=ratio;
}

void LissajousPlanner::load()
{
    this->nh.getParam(PARAM_ITERATION,this->iterations);
    this->nh.getParam(PARAM_AMP_X,this->plan.Ax);
    this->nh.getParam(PARAM_AMP_Y,this->plan.Ay);
    this->nh.getParam(PARAM_RATIO,this->plan.ratio);
    this->nh.getParam(PARAM_PHASE,this->plan.dphi);
    this->nh.getParam(PARAM_OMEGA,this->plan.omegax);
    ROS_INFO("Loaded %s : AmplifierX %lf AmplifierY %lf Ratio %i PhaseShift %lf Omega: %lf ",   
                                                    ros::this_node::getName().c_str(),
                                                    this->plan.Ax,
                                                    this->plan.Ay,
                                                    this->plan.ratio,
                                                    this->plan.dphi,
                                                    this->plan.omegax);
}

void LissajousPlanner::check_period(ros::Duration time)
{
    if(this->plan.omegax*time.toSec()>2*M_PI)
    {
        this->iterations_counter++;
    }
}


// ################################################################################################################################################

// ################################################################################################################################################

// ###############################################################################################################################################

ClickedPosePlanner::ClickedPosePlanner(ros::NodeHandle &nh,std::string topic_name):Planner(nh)
{
    this->sub_=this->nh.subscribe<geometry_msgs::PoseStamped>(topic_name,10,boost::bind(&ClickedPosePlanner::clickedCallback,this,_1));
    this->pose_=this->start_reference;
    this->start_reference=tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0.0,0.0,0.0));
}


tf::Vector3 ClickedPosePlanner::get_position(ros::Duration time)
{
    return this->pose_.getOrigin();
}

tf::Vector3 ClickedPosePlanner::get_velocity(ros::Duration time)
{
    if(this->direction_.length()>0.01)
    {
        return this->direction_.normalized();
    }
    else
    {
        return tf::Vector3(0,0,0);
    }
    
}    


tf::Quaternion ClickedPosePlanner::get_orientation(ros::Duration time)
{
  return this->pose_.getRotation();
}

double ClickedPosePlanner::get_angular_velocity(ros::Duration time)
{
    return 0.0;
}

void ClickedPosePlanner::check_period(ros::Duration time)
{
    return;
}

void ClickedPosePlanner::clickedCallback(const geometry_msgs::PoseStampedConstPtr msg)
{
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose,pose);
    this->direction_=pose.getOrigin()-this->pose_.getOrigin();
    this->pose_=pose;
    
}
void ClickedPosePlanner::load()
{
    return;
}



//Implementation of a Planner that gives a circle########################################################################################
//#######################################################################################################################################
Spiralplanner::Spiralplanner(ros::NodeHandle &nh):Planner(nh)
{
 
}
void Spiralplanner::set_parameter(double r_offset,double r_growth, double omega)
{
    this->plan.r_offset=r_offset;
    this->plan.r_growth=r_growth;
    this->plan.omega=omega;
}


tf::Vector3 Spiralplanner::get_position(ros::Duration time)
{
    double t=time.toSec();    
    tf::Vector3 pos(sin(this->plan.omega*t)*(this->plan.r_offset+this->plan.r_growth*this->plan.omega*t),
                    -cos(this->plan.omega*t)*(this->plan.r_offset+this->plan.r_growth*this->plan.omega*t),
                    0);
    return pos;

}

tf::Quaternion Spiralplanner::get_orientation(ros::Duration time)
{
    return tf::createQuaternionFromYaw(this->plan.omega*time.toSec());
}

tf::Vector3 Spiralplanner::get_velocity(ros::Duration time)
{
    double t=time.toSec();
    tf::Vector3 vel(cos(this->plan.omega*t)*(this->plan.r_offset+this->plan.r_growth*this->plan.omega*t)+sin(this->plan.omega*t)*this->plan.r_growth*this->plan.omega,
                    sin(this->plan.omega*t)*(this->plan.r_offset+this->plan.r_growth*this->plan.omega*t)-cos(this->plan.omega*t)*this->plan.r_growth*this->plan.omega,
                    0.0);
    return vel;
}

double Spiralplanner::get_angular_velocity(ros::Duration time)
{
    return this->plan.omega;    
}

void Spiralplanner::load()
{
    try{
        this->nh.getParam(PARAM_ITERATION,this->iterations);
        this->nh.getParam(PARAM_GROWTH,this->plan.r_growth);
        this->nh.getParam(PARAM_OMEGA,this->plan.omega);
        this->nh.getParam(PARAM_R0,this->plan.r_offset);
        ROS_INFO("Loaded %s : R_OFfset: %f  r_growth %f Omega: %f",    
                                                        ros::this_node::getName().c_str(),
                                                        this->plan.r_offset,
                                                        this->plan.r_growth,
                                                        this->plan.omega);
    }
    catch(...)
    {
        ROS_INFO("Error due loading paramter of: %s",ros::this_node::getName().c_str());
    }
   
}

void Spiralplanner::check_period(ros::Duration time)
{
    if(this->plan.omega*time.toSec()>2*M_PI)
    {
        this->iterations_counter++;
    }
}


