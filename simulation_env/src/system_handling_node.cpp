#include <ros/ros.h>
#include <multi_robot_msgs/SetInitialPose.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/tf.h>
#include <simulation_env/planner.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"SystemHandler");
    ros::NodeHandle nh;

    ros::ServiceClient set_gazebo_state=nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    set_gazebo_state.waitForExistence();
    
    
    std::vector<std::string> names;
    nh.getParam ( "/formation/names", names);
  
    tf::Pose reference;
    for(int i=0;i<names.size();i++)
    {
        ros::ServiceClient set_robot_state=nh.serviceClient<multi_robot_msgs::SetInitialPose>(names.at(i)+"/set_pose");
        set_robot_state.waitForExistence();
        std::string param="formation/"+names.at(i)+"/pose";
        std::vector<double> pose_vec;
        nh.getParam(param,pose_vec);    

        tf::Pose pose(tf::createQuaternionFromRPY(0,0,pose_vec.at(3)),tf::Vector3(pose_vec[0],pose_vec[1],pose_vec[2])); 
        if(i==0)
        {
            reference=pose;
        }      

        multi_robot_msgs::SetInitialPose call_pose;
        tf::poseTFToMsg(pose,call_pose.request.initial_pose);
        set_robot_state.call(call_pose);

        gazebo_msgs::SetModelState state;
        state.request.model_state.model_name=names.at(i);
        state.request.model_state.reference_frame="/map";
        tf::poseTFToMsg(pose, state.request.model_state.pose);
        set_gazebo_state.call(state);        
    }

    //Choose planner
    ros::NodeHandle nh_planner("planner");
    Planner* planner;
    int type;    
    nh.getParam("/planner/type",type);
    ROS_INFO("Got parameter /planner/type: %i",type);
    switch(type)
    {
        case 0:
        {
            ROS_INFO("Starting LissajousPlanner!"); 
            planner=new LissajousPlanner(nh_planner);
            break;
        }
        case 1:
        {
             ROS_INFO("Starting Circle Planner!");  
            planner=new CirclePlanner(nh_planner);
            break;
        }
      

    }
   
    planner->set_start_pose(reference);
    planner->load();
    
    ros::Duration(2).sleep();
    bool pause;
    nh.getParam("planner/pause",pause);
    ROS_INFO("Got Parameter /planner/pause: %d",pause);
    if(!pause)
    {
        planner->start();
    }    
    ros::spin();
    delete planner;
}