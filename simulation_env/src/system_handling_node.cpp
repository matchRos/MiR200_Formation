#include <ros/ros.h>
#include <multi_robot_msgs/SetInitialPose.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/tf.h>
#include <simulation_env/planner.h>

Planner* planner;
tf::Pose reference;


std::vector<std::string> getRobotNames(ros::NodeHandle nh)
{
    std::vector<std::string> names;   
    if(!nh.getParam ( "/formation/names", names))
    {
        ROS_WARN("Could not load %s",nh.resolveName( "/formation/names").c_str());
        throw(std::invalid_argument(std::string("Could not load")+nh.resolveName( "/formation/names")));
    }
    
    return names;
}


void setReferences(ros::NodeHandle nh)
{
    //Wait for the ros set model state service
    ros::ServiceClient set_gazebo_state=nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    set_gazebo_state.waitForExistence();
    
    
    std::vector<std::string> names=getRobotNames(nh);

    for(int i=0;i<names.size();i++)
    {
        //Wait for the controller set state service
        ros::ServiceClient set_robot_state=nh.serviceClient<multi_robot_msgs::SetInitialPose>(names.at(i)+"/controller/set_reference");
        set_robot_state.waitForExistence();

        //Get the reference parameters (vectors x y z r p y)
        std::string param="/"+names.at(i)+"/controller/reference";
        std::vector<double> pose_vec;
        if(!nh.getParam(param,pose_vec))
        {
            ROS_WARN("Could not load %s",nh.resolveName("/"+names.at(i)+"/controller/reference").c_str());
        }
        
        tf::Pose pose(tf::createQuaternionFromRPY(pose_vec.at(3),pose_vec.at(4),pose_vec.at(5)),tf::Vector3(pose_vec[0],pose_vec[1],pose_vec[2])); 
        
        //Save pose of the first( the master robot)
        if(i==0)
        {
            reference=pose;
        }      

        //Call the service for setting the controller pose
        multi_robot_msgs::SetInitialPose call_pose;
        tf::poseTFToMsg(pose,call_pose.request.initial_pose);
        set_robot_state.call(call_pose);

        //Same with gazebo pose
        gazebo_msgs::SetModelState state;
        state.request.model_state.model_name=names.at(i);
        state.request.model_state.reference_frame="/map";
        tf::poseTFToMsg(pose, state.request.model_state.pose);
        set_gazebo_state.call(state);        
    }
}

void startPlanner(ros::NodeHandle nh)
{
    //Choose planner
    ros::NodeHandle nh_planner("planner");
    
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
        case 2:
        {
            ROS_INFO("Starting Clicked Pose Planner!");
            planner=new ClickedPosePlanner(nh_planner,"/move_base_simple/goal");
            break;
        }
        case 3:
        {
            ROS_INFO("Starting Spiral Planner");
            planner=new Spiralplanner(nh_planner);
            break;
        }
        default:
        {
            ROS_ERROR("Wrong planner type choosen!");
        }
      

    }


    try
    {
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
    }
    catch(std::exception &e)
    {
        ROS_WARN("%s",e.what());
    }
}

void linkObject(ros::NodeHandle nh)
{
    std::vector<std::string> names=getRobotNames(nh);
    ros::ServiceClient attach_client=nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    attach_client.waitForExistence();

    gazebo_ros_link_attacher::Attach attach;
    attach.request.model_name_1=names[0];
    attach.request.link_name_1="base_footprint";

    attach.request.model_name_2="object";
    attach.request.link_name_2="object_link";

    attach.request.joint_type="fixed";

    attach_client.call(attach);     
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"SystemHandler");
    ros::NodeHandle nh;

    bool linkage=false;
    bool plan=false;
    bool set_references=false;

    std::vector<std::string> args(argv, argv+argc);
    for (size_t i = 1; i < args.size(); ++i) 
    {
        if (args[i] == "-link") 
        {
            linkage=true;

        }
        else if(args[i]=="-plan")
        {
            plan=true;
        }
        else if(args[i]=="-reference")
        {
            set_references=true;
        }
    }

    if(set_references)
    {
        ROS_INFO("Setting system references!");
        setReferences(nh);
    }
    if(linkage)
    {
        ROS_INFO("Linking object to system!");
        linkObject(nh);
    }
    if(plan)
    {
        ROS_INFO("Starting system planner!");
        startPlanner(nh);
    }
   

    ros::spin();
    delete planner;
}