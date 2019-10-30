#include <slave/slave.h>

Slave::Slave(ros::NodeHandle &nh):nh(nh)
{

    set_name("my_slave");
    set_master_name("my_master");

    set_orientation(0.0,0.0,0.0);
    set_position(0.0,0.0,0.0);

    set_translation(0.0,0.0,0.0);
    set_rotation(0.0,0.0,0.0);

    this->output=this->nh.advertise<geometry_msgs::Twist>("out",10);
    this->input=this->nh.subscribe("in",10,&Slave::input_callback,this);

    //Parameters that have to be implented more dynamic later
    this->frequenzy=10;
    this->step=1/frequenzy;

} 


void Slave::set_name(std::string name)
{
    this->name=name;
    this->nh.resolveName(this->name);
}
    
void Slave::set_master_name(std::string name)
{
    this->master_name=name;
}

//################################################################################################
//Setter for states

void Slave::set_cart_state(double x,double y,double z,Slave::cart_state &state)
{
    state.x=x;
    state.y=y;
    state.z=z;
}


void Slave::set_orientation(double x,double y,double z)
{
    this->set_cart_state(x,y,z,this->cart_ori);
}


void Slave::set_position(double x,double y,double z)
{
    this->set_cart_state(x,y,z,this->cart_pos);
}

void Slave::set_reference(double x,double y,double z)
{
    this->set_cart_state(x,y,z,this->ref_pos);
}

void Slave::set_translation(double x,double y,double z)
{
    this->set_cart_state(x,y,z,this->cart_vel_in);
}

void Slave::set_rotation(double x,double y,double z)
{
    this->set_cart_state(x,y,z,this->cart_rot_in);
}




//################################################################################################
//Linking important topics/transformations

void Slave::link_input(std::string topic_name)
{
    this->input.shutdown();
    ROS_INFO("Linking input %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->input=this->nh.subscribe(topic_name,10,&Slave::input_callback,this);
}

void Slave::link_output(std::string topic_name)
{
    this->output.shutdown();
    ROS_INFO("Linking output %s to topic: %s \n",this->name.c_str(),topic_name.c_str());
    this->output=this->nh.advertise<geometry_msgs::Twist>(topic_name,10);
}

void Slave::link_transform(std::string transform_name1,std::string transform_name2) 
{
    this->reference=transform_name1;
    this->reference=transform_name2;
}




//################################################################################################
//callback methods

void Slave::input_callback(geometry_msgs::Twist msg)
{
    this->set_translation(msg.linear.x,msg.linear.y,msg.linear.z);
    this->set_rotation(msg.angular.x,msg.angular.y,msg.angular.z);

    this->optimal_control();
    this->forward_propagation();
}

void Slave::odom_callback(nav_msgs::Odometry msg)
{   
    double x=msg.pose.pose.position.x;
    double y=msg.pose.pose.position.y;
    double z=msg.pose.pose.position.z;
    
    double x_o=msg.pose.pose.orientation.x;
    double y_o=msg.pose.pose.orientation.y;
    double z_o=msg.pose.pose.orientation.z;
    double w=msg.pose.pose.orientation.w;

}



void Slave::optimal_control()
{
    //calculate the ideal velocity state
    this->cart_rot.z=this->cart_rot_in.z;
    this->cart_vel.x=this->cart_vel_in.x-this->cart_rot_in.z*this->ref_pos.y;
    this->cart_vel.y=this->cart_vel_in.y+this->cart_rot_in.z*this->ref_pos.x;    


    //calculate least sqared state to ideal state
    this->control.v=cos(this->cart_ori.z)*this->cart_vel.x +sin(this->cart_ori.z)*this->cart_vel.y;
    this->control.omega=this->cart_rot_in.z;

    //calculate reached state
    this->global_from_local();
}

void Slave::global_from_local()
{
     this->cart_vel.x=cos(this->cart_ori.z)*this->control.v;
     this->cart_vel.y=sin(this->cart_ori.z)*this->control.v;
     this->cart_rot.z=this->control.omega;
}

void Slave::forward_propagation()
{
    this->cart_pos.x=this->cart_vel.x*this->step;
    this->cart_pos.y=this->cart_vel.y*this->step;
    this->cart_ori.z=this->cart_rot.z*this->step;
}

void Slave::run()
{
    ros::Rate rate(this->frequenzy);
    ROS_INFO("Started controller with name: %s",this->name.c_str());
    while(ros::ok())
    {
        //publish
        geometry_msgs::Twist msg;
        msg.linear.x=this->control.v;
        msg.linear.y=0.0;
        msg.linear.z=0.0;;
        msg.angular.x=0.0;
        msg.angular.y=0.0;
        msg.angular.z=this->control.omega;
        this->output.publish(msg);
        ros::spinOnce();

        rate.sleep();
    }

}