#! /usr/bin/env python

import rospy
import class_controller
import numpy as np


if __name__=="__main__":    
   
    x=rospy.get_param("robot_x")
    y=rospy.get_param("robot_y")
   
    ctr=class_controller.PathPlannerSlave(frequenzy=20,topic_name="cmd_vel",position=np.array([x,y,0.0]))
    ctr.link_input_topic(rospy.get_param("input_slave"))
    ctr.link_output_topic(rospy.get_param("output_slave"))
    ctr.set_location(rospy.get_param("is_left"))
    ctr.execute()