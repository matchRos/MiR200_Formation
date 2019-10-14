#! /usr/bin/env python

import rospy
import class_controller


if __name__=="__main__":    
    
   ctr=class_controller.PathPlannerSlave(frequenzy=20,topic_name="cmd_vel",distance=rospy.get_param("distance"))
   ctr.link_input_topic(rospy.get_param("input_slave"))
   ctr.link_output_topic(rospy.get_param("output_slave"))
   ctr.set_location(rospy.get_param("is_left"))
   ctr.execute()