#! /usr/bin/env python

import rospy
import class_controller

if __name__=="__main__":    
   ctr=class_controller.PathPlannerQuadratic(frequenzy=5)
   ctr.link_input_topic(rospy.get_param("input_master"))
   ctr.link_output_topic(rospy.get_param("output_master"))
   ctr.execute()