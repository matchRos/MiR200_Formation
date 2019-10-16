#! /usr/bin/env python

import rospy
import class_controller
import sys

if __name__=="__main__": 
    ctr_type=sys.argv[3]
  
         
    if ctr_type=="quad":
         ctr=class_controller.PathPlannerQuadratic()
    else:
        ctr=class_controller.RobotController()
   
    ctr.set_frequenzy(rospy.get_param("/frequenzy"))
    ctr.link_input_topic(sys.argv[1])
    ctr.link_output_topic(sys.argv[2])
    ctr.execute()