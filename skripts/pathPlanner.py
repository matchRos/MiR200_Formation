#! /usr/bin/env python

import rospy
import class_controller
import sys

if __name__=="__main__": 
    pp_type=sys.argv[3]  
         
    if pp_type=="quad":
         pp=class_controller.PathPlannerQuadratic()
    else:
        pp=class_controller.RobotController()
   
    pp.set_frequenzy(rospy.get_param("/frequenzy"))
    pp.link_input_topic(sys.argv[1])
    pp.link_output_topic(sys.argv[2])
    pp.execute()