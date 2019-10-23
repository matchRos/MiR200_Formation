#! /usr/bin/env python

import rospy
import planner
import controller
import sys

if __name__=="__main__": 
    pp_type=sys.argv[3]  
    if pp_type=="quad":
         pp=planner.PathPlannerQuadratic()
    else:
        pp=controller.RobotController()
    
    pp.set_frequenzy(rospy.get_param("/frequency"))
    pp.link_input_topic(sys.argv[1])
    pp.link_output_topic(sys.argv[2])
    pp.execute()