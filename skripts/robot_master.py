#! /usr/bin/env python

import rospy
import class_controller
import sys

if __name__=="__main__":    
   ctr=class_controller.RobotController(frequenzy=5)
   ctr.link_input_topic(sys.argv[1])
   ctr.link_output_topic(sys.argv[2])
   ctr.execute()