#! /usr/bin/env python

import rospy
import class_controller
import numpy as np
import sys


if __name__=="__main__":    
   ctr=class_controller.PathPlannerSlave(frequenzy=20,position=np.array([float(sys.argv[3]),float(sys.argv[4]),0.0]))
   ctr.link_input_topic(sys.argv[1])
   ctr.link_output_topic(sys.argv[2])
   ctr.execute()