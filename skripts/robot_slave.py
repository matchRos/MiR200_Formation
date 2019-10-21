#! /usr/bin/env python

import rospy
import class_controller
import numpy as np
import sys


if __name__=="__main__":    
   ctr=class_controller.PathPlannerSlave(node_name=sys.argv[1],position=np.array([float(sys.argv[4]),float(sys.argv[5]),0.0]))
   ctr.set_frequenzy(rospy.get_param("/frequency"))
   ctr.link_input_topic(sys.argv[2])
   ctr.link_output_topic(sys.argv[3])
   ctr.execute()