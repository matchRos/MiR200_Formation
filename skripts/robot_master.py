#! /usr/bin/env python

import rospy
import class_controller
import sys

if __name__=="__main__": 
    ctr=class_controller.PathPlannerMaster(node_name="robot_master")   
    ctr.link_input_topic(sys.argv[1])
    ctr.link_output_topic(sys.argv[2])
    ctr.set_frequenzy(rospy.get_param("/frequency"))
    for name in sys.argv[3:]:
        if name[0:2]!="__":
            ctr.add_slave(name)
    

    ctr.execute()