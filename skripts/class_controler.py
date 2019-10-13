#! /usr/bin/env python


import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point


class RobotController(rospy.Publisher):
    #Initialize a controler, with repsect to:
    #       name: The name of the node the controller is running at
    #       frequenzy: The publishing rate the controller is publishing at its main topic
    #       topic: the name of its main topic
    #       multi_node: Flag if the controller should run as a single instanze or multi instanze (see Anonymous node)
		
	def __init__(	self,
			node_name="my_robot_controller",
			frequenzy=10,
			queue_size=10,
			message_type=Twist,
			topic_name="/ctr",
			multi_node=True):
		self.message_type=message_type
		self.node_name=node_name
		self.topic_name=topic_name
		self.frequenzy=frequenzy
		self.msg=message_type()		
		self.multi_node=multi_node
		self.queue_size=queue_size
		rospy.Publisher.__init__(self,topic_name,message_type,queue_size=self.queue_size)

        # if execute is called the controller is running, publishing its data with respect to its settings
		
	def execute(self):	
		rospy.init_node(self.node_name,anonymous=self.multi_node)
		rate=rospy.Rate(self.frequenzy)	
		while not rospy.is_shutdown():
			self.publish(self.msg)
			rate.sleep()






#General PathPlanner. Provides an interface for any Pathplanner. Spezial Pathplanner can be developed by using this interface and overload the path_planning function
class PathPlanner(RobotController):	
	def __init__(	self,
			node_name="my_path",
			frequenzy=10,
			queue_size=10,
			message_type=Twist,
			topic_name="/path",
			multi_node=True):
		RobotController.__init__(self,node_name,frequenzy,queue_size,message_type,topic_name,multi_node)
		self.time_stamp=np.float64(1/np.float64(self.frequenzy))
	
	
	# Dummie funktion for path planning procedure
	def path_planning(self):
		return
	def execute(self):
		rospy.init_node(self.node_name,anonymous=self.multi_node)
		rate=rospy.Rate(self.frequenzy)
		rospy.loginfo("Initialized Pathplanner with "+str(self.time_stamp)+" seconds time stamp!")
		while not rospy.is_shutdown():
			self.path_planning()
			self.publish(self.msg)
			rate.sleep()







#Class for a Pathplanner that calculates a circular path and messages corresponding velocity for every timestamp
class VelocityPathPlannerCircular(PathPlanner):
	#Initialize Object withe parameters of Robot_Controller/Pathplanner
	def __init__(	self,
					radius=1,
					omega=1,		
					node_name="my_circle_path",
					frequenzy=10,
					queue_size=10,
					topic_name="/circle_path",
					multi_node=True):	
		#Call the contructor of parent class
		PathPlanner.__init__(self,node_name,frequenzy,queue_size,Twist,topic_name,multi_node)
		# Attributes of a circle deskription:
		#		phi:Angle between current position and reference axis
		#		omega: Angular velocitie of movement
		#		radius: Radius of the planned circle
		# example: unity circle with 1rad/sec omega=1 radius=1
		self.phi=np.float64(0.0)
		self.omega=omega
		self.radius=radius	
	
	#planning the cicle trajectory (velocities)
	def path_planning(self):
		self.msg.linear.x=self.omega*self.radius
		self.msg.angular





class VelocityPathPlannerRectangular(PathPlanner):
	def __init__(	self,
					lx=2,
					ly=2,	
					velocity=1,
					omega=1.15,	
					node_name="my_rectangular_path",
					frequenzy=10,
					queue_size=10,
					topic_name="rectangular_path",
					multi_node=True):
		PathPlanner.__init__(self,node_name,frequenzy,queue_size,Twist,topic_name,multi_node)
		self.v_max=velocity
		self.omega=omega
		self.ang_v_max=omega
		self.lx=lx
		self.ly=ly
		self.position=Twist()
		self.turn=False
	
	def path_planning(self):
		if self.turn:
			#Turn around by 90
			#			self.msg.linear.x=0.0
			#calculate_next angular step and differecne to target (90 deg)
			self.msg.linear.x=0.0
			deg=self.omega*self.time_stamp
			dist_deg=np.pi/2-self.position.angular.z
			if dist_deg<deg:
				# do remaining turn to target angle
				self.msg.angular.z=dist_deg/deg*self.omega
				self.turn=False
				self.position.angular.z=0.0
			else:
				# do complete turning step
				self.msg.angular.z=self.omega
				self.position.angular.z+=deg
		else:
			#stop turning and move forward
			self.msg.angular.z=0.0
			x=self.v_max*self.time_stamp
			diff_x=self.lx-self.position.linear.x
			if diff_x<x:
				#do remaining movement
				self.msg.linear.x=self.v_max*diff_x/x
				self.position.linear.x=0.0
				self.turn=True
			else:
				self.msg.linear.x=self.v_max
				self.position.linear.x+=x

		
	

if __name__=="__main__":
	controller=VelocityPathPlannerRectangular(topic_name='cmd_vel')	
	controller.execute()
		
