#! /usr/bin/env python


import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point


class RobotController():
    #Initialize a controler, with repsect to:
    #       name: The name of the node the controller is running at
    #       frequenzy: The publishing rate the controller is publishing at its main topic
    #       topic: the name of its main topic
    #       multi_node: Flag if the controller should run as a single instanze or multi instanze (see Anonymous node)
	def income(self,msg):
		self.msg_in=msg

	def __init__(	self,
					node_name="my_robot_controller",
					frequenzy=10,
					queue_size=10,
					message_type=Twist,
					topic_name="/ctr"):

		self.node_name=node_name
		self.topic_name=topic_name
		self.frequenzy=frequenzy
		self.msg_out=message_type()	
		self.msg_in=message_type()	
		self.queue_size=queue_size		
		self.pub=rospy.Publisher(topic_name+"_out",message_type,queue_size=self.queue_size)
		self.sub=rospy.Subscriber(topic_name+"_in",message_type,self.income)

	



#General PathPlanner. Provides an interface for any Pathplanner. Spezial Pathplanner can be developed by using this interface and overload the path_planning function
class PathPlanner(RobotController):	
	def __init__(	self,
					node_name="my_path",
					frequenzy=10,
					queue_size=10,
					message_type=Twist,
					topic_name="/path"):
		RobotController.__init__(self,node_name,frequenzy,queue_size,message_type,topic_name)
		self.time_stamp=np.float64(1/np.float64(self.frequenzy))


	# Dummie funktion for path planning procedure
	def path_planning(self):
		return

	def execute(self):
		rospy.init_node(self.node_name)
		rate=rospy.Rate(self.frequenzy)
		rospy.loginfo("Initialized Pathplanner"+self.node_name+ " with "+str(self.time_stamp)+" seconds time stamp!")
		
		while not rospy.is_shutdown():
			self.path_planning()
			self.pub.publish(self.msg_out)
			rate.sleep()







#Class for a Pathplanner that calculates a circular path and messages corresponding velocity for every timestamp
class PathPlannerCircular(PathPlanner):
	#Initialize Object withe parameters of Robot_Controller/Pathplanner
	def __init__(	self,
					radius=1,
					omega=1,		
					node_name="my_circle_path",
					frequenzy=10,
					queue_size=10,
					topic_name="/circle_path"):	
		#Call the contructor of parent class
		PathPlanner.__init__(self,node_name,frequenzy,queue_size,Twist,topic_name)
		
		# Attributes of a circle deskription:
		#		phi:Angle between start of turn and current orientation
		#		omega: Angular velocity of movement
		#		radius: Radius of the planned circle
		# example: unity circle with 1rad/sec -> omega=1 radius=1
		self.phi=np.float64(0.0)
		self.omega=omega
		self.radius=radius	
	
	#planning the cicle trajectory (velocities)
	def path_planning(self):
		self.msg_out.linear.x=self.omega*self.radius
		self.msg_out.angular=self.omega





class PathPlannerRectangular(PathPlanner):
	def __init__(	self,
					lx=2,
					ly=2,	
					velocity=1,
					omega=1.15,	
					node_name="my_rectangular_path",
					frequenzy=10,
					queue_size=10,
					topic_name="rectangular_path"):
		PathPlanner.__init__(self,node_name,frequenzy,queue_size,Twist,topic_name)
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
			#			self.msg_out.linear.x=0.0
			#calculate_next angular step and differecne to target (90 deg)
			self.msg_out.linear.x=0.0
			deg=self.omega*self.time_stamp
			dist_deg=np.pi/2-self.position.angular.z
			if dist_deg<deg:
				# do remaining turn to target angle
				self.msg_out.angular.z=dist_deg/deg*self.omega
				self.turn=False
				self.position.angular.z=0.0
			else:
				# do complete turning step
				self.msg_out.angular.z=self.omega
				self.position.angular.z+=deg
		else:
			#stop turning and move forward
			self.msg_out.angular.z=0.0
			x=self.v_max*self.time_stamp
			diff_x=self.lx-self.position.linear.x
			if diff_x<x:
				#do remaining movement
				self.msg_out.linear.x=self.v_max*diff_x/x
				self.position.linear.x=0.0
				self.turn=True
			else:
				self.msg_out.linear.x=self.v_max
				self.position.linear.x+=x

class PathPlannerSlave(PathPlanner):
	def __init__(	self,
					distance=1.0,
					node_name="my_rectangular_path",
					frequenzy=10,
					queue_size=10,
					topic_name="rectangular_path"):
		PathPlanner.__init__(self,node_name,frequenzy,queue_size,Twist,topic_name)
		self.distance=distance
	
	def path_planning(self):
		self.msg_out.angular.z=self.msg_in.angular.z
		self.msg_out.linear.x=self.msg_in.angular.z*self.distance+self.msg_in.linear.x

		


	

if __name__=="__main__":
	controller=PathPlannerSlave(topic_name='cmd_vel')	
	controller.execute()
		
