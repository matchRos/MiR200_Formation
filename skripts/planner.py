#! /usr/bin/env python
import rospy
from controller import RobotController



#Class for a Pathplanner that calculates a circular path and messages corresponding velocity for every timestamp
class PathPlannerCircular(RobotController):
	#Initialize Object withe parameters of Robot_Controller/Pathplanner
	def __init__(	self,
					radius=1,
					omega=1,		
					node_name="my_circle_path",
					frequenzy=10,
					queue_size=10,					
					topic_name="circle_path"):	
		#Call the contructor of parent class
		RobotController.__init__(self,node_name,frequenzy,queue_size,Twist,topic_name)		
		# Attributes of a circle deskription:
		#		phi:Angle between start of turn and current orientation
		#		omega: Angular velocity of movement
		#		radius: Radius of the planned circle
		# example: unity circle with 1rad/sec -> omega=1 radius=1		
		self.omega=omega
		self.radius=radius	
	
	#planning the cicle trajectory (velocities)
	def path_planning(self):
		self.msg_out.linear.x=self.omega*self.radius
		self.msg_out.angular.z=self.omega



#General class for path planning purpose
class PathPlannerQuadratic(RobotController):
	def __init__(	self,
					l=3,
					velocity=1,
					omega=0.2,	
					node_name="my_rectangular_path",
					frequenzy=10,
					queue_size=10,
					topic_name="rectangular_path"):
		RobotController.__init__(self,node_name,frequenzy,queue_size,Twist,topic_name)
		self.velocity=velocity
		self.omega=omega
		self.l=l
		self.position=Twist()
		self.turn=False	

	def path_planning(self):	

		if self.turn:					
			#calculate next angular step and difference to target (90 deg)
			#stop turning forward
			self.msg_out.linear.x=0.0
			deg=self.omega*self.time_step
			dist_deg=np.pi/2-self.position.angular.z
			
			
			if dist_deg<deg:
				# do remaining turn to target angle and switch to forward mode
				self.msg_out.angular.z=dist_deg/deg*self.omega
				self.turn=False
				self.position.angular.z=0.0
				
			else:
				# do complete turning step
				self.msg_out.angular.z=self.omega
				self.position.angular.z+=deg
			
		

		else:
			#calculate_next linear step and difference to target l
			#stop turning and move forward
			self.msg_out.angular.z=0.0
			x_step=self.velocity*self.time_step
			dist_x=self.l-self.position.linear.x
			if dist_x<x_step:
				#do remaining movement and switch to turning mode
				self.msg_out.linear.x=self.velocity*dist_x/x_step	
				self.turn=True		
				self.position.linear.x=0.0	
			else:
				#do complete movement step
				self.msg_out.linear.x=self.velocity
				self.position.linear.x+=x_step
