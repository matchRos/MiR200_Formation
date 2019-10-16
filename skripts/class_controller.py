#! /usr/bin/env python


import rospy
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from rospy.numpy_msg import numpy_msg
import math

#Interface Class for Controllers. Without any further implementations it just gets the input and passes it to output.
class RobotController():  


	#callback procedure wich is called if a new input message occures	
	def income(self,msg):
		self.msg_in=msg


	 #Initialize a controler, with repsect to:
    #       node_name: The name of the node the controller is running at
    #       frequenzy: The publishing rate the controller is publishing at its main topic
	#		queue_size: Size of the output queue
	#		message_type: Objeckt of type Ros-message for definition of input and output message type of the controller
	#       topic: the name of its main topic
	def __init__(	self,
					node_name="my_robot_controller",
					frequenzy=10,
					queue_size=10,
					message_type=Twist,
					topic_name="ctr"):

		self.node_name=node_name		
		self.frequenzy=frequenzy
		self.queue_size=queue_size
		
		self.message_type=message_type

		self.msg_out=message_type()	
		self.msg_in=message_type()	
		

		self.link_input_topic(topic_name+"_in")
		self.link_output_topic(topic_name+"_out")
	

	#Execution prcodeure of the controller. Initialization of the node and the ros-scope runs here. 
	#Therfore publishing runs here and as RobotCorntroller is a parent class the input is passed through as output without doing anything
	def execute(self):
		rospy.init_node(self.node_name)
		rate=rospy.Rate(self.frequenzy)
		rospy.loginfo("Initialized pass trought Controller"+self.node_name+ " at "+str(self.frequenzy)+"Hz!")
		
		while not rospy.is_shutdown():
			self.msg_out=self.msg_in			
			self.pub.publish(self.msg_out)
			rate.sleep()

	#links the source of the controller to the given topic_name
	def link_input_topic(self,topic_name):
		if hasattr(self, 'sub'):
			self.sub.unregister()
		self.sub=rospy.Subscriber(topic_name,self.message_type,self.income)
	#links the sink of the controller to the given topic_name	
	def link_output_topic(self,topic_name):		
		if hasattr(self, 'pub'):
			self.pub.unregister()
		self.pub=rospy.Publisher(topic_name,self.message_type,queue_size=self.queue_size)
	#Setter for Control frequenzy
	def set_frequenzy(self,frequenzy):
	
		self.frequenzy=frequenzy


# General PathPlanner. Provides an interface for any Pathplanner. 
# Spezial Pathplanner can be developed by using this interface and overload the path_planning function

class PathPlanner(RobotController):	
	# Initializes an Pathplanner as child of RobotController and passes arguments to it. 
	# Further a time_stamp for pathplanning purpose is determined and safed as attributes
	def __init__(	self,
					node_name="my_path",
					frequenzy=10,
					queue_size=10,
					message_type=Twist,					
					topic_name="path"):
		RobotController.__init__(self,node_name,frequenzy,queue_size,message_type,topic_name)
		self.time_stamp=np.float64(1/np.float64(self.frequenzy))
		

	# Dummie funktion for path planning procedure. Should be implemented in child class (important: this class should determine the data for msg_out)
	def path_planning(self):
		return

	# execution scope of this class. Initialisation of node and rates is done here and the ros-scope is runnign. 
	# Within it the path_planning procedure is called and the determined data of msg_out is published
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
					topic_name="circle_path"):	
		#Call the contructor of parent class
		PathPlanner.__init__(self,node_name,frequenzy,queue_size,Twist,topic_name)		
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
class PathPlannerQuadratic(PathPlanner):
	def __init__(	self,
					l=3,
					velocity=1,
					omega=0.2,	
					node_name="my_rectangular_path",
					frequenzy=10,
					queue_size=10,
					topic_name="rectangular_path"):
		PathPlanner.__init__(self,node_name,frequenzy,queue_size,Twist,topic_name)
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
			deg=self.omega*self.time_stamp
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
			x_step=self.velocity*self.time_stamp
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


class PathPlannerSlave(PathPlanner):
	def __init__(	self,				
					node_name="my_rectangular_path",
					frequenzy=10,
					position=np.array(3,dtype=np.float64), #numpy vector x,y,z					
					omega=1.0,
					velocity=1.0,
					queue_size=10,
					topic_name="rectangular_path"):
		PathPlanner.__init__(self,node_name,frequenzy,queue_size,Twist,topic_name)
		
		self.omega_max=omega
		self.velocity_max=velocity

		self.omega=0.0
		self.velocity=0.0

		self.phi=0.0

		self.position=position	
		self.distance=np.sqrt(self.position[0]**2+self.position[1]**2)
		self.prepare_roation=False
		self.prepare_translation=False
		
		self.rotate=False
		self.translate=False
		self.forward=True

	
	def path_planning(self):
		self.prepare_roation=not(-0.1<self.msg_in.angular.z<0.1)		
		self.prepare_translation=not(-0.1<self.msg_in.linear.x<0.1)
	
		if self.prepare_roation:
			self.omega=self.omega_max

			alpha=np.arctan2(self.position[1],self.position[0])		#angle between slave x-axis and master x-axis				
			if alpha >=0:				#first or second quadrant
				if alpha <=np.pi/2:			#first quadrant
					alpha=np.pi/2-alpha
					self.omega=-self.omega_max	#turn in negative dircetion
				elif alpha>np.pi/2: 		#second quadrant
					alpha-=np.pi/2
					self.omega=self.omega_max	#turn in positive direction
				else:
					raise ValueError("Slave "+self.node_name+" angle incorrect!")
				self.forward=False			#move backward due rotation
			

			elif alpha<0:				#third or forth quadrant
				if alpha<=-np.pi/2:		#third quadrant
					alpha=-np.pi/2-alpha
					self.omega=-self.omega_max	#turn in negative dircetion
				elif alpha>-np.pi/2:		#fourth quadrant
					alpha=-alpha
					self.omega=self.omega_max	#turn in positive direction
				else:
					raise ValueError("Slave "+self.node_name+" angle incorrect!")
				self.forward=True			#move forward due rotation

			else:							#error handling
					raise ValueError("Slave "+self.node_name+" angle incorrect!")	
				
					
			dist_deg=alpha-self.phi
			deg_step=self.omega_max*self.time_stamp
			if dist_deg<=deg_step:
				self.msg_out.angular.z=self.omega
				if not self.rotate or self.translate:
					self.phi+=dist_deg		
					self.rotate=True
				self.prepare_roation=False					
			else:				
				self.msg_out.angular.z=self.omega	
				self.phi+=deg_step
	


		#Prepare the slave for a rotation around the master therefore it position vektor (relative to the master) and its orientation vektor have to be orthogonal to each other
		elif self.prepare_translation:
			#calculate the proper angle for get above descriped property
			alpha=np.arctan2(self.position[1],self.position[0])		#angle between slave x-axis and master x-axis				
			if alpha >=0:				#first or second quadrant
				if alpha <=np.pi/2:			#first quadrant
					self.omega=self.omega_max	#turn in negative dircetion
				elif alpha>np.pi/2: 		#second quadrant
					self.omega=-self.omega_max	#turn in positive direction
				else:
					raise ValueError("Slave "+self.node_name+" angle incorrect!")			

			elif alpha<0:				#third or forth quadrant
				if alpha<=-np.pi/2:		#third quadrant
					self.omega=+self.omega_max	#turn in negative dircetion
				elif alpha>-np.pi/2:		#fourth quadrant
					self.omega=-self.omega_max	#turn in positive direction
				else:
					raise ValueError("Slave "+self.node_name+" angle incorrect!")				

			else:							#error handling
					raise ValueError("Slave "+self.node_name+" angle incorrect!")	


			dist_deg=self.phi
			deg_step=self.omega_max*self.time_stamp

			if dist_deg<=deg_step:
				self.msg_out.linear.z=self.omega*dist_deg/deg_step
				if not self.rotate or self.translate:
					self.phi-=dist_deg		
					self.translate=True	
				self.prepare_roation=False					
			else:
				self.msg_out.angular.z=-self.omega
				self.phi-=deg_step
		




		if self.translate:
			self.omega=0.0			
			self.velocity=self.msg_in.linear.x			
			self.translate=not(-0.1<self.msg_in.linear.x<0.1) 
		
		elif self.rotate:
			if self.forward:				
				self.omega=self.msg_in.angular.z
				self.velocity=self.omega*self.distance
			else:				
				self.omega=self.msg_in.angular.z
				self.velocity=-self.omega*self.distance
			
			self.rotate=not(-0.1<self.msg_in.angular.z<0.1)
		
		self.msg_out.linear.x=self.velocity
		self.msg_out.angular.z=self.omega
			

		
			
				

class PathPlannerSlavePrimitive(PathPlanner):
	def __init__(	self,
					distance=1.0,
					node_name="my_primitive_pathplanner",
					frequenzy=10,
					queue_size=10,
					position=np.array(3,dtype=np.float64), #numpy vector x,y,z		
					topic_name="primitive_path"):
		PathPlanner.__init__(self,node_name,frequenzy,queue_size,Twist,topic_name)
		self.distance=np.sqrt(self.position[0]**2+self.position[1]**2)	

	
	def path_planning(self):
		if self.left:
			self.msg_out.angular.z=self.msg_in.angular.z
			self.msg_out.linear.x=-self.msg_in.angular.z*self.distance+self.msg_in.linear.x
		else:
			self.msg_out.angular.z=self.msg_in.angular.z
			self.msg_out.linear.x=self.msg_in.angular.z*self.distance+self.msg_in.linear.x

	
	def set_location(self,left):
		self.left=left
